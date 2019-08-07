/*
 * Copyright (C) 2019 GlobalLogic
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "EvsHAL"

#include "EvsDisplay.h"
#include <ui/GraphicBufferAllocator.h>
#include <cutils/properties.h>
#include <hardware/gralloc1.h>
#include <img_gralloc_common_public.h>
#include <cmath>

#include <vendor/renesas/graphics/composer/2.0/IComposer.h>

using vendor::renesas::graphics::composer::V2_0::IComposer;

namespace android {
namespace hardware {
namespace automotive {
namespace evs {
namespace V1_0 {
namespace renesas {


#define BYTES_PER_PIXEL 4
#define NUM_BUFFERS (3)
#define BUFFER_FORMAT HAL_PIXEL_FORMAT_BGRX_8888
#define BUFFER_USAGE (GRALLOC1_PRODUCER_USAGE_GPU_RENDER_TARGET | GRALLOC1_PRODUCER_USAGE_CPU_WRITE_OFTEN)

EvsDisplay::EvsDisplay() {
    ALOGD("EvsDisplay instantiated");
    init();

}

EvsDisplay::~EvsDisplay() {
    ALOGD("EvsDisplay being destroyed");
    forceShutdown();
}

void EvsDisplay::init() {
    mComposer = IComposer::getService();

    mInfo.displayId = "Renesas EVS display";
    mInfo.vendorFlags = 0;

    for (uint32_t i = 0; i < NUM_BUFFERS; i++) {
        EVSBuffer buf = {
            .buffer.bufferId = i,
            .handle = nullptr,
            .ionFd = -1,
            .state = NOT_ALLOCATED
        };
        mBuffers.push_back(buf);
    }

    mDisplayWidth = mComposer->getDisplayWidth();
    mDisplayHeight = mComposer->getDisplayHeight();
}

/**
 * This gets called if another caller "steals" ownership of the display
 */
void EvsDisplay::forceShutdown()
{
    ALOGD("EvsDisplay forceShutdown");
    std::lock_guard<std::mutex> lock(mAccessLock);

    // If the buffer isn't being held by a remote client, release it now as an
    // optimization to release the resources more quickly than the destructor might
    // get called.
    for (auto& buf : mBuffers) {
        if (buf.buffer.memHandle) {
            // Report if we're going away while a buffer is outstanding
            if (buf.state == DEQUEUED) {
                ALOGE("EvsDisplay going down while client is holding a buffer");
            }

            // Drop the graphics buffer we've been using
            GraphicBufferAllocator& alloc(GraphicBufferAllocator::get());
            alloc.free(buf.buffer.memHandle);
            buf.buffer.memHandle = nullptr;
            buf.state = NOT_ALLOCATED;
        }
    }

    // Put this object into an unrecoverable error state since somebody else
    // is going to own the display now.
    mRequestedState = DisplayState::DEAD;
}

/**
 * Returns basic information about the EVS display provided by the system.
 * See the description of the DisplayDesc structure for details.
 */
Return<void> EvsDisplay::getDisplayInfo(getDisplayInfo_cb _hidl_cb)  {
    // Send back our self description
    _hidl_cb(mInfo);
    return Void();
}

/**
 * Clients may set the display state to express their desired state.
 * The HAL implementation must gracefully accept a request for any state
 * while in any other state, although the response may be to ignore the request.
 * The display is defined to start in the NOT_VISIBLE state upon initialization.
 * The client is then expected to request the VISIBLE_ON_NEXT_FRAME state, and
 * then begin providing video.  When the display is no longer required, the client
 * is expected to request the NOT_VISIBLE state after passing the last video frame.
 */
Return<EvsResult> EvsDisplay::setDisplayState(DisplayState state) {
    ALOGD("setDisplayState");
    std::lock_guard<std::mutex> lock(mAccessLock);

    if (state == DisplayState::NOT_VISIBLE) {
        if (mComposer != nullptr) {
            mComposer->setEVSCameraData(nullptr, mCameraDisplay);
            ALOGI("returnTargetBufferForDisplay | camera stop");
        }
    }

    if (mRequestedState == DisplayState::DEAD) {
        // This object no longer owns the display -- it's been superceeded!
        return EvsResult::OWNERSHIP_LOST;
    }

    // Ensure we recognize the requested state so we don't go off the rails
    if (state < DisplayState::NUM_STATES) {
        // Record the requested state
        mRequestedState = state;
        return EvsResult::OK;
    }
    else {
        // Turn off the display if asked for an unrecognized state
        mRequestedState = DisplayState::NOT_VISIBLE;
        return EvsResult::INVALID_ARG;
    }
}

/**
 * The HAL implementation should report the actual current state, which might
 * transiently differ from the most recently requested state.  Note, however, that
 * the logic responsible for changing display states should generally live above
 * the device layer, making it undesirable for the HAL implementation to
 * spontaneously change display states.
 */
Return<DisplayState> EvsDisplay::getDisplayState()  {
    ALOGD("getDisplayState");
    std::lock_guard<std::mutex> lock(mAccessLock);

    return mRequestedState;
}

/**
 * This call returns a handle to a frame buffer associated with the display.
 * This buffer may be locked and written to by software and/or GL.  This buffer
 * must be returned via a call to returnTargetBufferForDisplay() even if the
 * display is no longer visible.
 */
// TODO: We need to know if/when our client dies so we can get the buffer back! (blocked b/31632518)
Return<void> EvsDisplay::getTargetBuffer(getTargetBuffer_cb _hidl_cb)  {
    std::lock_guard<std::mutex> lock(mAccessLock);

    if (mRequestedState == DisplayState::DEAD) {
        ALOGE("Rejecting buffer request from object that lost ownership of the display.");
        BufferDesc nullBuff = {};
        _hidl_cb(nullBuff);
        return Void();
    }

    // If we don't already have a buffer, allocate one now
    if (mBuffers[mCurrentBuffer].state == NOT_ALLOCATED) {
        // Allocate the buffer that will hold our displayable image
        buffer_handle_t handle = nullptr;
        uint32_t stride;
        GraphicBufferAllocator& alloc(GraphicBufferAllocator::get());
        status_t result = alloc.allocate(mDisplayWidth, mDisplayHeight,
                                         BUFFER_FORMAT, 1,
                                         BUFFER_USAGE, &handle,
                                         &stride,
                                         0, "EvsDisplay");
        if (result != NO_ERROR) {
            ALOGE("Error %d allocating %d x %d graphics buffer",
                  result, mDisplayWidth, mDisplayHeight);
            BufferDesc nullBuff = {};
            _hidl_cb(nullBuff);
            return Void();
        }
        if (!handle) {
            ALOGE("We didn't get a buffer handle back from the allocator");
            BufferDesc nullBuff = {};
            _hidl_cb(nullBuff);
            return Void();
        }

        mBuffers[mCurrentBuffer].buffer.memHandle = handle;
        mBuffers[mCurrentBuffer].buffer.stride = stride;
        mBuffers[mCurrentBuffer].buffer.width = mDisplayWidth;
        mBuffers[mCurrentBuffer].buffer.height = mDisplayHeight;
        mBuffers[mCurrentBuffer].buffer.format = HAL_PIXEL_FORMAT_RGBA_8888;
        mBuffers[mCurrentBuffer].buffer.pixelSize = BYTES_PER_PIXEL;
        mBuffers[mCurrentBuffer].buffer.usage = BUFFER_USAGE;

        mBuffers[mCurrentBuffer].state = FREE;
        mBuffers[mCurrentBuffer].handle = handle;

        ALOGD("Allocated new buffer %p with stride %u", handle, stride);
    }

    _hidl_cb(mBuffers[mCurrentBuffer].buffer);
    mBuffers[mCurrentBuffer].state = DEQUEUED;

    mCurrentBuffer = (mCurrentBuffer + 1) % NUM_BUFFERS;
    return Void();
}

/**
 * This call tells the display that the buffer is ready for display.
 * The buffer is no longer valid for use by the client after this call.
 */
Return<EvsResult> EvsDisplay::returnTargetBufferForDisplay(const BufferDesc& buffer)  {
    std::lock_guard<std::mutex> lock(mAccessLock);

    // Nobody should call us with a null handle
    if (!buffer.memHandle.getNativeHandle()) {
        ALOGE ("returnTargetBufferForDisplay called without a valid buffer handle.\n");
        return EvsResult::INVALID_ARG;
    }

    if (buffer.bufferId >= NUM_BUFFERS) {
        ALOGE ("Got an unrecognized frame id.\n");
        return EvsResult::INVALID_ARG;
    }

    if (mBuffers[buffer.bufferId].state != DEQUEUED) {
        ALOGE ("A frame was returned with no outstanding frames.\n");
        return EvsResult::BUFFER_NOT_AVAILABLE;
    }

    // If we've been displaced by another owner of the display, then we can't do anything else
    if (mRequestedState == DisplayState::DEAD) {
        ALOGE ("EVSDisplay DisplayState::DEAD \n");
        return EvsResult::OWNERSHIP_LOST;
    }

    // If we were waiting for a new frame, this is it!
    if (mRequestedState == DisplayState::VISIBLE_ON_NEXT_FRAME) {
        mRequestedState = DisplayState::VISIBLE;
    }

    // Validate we're in an expected state
    if (mRequestedState != DisplayState::VISIBLE) {
        // We shouldn't get frames back when we're not visible.
        ALOGE ("Got an unexpected frame returned while not visible - ignoring.\n");
    } else {
        if (mComposer != nullptr) {
            android::hardware::hidl_handle hbuffer(reinterpret_cast<const native_handle_t*>(mBuffers[buffer.bufferId].handle));
            mComposer->setEVSCameraData(hbuffer, mCameraDisplay);
        } else {
            ALOGE("returnTargetBufferForDisplay | Composer start failed");
        }
    }
    return EvsResult::OK;
}

} // namespace renesas
} // namespace V1_0
} // namespace evs
} // namespace automotive
} // namespace hardware
} // namespace android
