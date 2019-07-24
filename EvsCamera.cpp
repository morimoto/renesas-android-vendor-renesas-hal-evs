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

#include "EvsCamera.h"
#include "EvsEnumerator.h"

#include <hardware/gralloc1.h>
#include <ui/GraphicBufferAllocator.h>
#include <ui/GraphicBufferMapper.h>
#include <sys/select.h>
#include <sys/time.h>
#include <vendor/renesas/graphics/composer/2.0/IComposer.h>

using vendor::renesas::graphics::composer::V2_0::IComposer;

namespace android {
namespace hardware {
namespace automotive {
namespace evs {
namespace V1_0 {
namespace renesas {

#define BYTES_PER_PIXEL 4
#define CAMERA_FORMAT V4L2_PIX_FMT_XBGR32
#define MIN_UNDEQUEUED_BUFFERS 4
#define MAX_BUFFERS_IN_FLIGHT 20
#define SELECT_TIMEOUT 3

#define BUFFER_FORMAT HAL_PIXEL_FORMAT_BGRA_8888
#define BUFFER_USAGE (GRALLOC1_PRODUCER_USAGE_GPU_RENDER_TARGET | GRALLOC1_PRODUCER_USAGE_CPU_WRITE_OFTEN | GRALLOC1_PRODUCER_USAGE_CPU_READ_OFTEN)

EvsCamera::EvsCamera(const char *id, uint32_t initWidth, uint32_t initHeight) :
        mFramesAllowed(0),
        mFramesInUse(0),
        mStreamState(STOPPED),
        mFd(-1),
        mBufType(V4L2_BUF_TYPE_VIDEO_CAPTURE),
        mMemType(V4L2_MEMORY_USERPTR) {

    ALOGD("EvsCamera instantiated");

    mDescription.cameraId = id;
    mWidth  = initWidth;
    mHeight = initHeight;
    mFormat = BUFFER_FORMAT;
    mUsage  = BUFFER_USAGE;

    /*
     * Needed for configuration of frame buffers and VIN output format
     * before streaming.
     */
    mComposer = IComposer::getService();
    mDisplayWidth = mComposer->getDisplayWidth();
    mDisplayHeight = mComposer->getDisplayHeight();

    if(!initialize(id)) {
        ALOGE("Failed to open v4l device %s\n", id);
    }
}


EvsCamera::~EvsCamera() {
    ALOGD("EvsCamera being destroyed");
    shutdown();
}


bool EvsCamera::initialize(const char* deviceName)
{

    if ((mFd = open(deviceName, O_RDWR)) == -1) {
        ALOGE("Error while opening device %s: %s", deviceName, strerror(errno));
        return false;
    }

    // Set our desired output format
    v4l2_format format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.pixelformat = CAMERA_FORMAT;
    format.fmt.pix.width = mWidth;
    format.fmt.pix.height = mHeight;
    format.fmt.pix.field = V4L2_FIELD_NONE;

    ALOGI("Requesting format %c%c%c%c (0x%08X) w=%u h=%u",
          ((char*)&format.fmt.pix.pixelformat)[0],
          ((char*)&format.fmt.pix.pixelformat)[1],
          ((char*)&format.fmt.pix.pixelformat)[2],
          ((char*)&format.fmt.pix.pixelformat)[3],
          format.fmt.pix.pixelformat,
          format.fmt.pix.width,
          format.fmt.pix.height);

    if (ioctl(mFd, VIDIOC_S_FMT, &format) < 0) {
        ALOGE("VIDIOC_S_FMT: %s", strerror(errno));
        return false;
    }

    // Report the current output format
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(mFd, VIDIOC_G_FMT, &format) == 0) {
        ALOGI("Current camera output format:  fmt=0x%X, %dx%d, pitch=%d",
               format.fmt.pix.pixelformat,
               format.fmt.pix.width,
               format.fmt.pix.height,
               format.fmt.pix.bytesperline);
    } else {
        ALOGE("VIDIOC_G_FMT: %s", strerror(errno));
        return false;
    }

    return true;
}


//
// This gets called if another caller "steals" ownership of the camera
//
void EvsCamera::shutdown()
{
    ALOGD("EvsCamera shutdown");
    // Make sure our output stream is cleaned up
    // (It really should be already)
    stopVideoStream();
    // Claim the lock while we work on internal state
    std::lock_guard <std::mutex> lock(mAccessLock);

    // Drop all the graphics buffers we've been using

    if (mBuffers.size() > 0) {
        GraphicBufferAllocator& alloc(GraphicBufferAllocator::get());
        for (auto&& rec : mBuffers) {
            if(rec.handle != nullptr) {
                if (rec.inUse) {
                    ALOGE("Error - releasing buffer despite remote ownership %p", rec.handle);
                }
                alloc.free(rec.handle);
                rec.handle = nullptr;
            }
        }
        mBuffers.clear();
    }

    // Put this object into an unrecoverable error state since somebody else
    // is going to own the underlying camera now
    mStreamState = DEAD;

    if (mFd != -1) {
        close(mFd);
        mFd = -1;
    }
}


int EvsCamera::registerBuffers()
{
    v4l2_buffer buffer;
    v4l2_requestbuffers rb;
    int ret = android::NO_ERROR;
    android::GraphicBufferMapper &mapper = android::GraphicBufferMapper::get();
    memset(&rb, 0, sizeof(struct v4l2_requestbuffers));

    rb.count = mFramesAllowed;
    rb.type = mBufType;
    rb.memory = mMemType;

    ret = ioctl(mFd, VIDIOC_REQBUFS, &rb);

    if (ret < 0) {
        ALOGE("Unable to allocate buffers: %s\n", strerror(errno));
        return ret;
    }

    for (unsigned int i = 0; i < rb.count; i++) {
        memset(&buffer, 0, sizeof(struct v4l2_buffer));
        buffer.index = i;
        buffer.type = mBufType;
        buffer.memory = mMemType;

        ret = ioctl(mFd, VIDIOC_QUERYBUF, &buffer);
        if (ret < 0) {
            ALOGE("Unable to query buffer: %s\n", strerror(errno));
            return ret;
        }

        void *pixels = nullptr;
        mapper.lock(mBuffers[i].handle,
               GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_SW_READ_OFTEN,
               android::Rect(mWidth, mHeight),
               &pixels);
        if (!pixels) {
            ALOGE("V4lAdapter failed to gain access to image buffer for writing");
            return android::INVALID_OPERATION;
        }

        const unsigned int length = buffer.length;
        memset(&buffer, 0, sizeof(struct v4l2_buffer));
        buffer.index = i;
        buffer.type = mBufType;
        buffer.memory = mMemType;
        buffer.m.userptr = (unsigned long) pixels;
        buffer.length = length;

        ret = ioctl(mFd, VIDIOC_QBUF, &buffer);
        if (ret < 0) {
            ALOGE("Unable to queue buffer[%d]: %s)\n", i, strerror(errno));
            return ret;
        }
    }
    return ret;
}


int EvsCamera::unregisterBuffers()
{
    GraphicBufferAllocator &alloc(GraphicBufferAllocator::get());
    for (auto&& rec : mBuffers) {
        // Is this record not in use, but holding a buffer that we can free?
        if (rec.handle != nullptr) {
            // Release buffer and update the record so we can recognize it as "empty"
            if(!rec.inUse) {
                android::GraphicBufferMapper &mapper = android::GraphicBufferMapper::get();
                mapper.unlock(rec.handle);
                alloc.free(rec.handle);
                rec.handle = nullptr;
                mFramesAllowed--;
            }
        }
    }
    return 0;
}

// Methods from ::android::hardware::automotive::evs::V1_0::IEvsCamera follow.
Return<void> EvsCamera::getCameraInfo(getCameraInfo_cb _hidl_cb) {
    ALOGD("getCameraInfo");
    // Send back our self description
    _hidl_cb(mDescription);
    return Void();
}


Return<EvsResult> EvsCamera::setMaxFramesInFlight(uint32_t bufferCount) {
    std::lock_guard<std::mutex> lock(mAccessLock);

    // If we've been displaced by another owner of the camera, then we can't do anything else
    if (mStreamState == DEAD) {
        ALOGE("ignoring setMaxFramesInFlight call when camera has been lost.");
        return EvsResult::OWNERSHIP_LOST;
    }

    // We cannot function without at least one video buffer to send data
    if (bufferCount < 1) {
        ALOGE("Ignoring setMaxFramesInFlight with less than one buffer requested");
        return EvsResult::INVALID_ARG;
    }

    // Update our internal state
    if (setAvailableFrames_Locked(bufferCount + MIN_UNDEQUEUED_BUFFERS)) {
        return EvsResult::OK;
    } else {
        return EvsResult::BUFFER_NOT_AVAILABLE;
    }
}


Return<EvsResult> EvsCamera::startVideoStream(const ::android::sp<IEvsCameraStream>& stream)  {
    ALOGD("startVideoStream");
    std::lock_guard<std::mutex> lock(mAccessLock);

    // If we've been displaced by another owner of the camera, then we can't do anything else
    if (mStreamState == DEAD) {
        ALOGE("ignoring startVideoStream call when camera has been lost.");
        return EvsResult::OWNERSHIP_LOST;
    }
    if (mStreamState != STOPPED) {
        ALOGE("ignoring startVideoStream call when a stream is already running.");
        return EvsResult::STREAM_ALREADY_RUNNING;
    }

    // If the client never indicated otherwise, configure ourselves for a single streaming buffer
    if (mFramesAllowed < 1) {
        if (!setAvailableFrames_Locked(1 + MIN_UNDEQUEUED_BUFFERS)) {
            ALOGE("Failed to start stream because we couldn't get a graphics buffer");
            return EvsResult::BUFFER_NOT_AVAILABLE;
        }
    }

    if(registerBuffers() != android::NO_ERROR) {
        return EvsResult::BUFFER_NOT_AVAILABLE;
    }
    if(!startStreaming(true)){
        unregisterBuffers();
        return EvsResult::BUFFER_NOT_AVAILABLE;
    }
    mStream = stream;

    // Start the frame generation thread
    mStreamState = RUNNING;
    mCaptureThread = std::thread([this](){ generateFrames(); });

    return EvsResult::OK;
}


bool EvsCamera::startStreaming(bool enable)
{
    ALOGD("start streaming %d", enable);
    if (ioctl(mFd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &mBufType) < 0) {
        ALOGE("Unable to %s streaming: %s.\n", enable ? "start" : "stop", strerror(errno));
        return false;
    }
    return true;
}

Return<void> EvsCamera::doneWithFrame(const BufferDesc& buffer)  {
    {
        std::lock_guard <std::mutex> lock(mAccessLock);
        if (buffer.memHandle == nullptr) {
            ALOGE("ignoring doneWithFrame called with null handle");
        } else if (buffer.bufferId >= mBuffers.size()) {
            ALOGE("ignoring doneWithFrame called with invalid bufferId %d (max is %zu)",
                  buffer.bufferId, mBuffers.size()-1);
        } else if (!mBuffers[buffer.bufferId].inUse) {
            ALOGE("ignoring doneWithFrame called on frame %d which is already free",
                  buffer.bufferId);
        } else {
            if(mFramesAllowed <= 0 || mFramesInUse <= 0) {
                // Should never happen
                ALOGE("Error in doneWithFrame(). Received a frame when no frames were allowed or in use.");
                return Void();
            }
            if(mStreamState == STOPPED) {
                //Release buffer that was in use
                GraphicBufferAllocator &alloc(GraphicBufferAllocator::get());
                alloc.free(mBuffers[buffer.bufferId].handle);
                mBuffers[buffer.bufferId].handle = nullptr;
                mFramesAllowed--;
            } else {
                // Mark the frame as available
                queueBuffer(buffer.bufferId);
                mBuffers[buffer.bufferId].inUse = false;
            }
            mFramesInUse--;
        }
    }
    mBufferCond.notify_one();
    return Void();
}


Return<void> EvsCamera::stopVideoStream()  {
    ALOGD("stopVideoStream");
    std::unique_lock <std::mutex> lock(mAccessLock);

    if (mStreamState == RUNNING) {
        // Tell the GenerateFrames loop we want it to stop
        mStreamState = STOPPING;

        // Block outside the mutex until the "stop" flag has been acknowledged
        // We won't send any more frames, but the client might still get some already in flight
        ALOGD("Waiting for stream thread to end...");
        lock.unlock();
        mBufferCond.notify_one();
        mCaptureThread.join();
        lock.lock();

        startStreaming(false);
        unregisterBuffers();
        mStreamState = STOPPED;
        mStream = nullptr;
        ALOGD("Stream marked STOPPED.");
    }
    return Void();
}


Return<int32_t> EvsCamera::getExtendedInfo(uint32_t opaqueIdentifier)  {
    ALOGV("getExtendedInfo");

    switch(opaqueIdentifier) {
        case WIDTH:
            return mWidth;
        case HEIGHT:
            return mHeight;
        default:
            ALOGE("setExtendedInfo: Unknown Identifier %d", opaqueIdentifier);
            // Return zero by default as required by the spec
            return 0;
    }
}


Return<EvsResult> EvsCamera::setExtendedInfo(uint32_t /*opaqueIdentifier*/, int32_t /*opaqueValue*/)  {
    ALOGV("setExtendedInfo");
    std::lock_guard<std::mutex> lock(mAccessLock);

    // If we've been displaced by another owner of the camera, then we can't do anything else
    if (mStreamState == DEAD) {
        ALOGE("ignoring setExtendedInfo call when camera has been lost.");
        return EvsResult::OWNERSHIP_LOST;
    }
    // We don't store any device specific information in this implementation
    return EvsResult::INVALID_ARG;
}


bool EvsCamera::setAvailableFrames_Locked(unsigned bufferCount) {
    if (bufferCount < 1) {
        ALOGE("Ignoring request to set buffer count to zero");
        return false;
    }
    if (bufferCount > MAX_BUFFERS_IN_FLIGHT) {
        ALOGE("Rejecting buffer request in excess of internal limit");
        return false;
    }

    if(mStreamState == RUNNING) {
        ALOGE("Rejecting buffer request. Buffers allready initialized");
        return false;
    }

    // Is an increase required?
    if (mFramesAllowed < bufferCount) {
        // An increase is required
        const unsigned needed = bufferCount - mFramesAllowed;
        ALOGI("Allocating %d buffers for camera frames", needed);

        const unsigned added = increaseAvailableFrames_Locked(needed);
        if (added != needed) {
            // If we didn't add all the frames we needed, then roll back to the previous state
            ALOGE("Rolling back to previous frame queue size");
            decreaseAvailableFrames_Locked(added);
            return false;
        }
    } else if (mFramesAllowed > bufferCount) {
        // A decrease is required
        const unsigned framesToRelease = mFramesAllowed - bufferCount;
        ALOGI("Returning %d camera frame buffers", framesToRelease);

        const unsigned released = decreaseAvailableFrames_Locked(framesToRelease);
        if (released != framesToRelease) {
            // This shouldn't happen with a properly behaving client because the client
            // should only make this call after returning sufficient outstanding buffers
            // to allow a clean resize.
            ALOGE("Buffer queue shrink failed -- too many buffers currently in use?");
        }
    }
    return true;
}


unsigned EvsCamera::increaseAvailableFrames_Locked(unsigned numToAdd) {
    // Acquire the graphics buffer allocator
    GraphicBufferAllocator &alloc(GraphicBufferAllocator::get());

    unsigned added = 0;

    while (added < numToAdd) {
        buffer_handle_t memHandle = nullptr;
        status_t result = alloc.allocate(mWidth, mHeight,
                                         mFormat, 1,
                                         mUsage,
                                         &memHandle, &mStride, 0, "EvsCamera");
        if (result != NO_ERROR) {
            ALOGE("Error %d allocating %d x %d graphics buffer", result, mWidth, mHeight);
            break;
        }
        if (!memHandle) {
            ALOGE("We didn't get a buffer handle back from the allocator");
            break;
        }

        // Find a place to store the new buffer
        bool stored = false;
        for (auto&& rec : mBuffers) {
            if (rec.handle == nullptr) {
                // Use this existing entry
                rec.handle = memHandle;
                rec.inUse = false;
                stored = true;
                break;
            }
        }
        if (!stored) {
            // Add a BufferRecord wrapping this handle to our set of available buffers
            mBuffers.emplace_back(memHandle);
        }

        mFramesAllowed++;
        added++;
    }

    return added;
}


unsigned EvsCamera::decreaseAvailableFrames_Locked(unsigned numToRemove) {
    // Acquire the graphics buffer allocator
    GraphicBufferAllocator &alloc(GraphicBufferAllocator::get());

    if (numToRemove > mFramesAllowed) {
        ALOGW("%s numToRemove > mFramesAllowed", __func__);
        numToRemove = mFramesAllowed;
    }

    unsigned removed = 0;

    for (auto&& rec : mBuffers) {
        // Is this record not in use, but holding a buffer that we can free?
        if ((rec.inUse == false) && (rec.handle != nullptr)) {
            // Release buffer and update the record so we can recognize it as "empty"
            alloc.free(rec.handle);
            rec.handle = nullptr;

            mFramesAllowed--;
            removed++;

            if (removed == numToRemove) {
                break;
            }
        }
    }

    return removed;
}


// This is the asynchronous frame generation thread that runs in parallel with the
// main serving thread.  There is one for each active camera instance.
void EvsCamera::generateFrames() {
    ALOGD("Frame generation loop started");
    while (true) {
        bool timeForFrame = false;
        // Lock scope for updating shared state
        {
            std::unique_lock<std::mutex> lock(mAccessLock);
            if (mStreamState != RUNNING) {
                // Break out of our main thread loop
                ALOGW("Break out of our main thread loop\n");
                break;
            }

            // Are we allowed to issue another buffer?
            if (mFramesInUse >= mFramesAllowed - MIN_UNDEQUEUED_BUFFERS) {
                // Can't do anything right now -- skip this frame
                // Allways keep at least MIN_UNDEQUEUED_BUFFERS buffer with V4L
                mBufferCond.wait(lock);
            } else {
                timeForFrame = true;
                mFramesInUse++;
            }
        }

        if (timeForFrame) {
            int idx = dequeueBuffer();
            if(idx == -1) {
                // Can not dequeue buffer
                mFramesInUse--;
                break;
            }

            BufferDesc buff = {
                .width      = mWidth,
                .height     = mHeight,
                .stride     = mStride,
                .pixelSize  = BYTES_PER_PIXEL,
                .format     = HAL_PIXEL_FORMAT_RGBA_8888,
                .usage      = mUsage,
                .bufferId   = (uint32_t) idx,
                .memHandle  = mBuffers[idx].handle
            };

            mBuffers[idx].inUse = true;

            // Issue the (asynchronous) callback to the client -- can't be holding the lock
            auto result = mStream->deliverFrame(buff);

            if (!result.isOk()) {
                // This can happen if the client dies and is likely unrecoverable.
                // To avoid consuming resources generating failing calls, we stop sending
                // frames.  Note, however, that the stream remains in the "STREAMING" state
                // until cleaned up on the main thread.
                ALOGE("Frame delivery call failed in the transport layer.");

                // Since we didn't actually deliver it, mark the frame as available
                std::lock_guard<std::mutex> lock(mAccessLock);
                mBuffers[idx].inUse = false;
                queueBuffer(idx);
                mFramesInUse--;
                break;
            }
        }
    }

    // If we've been asked to stop, send one last NULL frame to signal the actual end of stream
    BufferDesc nullBuff = {};
    ALOGE("Delivering end of stream marker");
    auto result = mStream->deliverFrame(nullBuff);

    if (!result.isOk()) {
        ALOGE("Error delivering end of stream marker");
    }
    return;
}

int EvsCamera::dequeueBuffer() {
    int ret = android::NO_ERROR;
    android::GraphicBufferMapper &mapper = android::GraphicBufferMapper::get();
    v4l2_buffer buf;
    memset(&buf, 0, sizeof (struct v4l2_buffer));
    buf.type = mBufType;
    buf.memory = mMemType;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(mFd, &rfds);

    struct timeval tv = {
        .tv_sec = SELECT_TIMEOUT,
        .tv_usec = 0
    };

    if(select(mFd + 1, &rfds, NULL, NULL,  &tv) != 1) {
        ALOGE("ERROR: data not available");
        return -1;
    }

    ret = ioctl(mFd, VIDIOC_DQBUF, &buf);
    if (ret < 0) {
        ALOGE("Unable to dequeue buffer[%d]: %s \n", buf.index,  strerror(errno));
        return -1;
    }
    mapper.unlock(mBuffers[buf.index].handle);
    return buf.index;
}

void EvsCamera::queueBuffer(int idx) {
    int ret = android::NO_ERROR;
    v4l2_buffer buf;
    void *pixels = nullptr;
    android::GraphicBufferMapper &mapper = android::GraphicBufferMapper::get();
    mapper.lock(mBuffers[idx].handle,
                GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_SW_READ_OFTEN,
                android::Rect(mWidth, mHeight),
                &pixels);

    if (!pixels) {
            ALOGE("Failed to gain access to image buffer for writing");
        return;
    }
    memset(&buf, 0, sizeof (struct v4l2_buffer));
    buf.type = mBufType;
    buf.memory = mMemType;
    buf.index = idx;
    buf.m.userptr = (unsigned long) pixels;
    buf.length = mWidth * mHeight * BYTES_PER_PIXEL;

    ret = ioctl(mFd, VIDIOC_QBUF, &buf);
    if (ret < 0) {
        ALOGE("Unable to queue buffer[%d]: userptr: %p %s \n", idx, pixels,  strerror(errno));
    }
}

} // namespace renesas
} // namespace V1_0
} // namespace evs
} // namespace automotive
} // namespace hardware
} // namespace android
