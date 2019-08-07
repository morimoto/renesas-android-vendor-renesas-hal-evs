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

#ifndef VENDOR_RENESAS_HAL_EVS_EVSDISPLAY_H
#define VENDOR_RENESAS_HAL_EVS_EVSDISPLAY_H

#include <android/hardware/automotive/evs/1.0/IEvsDisplay.h>
#include <gui/IConsumerListener.h>
#include <binder/ProcessState.h>
#include <vendor/renesas/graphics/composer/2.0/IComposer.h>

#include <vector>

namespace android {
namespace hardware {
namespace automotive {
namespace evs {
namespace V1_0 {
namespace renesas {

using vendor::renesas::graphics::composer::V2_0::IComposer;

class EvsDisplay : public IEvsDisplay {
public:
    // Methods from ::android::hardware::automotive::evs::V1_0::IEvsDisplay follow.
    Return<void> getDisplayInfo(getDisplayInfo_cb _hidl_cb)  override;
    Return<EvsResult> setDisplayState(DisplayState state)  override;
    Return<DisplayState> getDisplayState()  override;
    Return<void> getTargetBuffer(getTargetBuffer_cb _hidl_cb)  override;
    Return<EvsResult> returnTargetBufferForDisplay(const BufferDesc& buffer)  override;

    // Implementation details
    EvsDisplay();
    ~EvsDisplay() override;

    void forceShutdown();   // This gets called if another caller "steals" ownership of the display

private:

    enum CameraStreamDisplay: int8_t {
        DISPLAY_PRIMARY = 0,
        DISPLAY_EXTERNAL,
        DISPLAY_SECOND_EXTERNAL,
        ALL_DISPLAYS
    } mCameraDisplay = DISPLAY_PRIMARY;

    enum BufferState: uint32_t {
        NOT_ALLOCATED = 0,
        FREE,
        DEQUEUED
    };

    struct EVSBuffer{
        BufferDesc buffer;
        buffer_handle_t handle;
        int ionFd;
        BufferState state;
    };

    DisplayDesc     mInfo           = {};

    DisplayState    mRequestedState = DisplayState::NOT_VISIBLE;

    std::mutex      mAccessLock;

    int32_t mDisplayWidth;
    int32_t mDisplayHeight;

    std::vector<EVSBuffer> mBuffers;
    int mCurrentBuffer = 0;
    sp<IComposer> mComposer;

    void init();
};

} // namespace renesas
} // namespace V1_0
} // namespace evs
} // namespace automotive
} // namespace hardware
} // namespace android

#endif  // VENDOR_RENESAS_HAL_EVS_EVSDISPLAY_H
