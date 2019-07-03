/*
 * Copyright (C) 2018 GlobalLogic
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

#ifndef VENDOR_RENESAS_HAL_EVS_EVSENUMERATOR_H
#define VENDOR_RENESAS_HAL_EVS_EVSENUMERATOR_H

#include <android/hardware/automotive/evs/1.0/IEvsEnumerator.h>
#include <android/hardware/automotive/evs/1.0/IEvsCamera.h>

#include <list>


namespace android {
namespace hardware {
namespace automotive {
namespace evs {
namespace V1_0 {
namespace renesas {


class EvsCamera;    // from EvsCamera.h
class EvsDisplay;   // from EvsDisplay.h


class EvsEnumerator : public IEvsEnumerator {
public:
    // Methods from ::android::hardware::automotive::evs::V1_0::IEvsEnumerator follow.
    Return<void> getCameraList(getCameraList_cb _hidl_cb)  override;
    Return<sp<IEvsCamera>> openCamera(const hidl_string& cameraId) override;
    Return<void> closeCamera(const ::android::sp<IEvsCamera>& carCamera)  override;
    Return<sp<IEvsDisplay>> openDisplay()  override;
    Return<void> closeDisplay(const ::android::sp<IEvsDisplay>& display)  override;
    Return<DisplayState> getDisplayState()  override;

    // Implementation details
    EvsEnumerator();
    ~EvsEnumerator () override = default;

private:
    struct CameraRecord {
        CameraDesc desc;
        wp<EvsCamera> activeInstance;

        struct Dim {
            uint32_t width{0};
            uint32_t height{0};
        } dim;

        CameraRecord(const char *cameraId, uint32_t width, uint32_t height) : desc(){
            desc.cameraId = cameraId;
            dim.width = width;
            dim.height = height;
        }
    };

    static CameraRecord* findCameraById(const std::string& cameraId);

    // NOTE:  All members values are static so that all clients operate on the same state
    //        That is to say, this is effectively a singleton despite the fact that HIDL
    //        constructs a new instance for each client.
    static std::list<CameraRecord> sCameraList;

    static wp<EvsDisplay>          sActiveDisplay; // Weak pointer. Object destructs if client dies.

    static bool subdevCameraSetup(const char *subdev_name);

};

} // namespace renesas
} // namespace V1_0
} // namespace evs
} // namespace automotive
} // namespace hardware
} // namespace android

#endif  // VENDOR_RENESAS_HAL_EVS_EVSENUMERATOR_H
