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

#include "EvsEnumerator.h"
#include "EvsCamera.h"
#include "EvsDisplay.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace android {
namespace hardware {
namespace automotive {
namespace evs {
namespace V1_0 {
namespace renesas {


// NOTE:  All members values are static so that all clients operate on the same state
//        That is to say, this is effectively a singleton despite the fact that HIDL
//        constructs a new instance for each client.
std::list<EvsEnumerator::CameraRecord>   EvsEnumerator::sCameraList;
wp<EvsDisplay>                           EvsEnumerator::sActiveDisplay;


EvsEnumerator::EvsEnumerator() {
    ALOGD("EvsEnumerator created");
    /*
    For HDMI connector:
    v4l-subdev0 --- rcar_csi2 feaa0000.csi2
    v4l-subdev1 --- adv748x 4-0070 hdmi
    v4l-subdev2 --- adv748x 4-0070 txa
    */

    /* subdev for HDMI camera input */
    const char *subdev_name = "/dev/v4l-subdev1";
    // Constant predefined list of EVS cameras in the "/dev" filesystem.
    const std::vector<const char *> cameraNames {
        "/dev/video3",
        "/dev/video2",
        "/dev/video1",
        "/dev/video0"
    };

    if(access(subdev_name, F_OK ) != -1){
        if(subdevCameraSetup(subdev_name)) {
            ALOGD("Successfully setup camera %s", subdev_name);
        } else {
            ALOGE("Error during setup %s", subdev_name);
        }
    } else {
        // Probe each camera one by one and use only that which can be opened and requested.
        for (auto cn : cameraNames) {
            const int fd = open(cn, O_RDWR);
            if (-1 == fd) {
                ALOGE("Error while opening device %s: %s.", cn, strerror(errno));
                continue;
            }

            v4l2_format format;
            std::memset(&format, 0x00, sizeof(format));
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(fd, VIDIOC_G_FMT, &format) < 0) {
                ALOGE("VIDIOC_G_FMT: %s for device %s.", strerror(errno), cn);
            } else {
                sCameraList.emplace_back(cn, format.fmt.pix.width, format.fmt.pix.height);
                ALOGI("Camera %s (%ux%u) is successfully probed.",
                        cn, format.fmt.pix.width, format.fmt.pix.height);
            }

            close(fd);
        }
    }
}

/*
 * This method send VIDIOC_SUBDEV_G_FMT ioctl through subdev interface to
 * receive current camera format. In case of correct answer it creates
 * CameraRecord in EvsEenumerator::sCameraList with preferred camera parameters.
 * If ioctl return zeros in width and height fields - it means that camera not
 * connected.
 */
bool EvsEnumerator::subdevCameraSetup(const char* subdev_name) {
    /*
    Set input resolution and format.
    Here we are opening the appropriate sub-device node to do this.
    We know the sub-device number. However, it might not be known always.

    For HDMI connector:
    v4l-subdev0 --- rcar_csi2 feaa0000.csi2
    v4l-subdev1 --- adv748x 4-0070 hdmi
    v4l-subdev2 --- adv748x 4-0070 txa

    For CVBS connector:
    v4l-subdev3 --- rcar_csi2 fea80000.csi2
    v4l-subdev4 --- adv748x 4-0070 afe
    v4l-subdev5 --- adv748x 4-0070 txb

    Flow of digital data:
    HDMI Connector -{HD digital Signal}-> [ ADV7482 HDMI --> ADV7482 TXA ] -{RGB-888 24-bit data}-> CSI40 --> VIN --> V4L2 /dev/videoX
    RCA  Connector -{NTSC/PAL Signal}->   [ ADV7482 AFE  --> ADV7482 TXB ] -{YCbCr422 8-bit data}-> CSI20 --> VIN --> V4L2 /dev/videoX

    PADs:
    [0]ADV7482  HDMI[1] --> [0]ADV7482 TXA[1] --> [1-4]CSI40[0] --> [0]VIN
    [0-7]ADV7482 AFE[8] --> [0]ADV7482 TXB[1] --> [1-4]CSI20[0] --> [0]VIN

    Supports clipping and scaling using the hardware function of the VIN
    The clipping area is set with VIDIOC_S_CROP interface of V4L2
    The output image size is set by VIDIOC_S_FMT interface of V4L2
    VIN input image --> Clipping(VIDIOC_S_CROP) --> Output image(VIDIOC_S_FMT)
    */

    v4l2_subdev_format sub_format = {};
    sub_format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sub_format.format.code = MEDIA_BUS_FMT_RGB888_1X24;
    sub_format.format.field = V4L2_FIELD_NONE;
    sub_format.format.colorspace = V4L2_COLORSPACE_SRGB;
    sub_format.format.width = 0;
    sub_format.format.height = 0;
    sub_format.pad = 1;

    int fd = -1;
    if( (fd = open(subdev_name, O_RDWR)) == -1) {
        ALOGE("Error while opening device %s: %s", subdev_name, strerror(errno));
        return false;
    }

    if (ioctl(fd, VIDIOC_SUBDEV_G_FMT, &sub_format) < 0) {
        ALOGE("%s VIDIOC_SUBDEV_G_FMT: %s", subdev_name, strerror(errno));
        return false;
    }
    close(fd);

    ALOGD("Parameters received from camera is w=%u h=%u\n",
                sub_format.format.width, sub_format.format.height);
    if(sub_format.format.height && sub_format.format.width){
        /* Set format on source pad of CSI2 */
        sub_format.pad = 0;
        if ((fd = open("/dev/v4l-subdev0", O_RDWR)) == -1) {
            ALOGE("Error while opening device %s: %s", "/dev/v4l-subdev0", strerror(errno));
            return false;
        }
        if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sub_format) < 0) {
            ALOGE("/dev/v4l-subdev0 VIDIOC_SUBDEV_S_FMT: %s", strerror(errno));
            close(fd);
            return false;
        }
        close(fd);

        /* Set format on source (output, ADV748X_HDMI_SOURCE) pad of ADV7482 HDMI */
        sub_format.pad = 1;
        if ((fd = open("/dev/v4l-subdev1", O_RDWR)) == -1) {
            ALOGE("Error while opening device %s: %s", "/dev/v4l-subdev1", strerror(errno));
            return false;
        }
        if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sub_format) < 0) {
            ALOGE("/dev/v4l-subdev1 VIDIOC_SUBDEV_S_FMT: %s", strerror(errno));
            close(fd);
            return false;
        }
        close(fd);

        /* Set format on sink (input, ADV748X_CSI2_SINK) pad of ADV7482 TXA */
        sub_format.pad = 0;
        if ((fd = open("/dev/v4l-subdev2", O_RDWR)) == -1) {
            ALOGE("Error while opening device %s: %s", "/dev/v4l-subdev2", strerror(errno));
            return false;
        }
        if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sub_format) < 0) {
            ALOGE("/dev/v4l-subdev2 VIDIOC_SUBDEV_S_FMT: %s", strerror(errno));
            close(fd);
            return false;
        }
        close(fd);

        sCameraList.emplace_back("/dev/video0", sub_format.format.width, sub_format.format.height);
    } else {
        ALOGD("No camera found with %s", subdev_name);
        return false;
    }
    return true;
}

// Methods from ::android::hardware::automotive::evs::V1_0::IEvsEnumerator follow.
Return<void> EvsEnumerator::getCameraList(getCameraList_cb _hidl_cb)  {
    ALOGD("getCameraList");

    const unsigned numCameras = sCameraList.size();

    // Build up a packed array of CameraDesc for return
    hidl_vec<CameraDesc> hidlCameras;
    hidlCameras.resize(numCameras);
    unsigned i = 0;
    for (const auto& cam : sCameraList) {
        hidlCameras[i++] = cam.desc;
    }

    // Send back the results
    ALOGD("reporting %zu cameras available", hidlCameras.size());
    _hidl_cb(hidlCameras);

    // HIDL convention says we return Void if we sent our result back via callback
    return Void();
}


Return<sp<IEvsCamera>> EvsEnumerator::openCamera(const hidl_string& cameraId) {
    ALOGD("openCamera");

    // Is this a recognized camera id?
    CameraRecord *pRecord = findCameraById(cameraId);
    if (!pRecord) {
        ALOGE("Requested camera %s not found", cameraId.c_str());
        return nullptr;
    }

    // Has this camera already been instantiated by another caller?
    sp<EvsCamera> pActiveCamera = pRecord->activeInstance.promote();
    if (pActiveCamera != nullptr) {
        ALOGW("Killing previous camera because of new caller");
        closeCamera(pActiveCamera);
    }

    // Construct a camera instance for the caller
    pActiveCamera = new EvsCamera(cameraId.c_str(), pRecord->dim.width, pRecord->dim.height);
    pRecord->activeInstance = pActiveCamera;
    if (pActiveCamera == nullptr) {
        ALOGE("Failed to allocate new EvsCamera object for %s\n", cameraId.c_str());
    }

    return pActiveCamera;
}


Return<void> EvsEnumerator::closeCamera(const ::android::sp<IEvsCamera>& pCamera) {
    ALOGD("closeCamera");

    if (pCamera == nullptr) {
        ALOGE("Ignoring call to closeCamera with null camera ptr");
        return Void();
    }

    // Get the camera id so we can find it in our list
    std::string cameraId;
    pCamera->getCameraInfo([&cameraId](CameraDesc desc) {
                               cameraId = desc.cameraId;
                           }
    );

    // Find the named camera
    CameraRecord * const pRecord = findCameraById(cameraId);

    // Is the display being destroyed actually the one we think is active?
    if (!pRecord) {
        ALOGE("Asked to close a camera whose name isn't recognized");
    } else {
        sp<EvsCamera> pActiveCamera = pRecord->activeInstance.promote();

        if (pActiveCamera == nullptr) {
            ALOGE("Somehow a camera is being destroyed when the enumerator didn't know one existed");
        } else if (pActiveCamera != pCamera) {
            // This can happen if the camera was aggressively reopened, orphaning this previous instance
            ALOGW("Ignoring close of previously orphaned camera - why did a client steal?");
        } else {
            // Drop the active camera
            pActiveCamera->shutdown();
            pRecord->activeInstance = nullptr;
        }
    }

    return Void();
}


Return<sp<IEvsDisplay>> EvsEnumerator::openDisplay() {
    ALOGD("openDisplay");

    // If we already have a display active, then we need to shut it down so we can
    // give exclusive access to the new caller.
    sp<EvsDisplay> pActiveDisplay = sActiveDisplay.promote();
    if (pActiveDisplay != nullptr) {
        ALOGW("Killing previous display because of new caller");
        closeDisplay(pActiveDisplay);
    }

    // Create a new display interface and return it
    pActiveDisplay = new EvsDisplay();
    sActiveDisplay = pActiveDisplay;

    ALOGD("Returning new EvsDisplay object %p", pActiveDisplay.get());
    return pActiveDisplay;
}


Return<void> EvsEnumerator::closeDisplay(const ::android::sp<IEvsDisplay>& pDisplay) {
    ALOGD("closeDisplay");

    // Do we still have a display object we think should be active?
    sp<EvsDisplay> pActiveDisplay = sActiveDisplay.promote();
    if (pActiveDisplay == nullptr) {
        ALOGE("Somehow a display is being destroyed when the enumerator didn't know one existed");
    } else if (sActiveDisplay != pDisplay) {
        ALOGW("Ignoring close of previously orphaned display - why did a client steal?");
    } else {
        // Drop the active display
        pActiveDisplay->forceShutdown();
        sActiveDisplay = nullptr;
    }

    return Void();
}


Return<DisplayState> EvsEnumerator::getDisplayState()  {
    ALOGD("getDisplayState");

    // Do we still have a display object we think should be active?
    sp<IEvsDisplay> pActiveDisplay = sActiveDisplay.promote();
    if (pActiveDisplay != nullptr) {
        return pActiveDisplay->getDisplayState();
    } else {
        return DisplayState::NOT_OPEN;
    }
}

EvsEnumerator::CameraRecord* EvsEnumerator::findCameraById(const std::string& cameraId) {
    // Find the named camera
    for (auto &&cam : sCameraList) {
        if (cam.desc.cameraId == cameraId) {
            // Found a match!
            return &cam;
        }
    }

    // We didn't find a match
    return nullptr;
}

} // namespace renesas
} // namespace V1_0
} // namespace evs
} // namespace automotive
} // namespace hardware
} // namespace android
