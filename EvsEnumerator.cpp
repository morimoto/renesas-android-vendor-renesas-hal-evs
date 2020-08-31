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
#include <ui/DisplayConfig.h>
#include <ui/DisplayState.h>


namespace android {
namespace hardware {
namespace automotive {
namespace evs {
namespace V1_1 {
namespace renesas {


// NOTE:  All members values are static so that all clients operate on the same state
//        That is to say, this is effectively a singleton despite the fact that HIDL
//        constructs a new instance for each client.
std::list<EvsEnumerator::CameraRecord>   EvsEnumerator::sCameraList;
wp<EvsDisplay>                           EvsEnumerator::sActiveDisplay;
sp<IAutomotiveDisplayProxyService>       EvsEnumerator::sDisplayProxyService;
std::unordered_map<uint8_t, uint64_t>    EvsEnumerator::sDisplayPortList;


EvsEnumerator::EvsEnumerator(Platform platform, sp<IAutomotiveDisplayProxyService> windowService) {
    ALOGD("EvsEnumerator created");

    switch (platform) {
        case Platform::Salvator:
            if(!enumerateCamerasSalvator()) {
                ALOGE("Error during enumerate cameras.");
            }
            break;
        case Platform::Kingfisher:
            if(!enumerateCamerasKingfisher()) {
                ALOGE("Error during enumerate cameras.");
            }
            break;
        case Platform::Unknown:
        default:
            ALOGE("Unknown hardware environment!");
            break;
    }

    if (sDisplayProxyService == nullptr) {
        sDisplayProxyService = windowService;
    }

    enumerateDisplays();
}


// This method send VIDIOC_SUBDEV_G_FMT ioctl through subdev interface to
// receive current camera format. In case of correct answer it creates
// CameraRecord in EvsEenumerator::sCameraList with preferred camera parameters.
// If ioctl return zeros in width and height fields - it means that camera not
// connected.
bool EvsEnumerator::enumerateCamerasSalvator() {
    // Set input resolution and format.
    // Here we are opening the appropriate sub-device node to do this.
    // We know the sub-device number. However, it might not be known always.
    //
    // For HDMI connector:
    // v4l-subdev0 --- rcar_csi2 feaa0000.csi2
    // v4l-subdev2 --- adv748x 4-0070 hdmi
    // v4l-subdev3 --- adv748x 4-0070 txa
    //
    // For CVBS connector:
    // v4l-subdev4 --- rcar_csi2 fea80000.csi2
    // v4l-subdev1 --- adv748x 4-0070 afe
    // v4l-subdev5 --- adv748x 4-0070 txb
    //
    // Flow of digital data:
    // HDMI Connector -{HD digital Signal}-> [ ADV7482 HDMI --> ADV7482 TXA ] -{RGB-888 24-bit data}-> CSI40 --> VIN --> V4L2 /dev/videoX
    // RCA  Connector -{NTSC/PAL Signal}->   [ ADV7482 AFE  --> ADV7482 TXB ] -{YCbCr422 8-bit data}-> CSI20 --> VIN --> V4L2 /dev/videoX
    //
    // PADs:
    // [0]ADV7482  HDMI[1] --> [0]ADV7482 TXA[1] --> [1-4]CSI40[0] --> [0]VIN
    // [0-7]ADV7482 AFE[8] --> [0]ADV7482 TXB[1] --> [1-4]CSI20[0] --> [0]VIN
    //
    // Supports clipping and scaling using the hardware function of the VIN
    // The clipping area is set with VIDIOC_S_CROP interface of V4L2
    // The output image size is set by VIDIOC_S_FMT interface of V4L2
    // VIN input image --> Clipping(VIDIOC_S_CROP) --> Output image(VIDIOC_S_FMT)

    v4l2_subdev_format sub_format = {};
    sub_format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sub_format.format.code = MEDIA_BUS_FMT_RGB888_1X24;
    sub_format.format.field = V4L2_FIELD_NONE;
    sub_format.format.colorspace = V4L2_COLORSPACE_SRGB;
    sub_format.format.width = 0;
    sub_format.format.height = 0;
    sub_format.pad = 1;

    int fd = -1;

    // To enable a link use the media-ctl utility from v4l-utils package
    // media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':1 -> 'VIN0 output':0 [1]"
    // or
    // media-ctl -d /dev/media0 -l "1:1 -> 54:0 [1]"
    // IDs/indexes were received from here: media-ctl /dev/media0 -p
    if( (fd = open("/dev/media0", O_RDWR)) == -1) {
        ALOGE("Error while opening device %s: %s", "/dev/media0", strerror(errno));
        return false;
    }

    media_link_desc ulink = {};

    // source pad - 'rcar_csi2 feaa0000.csi2':1
    ulink.source.entity = 1;
    ulink.source.index = 1;
    ulink.source.flags = MEDIA_PAD_FL_SOURCE;

    // sink pad - 'VIN0 output':0
    ulink.sink.entity = 54;
    ulink.sink.index = 0;
    ulink.sink.flags = MEDIA_PAD_FL_SINK;

    ulink.flags = MEDIA_LNK_FL_ENABLED;

    if (ioctl(fd, MEDIA_IOC_SETUP_LINK, &ulink) < 0) {
        ALOGE("%s MEDIA_IOC_SETUP_LINK: %s", "/dev/media0", strerror(errno));
        close(fd);
        return false;
    }
    close(fd);


    // Setup video formats
    // media-ctl -d /dev/media0 -V "'rcar_csi2 feaa0000.csi2':1 [fmt:RGB888_1X24/1920x1080 field:none]"
    // media-ctl -d /dev/media0 -V "'adv748x 4-0070 hdmi':1 [fmt:RGB888_1X24/1920x1080 field:none]"
    // media-ctl -d /dev/media0 -V "'adv748x 4-0070 txa':1 [fmt:RGB888_1X24/1920x1080 field:none]"
    const char *subdev_name = "/dev/v4l-subdev2";
    if( (fd = open(subdev_name, O_RDWR)) == -1) {
        ALOGE("Error while opening device %s: %s", subdev_name, strerror(errno));
        close(fd);
        return false;
    }

    if (ioctl(fd, VIDIOC_SUBDEV_G_FMT, &sub_format) < 0) {
        ALOGE("%s VIDIOC_SUBDEV_G_FMT: %s", subdev_name, strerror(errno));
        close(fd);
        return false;
    }
    close(fd);

    ALOGD("Parameters received from camera is w=%u h=%u\n",
                sub_format.format.width, sub_format.format.height);
    if(sub_format.format.height && sub_format.format.width){
        // Set format on source pad of CSI2
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

        // Set format on source (output, ADV748X_HDMI_SOURCE) pad of ADV7482 HDMI
        sub_format.pad = 1;
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

        // Set format on sink (input, ADV748X_CSI2_SINK) pad of ADV7482 TXA
        sub_format.pad = 0;
        if ((fd = open("/dev/v4l-subdev3", O_RDWR)) == -1) {
            ALOGE("Error while opening device %s: %s", "/dev/v4l-subdev3", strerror(errno));
            return false;
        }
        if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sub_format) < 0) {
            ALOGE("/dev/v4l-subdev3 VIDIOC_SUBDEV_S_FMT: %s", strerror(errno));
            close(fd);
            return false;
        }
        close(fd);

        const uint32_t data [] {
                0
              , sub_format.format.width
              , sub_format.format.height
              , HAL_PIXEL_FORMAT_RGBA_8888
              , ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT
              , 30};
        camera_metadata_t * metadata = allocate_camera_metadata(/*entrys*/1
              , calculate_camera_metadata_entry_data_size(
                    get_camera_metadata_tag_type(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)
                  , sizeof(data)) );
        const int32_t err = add_camera_metadata_entry(metadata
              , ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS
              , data, sizeof(data));

        if (err) {
            metadata = nullptr;
            ALOGW("Failed to prepare metadata entry, ignored.");
        }

        sCameraList.emplace_back(metadata, "/dev/video0", sub_format.format.width, sub_format.format.height);
    } else {
        ALOGD("No camera found with %s", subdev_name);
        return false;
    }
    return true;
}


bool EvsEnumerator::enumerateCamerasKingfisher() {
    // Constant predefined list of EVS cameras in the "/dev" filesystem.
    // source pad - entity 1: 'rcar_csi2 feaa0000.csi2':1
    // source pad - entity 1: 'rcar_csi2 feaa0000.csi2':2
    // source pad - entity 1: 'rcar_csi2 feaa0000.csi2':3
    // source pad - entity 1: 'rcar_csi2 feaa0000.csi2':4
    // sink pad - entity 43: 'VIN0 output':0
    // sink pad - entity 47: 'VIN1 output':0
    // sink pad - entity 51: 'VIN2 output':0
    // sink pad - entity 55: 'VIN3 output':0
    struct {
        const char * devname;
        const char * subdevname;
        int source;
        int sink;
    } cameras [] = {
        {"/dev/video3", "/dev/v4l-subdev2", 4, 55},
        {"/dev/video2", "/dev/v4l-subdev3", 3, 51},
        {"/dev/video1", "/dev/v4l-subdev4", 2, 47},
        {"/dev/video0", "/dev/v4l-subdev5", 1, 43}
    };
    // media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':1 -> 'VIN0 output':0 [1]"
    // media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':2 -> 'VIN1 output':0 [1]"
    // media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':3 -> 'VIN2 output':0 [1]"
    // media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':4 -> 'VIN3 output':0 [1]"

    // media-ctl -d /dev/media0 -V "'rcar_csi2 feaa0000.csi2':1 [fmt:UYVY8_2X8/1280x800 field:none]"

    int fd = -1;
    if( (fd = open("/dev/media0", O_RDWR)) == -1) {
        ALOGE("Error while opening device %s: %s", "/dev/media0", strerror(errno));
        return false;
    }

    // Setup pipiline links.
    for (auto& [name, subname, source, sink] : cameras) {
        media_link_desc ulink = {};

        ulink.source.entity = 1;
        ulink.source.index = source;
        ulink.source.flags = MEDIA_PAD_FL_SOURCE;

        ulink.sink.entity = sink;
        ulink.sink.index = 0;
        ulink.sink.flags = MEDIA_PAD_FL_SINK;

        ulink.flags = MEDIA_LNK_FL_ENABLED;

        if (ioctl(fd, MEDIA_IOC_SETUP_LINK, &ulink) < 0) {
            ALOGE("%s MEDIA_IOC_SETUP_LINK: %s, for camera: %s.", "/dev/media0", strerror(errno), name);
            continue;
        }
    }
    close(fd);

    // - entity 15: ov10635 28-0063 (1 pad, 1 link)
    //              type V4L2 subdev subtype Sensor flags 0
    //              device node name /dev/v4l-subdev2
    //         pad0: Source
    //                 [fmt:UYVY8_2X8/1280x800@1/30 field:none colorspace:smpte170m
    //                  crop.bounds:(0,0)/1280x800
    //                  crop:(0,0)/1280x800]
    //                 -> "max9286 18-002c":3 [ENABLED,IMMUTABLE]
    v4l2_subdev_format sub_format = {};
    sub_format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sub_format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
    sub_format.format.field = V4L2_FIELD_NONE;
    sub_format.format.colorspace = V4L2_COLORSPACE_SMPTE170M;
    // Setup sensors.
    for (auto& [name, subname, source, sink] : cameras) {
        sub_format.format.width = 0;
        sub_format.format.height = 0;
        sub_format.pad = 0;

        if((fd = open(subname, O_RDWR)) == -1) {
            ALOGE("Error while opening device %s: %s", subname, strerror(errno));
            continue;
        }
        if (ioctl(fd, VIDIOC_SUBDEV_G_FMT, &sub_format) < 0) {
            ALOGE("%s VIDIOC_SUBDEV_G_FMT: %s", subname, strerror(errno));
            continue;
        } else {
            ALOGI("Default settings for %s: %ux%u.", subname, sub_format.format.width, sub_format.format.height);
        }
        close(fd);

        fd = open(name, O_RDWR);
        if (-1 == fd) {
            ALOGE("Error while opening device %s: %s.", name, strerror(errno));
            continue;
        }

        v4l2_format format;
        std::memset(&format, 0x00, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_G_FMT, &format) < 0) {
            ALOGE("VIDIOC_G_FMT: %s for device %s.", strerror(errno), name);
        }
        format.fmt.pix.width = sub_format.format.width;
        format.fmt.pix.height = sub_format.format.height;
        if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            ALOGE("VIDIOC_S_FMT: %s for device %s.", strerror(errno), name);
        }

        if (ioctl(fd, VIDIOC_G_FMT, &format) < 0) {
            ALOGE("VIDIOC_G_FMT: %s for device %s.", strerror(errno), name);
        } else {
            sCameraList.emplace_back(name, format.fmt.pix.width, format.fmt.pix.height);
            ALOGI("Camera %s (%ux%u) is successfully probed.",
                    name, format.fmt.pix.width, format.fmt.pix.height);
        }

        close(fd);
    }

    // entity 1: rcar_csi2 feaa0000.csi2 (5 pads, 9 links)
    //            device node name /dev/v4l-subdev0
    //     pad0: Sink
    //            <- "max9286 18-002c":4 [ENABLED,IMMUTABLE]
    if((fd = open("/dev/v4l-subdev0", O_RDWR)) == -1) {
        ALOGE("Error while opening device %s: %s", "/dev/v4l-subdev0", strerror(errno));
        return false;
    }
    if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sub_format) < 0) {
        ALOGE("%s VIDIOC_SUBDEV_S_FMT: %s", "/dev/v4l-subdev0", strerror(errno));
        return false;
    }
    close(fd);

    return true;
}


void EvsEnumerator::enumerateDisplays() {
    if (sDisplayProxyService != nullptr) {
        // Get a display ID list.
        sDisplayProxyService->getDisplayIdList([](const auto& displayIds) {
            for (const auto& id : displayIds) {
                const auto port = id & 0xFF;
                sDisplayPortList.insert_or_assign(port, id);
                sDisplayProxyService->getDisplayInfo(id, [=](const auto& cfg, const auto& state) {
                    android::DisplayConfig* pConfig = (android::DisplayConfig*)cfg.data();
                    android::ui::DisplayState* pState = (android::ui::DisplayState*)state.data();
                    ALOGI("Found display. Id: %lu, Port: %d, "
                          "Resolution: %ux%u (viewport: %ux%u), Orientation: %s, Dpi: %f/%f, "
                          "Refresh: %f, VsyncOff: %lu/%lu, LayerStack: %u, Pdl: %lu, CfgGroup: %d."
                          , id, int (port)
                          , pConfig->resolution.getWidth(), pConfig->resolution.getHeight()
                          , pState->viewport.getWidth(), pState->viewport.getHeight()
                          , android::ui::toCString(pState->orientation)
                          , pConfig->xDpi, pConfig->yDpi
                          , pConfig->refreshRate
                          , pConfig->appVsyncOffset, pConfig->sfVsyncOffset
                          , pState->layerStack
                          , pConfig->presentationDeadline
                          , pConfig->configGroup);
                });
            }
        });
    }
}


// Methods from ::android::hardware::automotive::evs::V1_0::IEvsEnumerator follow.
Return<void> EvsEnumerator::getCameraList(getCameraList_cb _hidl_cb)  {
    ALOGD("getCameraList");

    const unsigned numCameras = sCameraList.size();

    // Build up a packed array of CameraDesc for return
    hidl_vec<CameraDesc_1_0> hidlCameras;
    hidlCameras.resize(numCameras);
    unsigned i = 0;
    for (const auto& cam : sCameraList) {
        hidlCameras[i++] = cam.desc.v1;
    }

    // Send back the results
    ALOGD("reporting %zu cameras available", hidlCameras.size());
    _hidl_cb(hidlCameras);

    // HIDL convention says we return Void if we sent our result back via callback
    return Void();
}


Return<sp<IEvsCamera_1_0>> EvsEnumerator::openCamera(const hidl_string& cameraId) {
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


Return<void> EvsEnumerator::closeCamera(const ::android::sp<IEvsCamera_1_0>& pCamera) {
    ALOGD("closeCamera");

    auto pCamera_1_1 = IEvsCamera::castFrom(pCamera).withDefault(nullptr);
    if (pCamera_1_1 == nullptr) {
        ALOGE("Ignoring call to closeCamera with null camera ptr");
        return Void();
    }

    // Get the camera id so we can find it in our list
    std::string cameraId;
    pCamera_1_1->getCameraInfo_1_1([&cameraId](CameraDesc desc) {
                                     cameraId = desc.v1.cameraId;
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
        } else if (pActiveCamera != pCamera_1_1) {
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


Return<sp<IEvsDisplay_1_0>> EvsEnumerator::openDisplay() {
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


Return<void> EvsEnumerator::closeDisplay(const ::android::sp<IEvsDisplay_1_0>& pDisplay) {
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


// Methods from ::android::hardware::automotive::evs::V1_1::IEvsEnumerator follow.
Return<void> EvsEnumerator::getCameraList_1_1(getCameraList_1_1_cb _hidl_cb)
{
    ALOGD("getCameraList_1_1");
    const unsigned numCameras = sCameraList.size();
    std::vector<CameraDesc_1_1> descriptions;
    descriptions.reserve(numCameras);
    for (const auto& cam : sCameraList) {
        descriptions.push_back(cam.desc);
    }
    hidl_vec<CameraDesc_1_1> hidlDescriptions(descriptions);
    ALOGD("Reporting %zu cameras available.", hidlDescriptions.size());
    _hidl_cb(hidlDescriptions);
    return Void();
}


Return<sp<IEvsCamera_1_1>> EvsEnumerator::openCamera_1_1(const hidl_string& cameraId, const Stream& config)
{
    ALOGD("openCamera_1_1");

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
    pActiveCamera = new EvsCamera(cameraId.c_str(), config);
    pRecord->activeInstance = pActiveCamera;
    if (pActiveCamera == nullptr) {
        ALOGE("Failed to allocate new EvsCamera object for %s\n", cameraId.c_str());
    }

    return pActiveCamera;
}


Return<void> EvsEnumerator::getDisplayIdList(getDisplayIdList_cb _hidl_cb)
{
    hidl_vec<uint8_t> ids;
    ids.resize(sDisplayPortList.size());

    int i = 0;
    sp<EvsDisplay> pActiveDisplay = sActiveDisplay.promote();
    bool isActive = false;
    if (pActiveDisplay) {
        isActive = true;
        ids[i++] = pActiveDisplay->getDisplayId() & 0xFF;
    }

    for (const auto & [port, id] : sDisplayPortList) {
        if (isActive && id == ids[0]) {
            continue;
        }
        ids[i++] = port;
    }

    _hidl_cb(ids);
    return Void();
}


Return<sp<IEvsDisplay_1_1>> EvsEnumerator::openDisplay_1_1(uint8_t port)
{
    ALOGD("openDisplay_1_1");

    // If we already have a display active, then we need to shut it down so we can
    // give exclusive access to the new caller.
    sp<EvsDisplay> pActiveDisplay = sActiveDisplay.promote();
    if (pActiveDisplay != nullptr) {
        ALOGW("Killing previous display because of new caller");
        closeDisplay(pActiveDisplay);
    }

    // Create a new display interface and return it
    pActiveDisplay = new EvsDisplay(sDisplayProxyService, sDisplayPortList[port]);
    sActiveDisplay = pActiveDisplay;

    ALOGD("Returning new EvsDisplay object %p", pActiveDisplay.get());
    return pActiveDisplay;
}


Return<void> EvsEnumerator::getUltrasonicsArrayList(getUltrasonicsArrayList_cb _hidl_cb)
{
    hidl_vec<UltrasonicsArrayDesc> nullValue;
    _hidl_cb(nullValue);
    return Void();
}


Return<sp<IEvsUltrasonicsArray>> EvsEnumerator::openUltrasonicsArray(const hidl_string&)
{
    return nullptr;
}


Return<void> EvsEnumerator::closeUltrasonicsArray(const sp<IEvsUltrasonicsArray>&)
{
    return Void();
}


EvsEnumerator::CameraRecord* EvsEnumerator::findCameraById(const std::string& cameraId) {
    // Find the named camera
    for (auto &&cam : sCameraList) {
        if (cam.desc.v1.cameraId == cameraId) {
            // Found a match!
            return &cam;
        }
    }

    // We didn't find a match
    return nullptr;
}

} // namespace renesas
} // namespace V1_1
} // namespace evs
} // namespace automotive
} // namespace hardware
} // namespace android
