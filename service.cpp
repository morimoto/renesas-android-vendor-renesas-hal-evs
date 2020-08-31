/*
 * Copyright (C) 2020 GlobalLogic
 * Copyright (C) 2016 The Android Open Source Project
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

#include <hidl/HidlTransportSupport.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>
#include <utils/Log.h>

#include "ServiceNames.h"
#include "EvsEnumerator.h"

#include <vendor/renesas/graphics/composer/2.0/IComposer.h>

// libhidl:
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

// Generated HIDL files
using android::hardware::automotive::evs::V1_1::IEvsEnumerator;

// The namespace in which all our implementation code lives
using namespace android::hardware::automotive::evs::V1_1::renesas;
using namespace android;

using vendor::renesas::graphics::composer::V2_0::IComposer;
using android::frameworks::automotive::display::V1_0::IAutomotiveDisplayProxyService;

int main(int argc, char ** argv) {
    Platform platform = Platform::Unknown;
    if (argc >= 2) {
        ALOGI("The specified platform: %s.", argv[1]);
        if (!strcmp(argv[1], "salvator")) {
            platform = Platform::Salvator;
        } else if (!strcmp(argv[1], "kingfisher")) {
            platform = Platform::Kingfisher;
        }
    }

    ALOGI("Waiting for IComposer service...");
    sp<IComposer> composer = IComposer::getService();
    ALOGI("Waiting for IAutomotiveDisplayProxyService service...");
    sp<IAutomotiveDisplayProxyService> windowService = IAutomotiveDisplayProxyService::getService();
    android::sp<IEvsEnumerator> service = new EvsEnumerator(platform, windowService);

    configureRpcThreadpool(1, true /* callerWillJoin */);

    // Register our service -- if somebody is already registered by our name,
    // they will be killed (their thread pool will throw an exception).
    status_t status = service->registerAsService(kEnumeratorServiceName);
    LOG_ALWAYS_FATAL_IF(status != android::OK,
                        "Could not register service %s (%d).", kEnumeratorServiceName, status);
    joinRpcThreadpool();
    return 1;
}
