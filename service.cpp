/*
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

#include <unistd.h>

#include <hidl/HidlTransportSupport.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>
#include <utils/Log.h>

#include "ServiceNames.h"
#include "EvsEnumerator.h"
#include "EvsDisplay.h"

#include <vendor/renesas/graphics/composer/2.0/IComposer.h>

// libhidl:
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

// Generated HIDL files
using android::hardware::automotive::evs::V1_0::IEvsEnumerator;
using android::hardware::automotive::evs::V1_0::IEvsDisplay;

// The namespace in which all our implementation code lives
using namespace android::hardware::automotive::evs::V1_0::renesas;
using namespace android;

using vendor::renesas::graphics::composer::V2_0::IComposer;

int main() {

    sp<IComposer> composer = IComposer::getService();
    android::sp<IEvsEnumerator> service = new EvsEnumerator();

    configureRpcThreadpool(1, true /* callerWillJoin */);

    // Register our service -- if somebody is already registered by our name,
    // they will be killed (their thread pool will throw an exception).
    status_t status = service->registerAsService(kEnumeratorServiceName);
    LOG_ALWAYS_FATAL_IF(status != android::OK,
                        "Could not register service %s (%d).", kEnumeratorServiceName, status);
    joinRpcThreadpool();
    return 1;
}
