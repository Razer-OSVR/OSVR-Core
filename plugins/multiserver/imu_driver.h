// Copyright 2016 Razer Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDED_com_razer_OSVRHackerDevKit_IMU_driver_h
#define INCLUDED_com_razer_OSVRHackerDevKit_IMU_driver_h

#include <osvr/Util/ReturnCodesC.h>
#include <osvr/PluginKit/PluginKitC.h>


/// @brief duh
/// 
/// /param params the parameters fro m the server configuration json file
///
/// json param /p name is the name of the IMU driver
/// json param /p read_timeout is the timeout used when reading data on USB
///   from the IMU. A negative value means a blocking read.
/// json param /p reconnect_timeout is the timeout between each hardware detect
///   made directly by the driver if the HMD is disconnected after the startup.
///   On non-windows platforms, a /p 0 value is necessary to garantee we
///   reconnect to the device before the multiserver does after the HMD is
///   disconnected/reconnected
/// json param /p detect_on_instantiation bool
/// json param /p register_hwdetect bool
/// 
OSVR_ReturnCode rz_OSVRHackerDevKit_driver_callback( OSVR_PluginRegContext ctx,
        const char *params, void * = NULL );



#endif // guard
