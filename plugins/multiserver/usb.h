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

#ifndef INCLUDED_com_razer_OSVRHackerDevKit_usb_h
#define INCLUDED_com_razer_OSVRHackerDevKit_usb_h


#include "hidapi/hidapi.h"


/// @brief returns a chained list of all devices corresponding to at least one
///     of the supplied (vendor_id,product_id).
/// 
/// @param id array of (vendor_id,product_id) pairs
/// @param nid size of /p id array
/// 
/// if /p id is /p NULL, then return information on all available devices
struct hid_device_info * rz_get_devices_info( unsigned short const id[][2], int nid );


/// @brief Returns the first device that could be opened (in given order)
/// 
/// @param info_list a pointer to a list of device to try to open. not /p NULL
///      but can point to a /p NULL /p hid_device_info*
/// 
/// Opened hid_device_info is moved to the front of /p info_list.
hid_device * rz_open_device_info( struct hid_device_info **info_list );


/// @brief Returns the first device that could be opened (in given order)
/// 
/// @param id array of vendor_id/product_id pairs
/// @param nid size of /p id array
/// @param opened Used to output hid_device_info of opened device. Can be NULL.
hid_device * rz_get_first_device( unsigned short const id[][2], int nid, struct hid_device_info **opened );


/// @brief print hid_device_info
void rz_print_device_info( struct hid_device_info const * i );



#endif // guard
