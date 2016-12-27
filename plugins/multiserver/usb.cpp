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


#include <stdio.h>  // fprintf
#include <assert.h>


#include "usb.h"


struct hid_device_info * rz_get_devices_info( unsigned short const id[][2], int nid ) {
    struct hid_device_info *info = NULL;
    int i;
    // every devices
    if (id == NULL) {
        return hid_enumerate( 0, 0 );
    }
    // chain hid_device_info from subsequent vendor_id/product_id pairs
    assert( nid >= 0 ); // nid > INT_MIN
    for (i = nid; i --> 0; ) {
        struct hid_device_info *first = hid_enumerate( id[i][0], id[i][1] );
        if (first != NULL) {
            struct hid_device_info *last = first;
            while (last->next != NULL) {
                last = last->next;
            }
            // queue the previously opened devices to the newly opened ones
            last->next = info;
            info = first;
        }
    }
    return info;
}


hid_device * rz_open_device_info( struct hid_device_info **info_list ) {
    struct hid_device_info **info = info_list;
    hid_device *device = NULL;
    assert( info_list != NULL );
    for (; *info != NULL; info = &(*info)->next) {
        device = hid_open_path( (*info)->path );
        if (device != NULL) {
            // extract opened info from the list
            struct hid_device_info *opened = *info;
            *info = (*info)->next;
            // put it in front of the list
            opened->next = *info_list;
            *info_list = opened;
            break;
        }
    }
    return device;
}


hid_device * rz_get_first_device( unsigned short const id[][2], int nid, struct hid_device_info **opened ) {
    if (id != NULL) {
        int i;
        for (i = 0; i < nid; i++) {
            struct hid_device_info *info = hid_enumerate( id[i][0], id[i][1] );
            hid_device *device = rz_open_device_info( &info );
            if (device != NULL) { // we opened a device
                // head of the linked list is hid_device_info of opened device
                if (opened != NULL) { // caller wants it, pop it
                    *opened = info;
                    info = (*opened)->next;
                    (*opened)->next = NULL;
                }
                hid_free_enumeration( info );
                return device;
            }
            hid_free_enumeration( info );
        }
    }
    return NULL;
}



void rz_print_device_info( struct hid_device_info const * i ) {
    if ( i != NULL ) {
        fprintf( stderr, "path         : \"%s\"\n", i->path );
        fprintf( stderr, "vendor id    : 0x%04x\n", i->vendor_id );
        fprintf( stderr, "device id    : 0x%04x\n", i->product_id );
        fprintf( stderr, "serial_number: \"%ls\"\n", i->serial_number );
        fprintf( stderr, "manufacturer : \"%ls\"\n", i->manufacturer_string );
        fprintf( stderr, "product      : \"%ls\"\n", i->product_string );
        fprintf( stderr, "interface nb : %d\n", i->interface_number );
        fprintf( stderr, "release nb   : 0x%04hx\n", i->release_number );
        fprintf( stderr, "\n" );
        // Do not use those. If available at all, they may require detaching
        // and re-attaching the kernel driver. Prefer using interface number.
        // fprintf( stderr, "usage        : 0x%04hx\n", i->usage );
        // fprintf( stderr, "usage page   : 0x%04hx\n", i->usage_page );
    }
}
#if 0
static
struct hid_device_info * debug_print_all_devices_info( struct hid_device_info * info ) {
    struct hid_device_info * head = info;
    if ( info != NULL ) {
        fprintf( stderr, "Devices found:\n\n" );
        for (  ;  info != NULL  ;  info = info->next ) {
            rz_print_device_info( info );
        }
    }
    return head;
}
#endif


