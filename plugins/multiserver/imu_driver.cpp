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

#include <osvr/Util/ReturnCodesC.h>
#include <osvr/Util/TimeValueC.h>
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>

#include "com_osvr_Multiserver_OSVRHackerDevKit_json.h"

#include "hidapi/hidapi.h"
#include "quat.h"

#include <json/value.h>
#include <json/reader.h>


#include <stdio.h>  // fprintf
#include <assert.h>


// int16_t to unpack Qpoint numbers
#ifdef __cplusplus
# ifndef __STDC_LIMIT_MACROS
#  define __STDC_LIMIT_MACROS
# endif
#endif
#include <stdint.h>


#ifndef M_SQRT2_2
#define M_SQRT2_2 0.70710678118654752440084436210484
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#include "usb.h"
#include "imu_driver.h"


namespace {

struct unpacked_report {
    signed   char version;
    unsigned char video_status;
    unsigned char message_number;
    // game rotation (quaternion)
    struct {
        float x, y, z, w;
    } rotq;
    // rotational velocity in the same coordinates
    struct { // quaternion
        float x, y, z, w;
    } gyrq;
    float gyr_dt;
};

enum {
    VIDEO_STATUS_UNKNOWN,
    VIDEO_STATUS_NONE,
    VIDEO_STATUS_PORTRAIT,
    VIDEO_STATUS_LANDSCAPE
};


/// @brief unpack a Qpoint fixed precision number into a float
/// 
/// @param data the Qpoint as a little endian two's complement 16 bits integer
/// @param Q the number of fractional bits, in [0,15]
/// @param scale a factor by which to multiply the packed number
static inline
float unpack_16Qpoint_to_float( unsigned char const *data, int Q, float scale ) {
    assert( data != NULL );
    assert( 0 <= Q && Q <= 15 );
#if defined INT16_MAX
    // If two's complement is guaranteed then buffer is allocated accordingly
    // so we can directly cast if endianness is the same as input
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__==__ORDER_LITTLE_ENDIAN__)
    int16_t const i = *(int16_t const*)data;
#else
    int16_t i = 1; // Not sure about the endianness at compile time
    unsigned char *p = (unsigned char *)&i;
    if (p[0]) {
        i = *(int16_t const*)data;
    } else {
        p[0] = data[1];
        p[1] = data[0];
    }
#endif
#else
    // Fallback doesn't need C99 and makes no assumption about negative integer
    // representation, nor about the size of short/int/long, nor the endianness
    unsigned const u = (unsigned)data[1] << 8 | data[0];
    long int const i = (u ^ 0x8000U) - (long)0x8000U;
#endif
    return (float)i * (scale / (float)(1U << Q));
}

static
struct unpacked_report rz_hmd_tracker_parse_v1( unsigned char const *data ) {
    struct unpacked_report r;
    r.version = data[0] & 0xFU;
    r.message_number = data[1];
    r.video_status = VIDEO_STATUS_UNKNOWN;
    r.rotq.x = unpack_16Qpoint_to_float( data + 2, 14, 1.f );
    r.rotq.y = unpack_16Qpoint_to_float( data + 4, 14, 1.f );
    r.rotq.z = unpack_16Qpoint_to_float( data + 6, 14, 1.f );
    r.rotq.w = unpack_16Qpoint_to_float( data + 8, 14, 1.f );
    return r;
}
static
struct unpacked_report rz_hmd_tracker_parse_v2( unsigned char const *data ) {
    struct unpacked_report r = rz_hmd_tracker_parse_v1( data );
    float const dt = 1.f / 50.f;
    // rotationnal velocity as euler angles (roll, pitch, yaw) relative to HMD
    q_vec_type const eulerh = { unpack_16Qpoint_to_float( data + 10, 9, dt ),
                                unpack_16Qpoint_to_float( data + 12, 9, dt ),
                                unpack_16Qpoint_to_float( data + 14, 9, dt ) };
    q_vec_type   eulerw;
    q_type       quatw;
    // convert angular velocity to world space coordinate system
    q_type const hmdrot = { r.rotq.x, r.rotq.y, r.rotq.z, r.rotq.w };
    q_xform( eulerw, hmdrot, eulerh );
    // convert angular velocity to a quaternion
    q_from_euler( quatw, eulerw[2], eulerw[1], eulerw[0] ); // yaw, pitch, roll
    r.gyrq.x = (float)quatw[0];
    r.gyrq.y = (float)quatw[1];
    r.gyrq.z = (float)quatw[2];
    r.gyrq.w = (float)quatw[3];
    r.gyr_dt = (float)dt;
    return r;
}
static
struct unpacked_report rz_hmd_tracker_parse_v3( unsigned char const *data ) {
    struct unpacked_report r = rz_hmd_tracker_parse_v2( data );
    bool const video_detected = (data[0] & 0x1U << 4) != 0;
    bool const video_portrait = (data[0] & 0x1U << 5) != 0;
    if (!video_detected) {
        r.video_status = VIDEO_STATUS_NONE;
    } else {
        if (video_portrait) {
            r.video_status = VIDEO_STATUS_PORTRAIT;
        } else {
            r.video_status = VIDEO_STATUS_LANDSCAPE;
        }
    }
    return r;
}



/// @brief parse a report from the OSVR HMD's tracker
/// 
/// The /p version field is set to 0 when the report can't be parsed properly.
struct unpacked_report rz_hmd_tracker_parse( unsigned char const *data, int size ) {
    assert( data != NULL );
    int const version = data[0] & 0xFU;
    switch (version) {
    case 1:
        if (size == 16 || size == 32) {
            return rz_hmd_tracker_parse_v1( data );
        }
        break;
    case 2:
        if (size == 16) {
            return rz_hmd_tracker_parse_v2( data );
        }
        break;
    case 3:
        if (size == 16) {
            return rz_hmd_tracker_parse_v3( data );
        }
        break;
    default:
        break;
    }
    struct unpacked_report no_report;
    no_report.version = 0;
    return no_report;
}


/// @brief read a report from the HMD tracker
/// 
/// @param hmdt the HMD tracker USB device
/// @param ms_timeout timeout (in millisecond) on the read operation.
///     A negative value means a blocking read.
/// 
/// @return version -1 if an error occured, version 0 if no report could be read
struct unpacked_report rz_hmd_tracker_read( hid_device *hmdt, int ms_timeout ) {
    assert( hmdt != NULL );
    // Reports are 32 bytes max. Should be only 1 type of report so we shouldn't
    // need the additionnal byte, but add it just in case.
#if defined INT16_MAX       // /see unpack_16Qpoint_to_float
    // We try to read the Qpoints by casting the pointer to int16_t
    int16_t        buf[17]; // avoid alignment and strict-aliasing issues,
    unsigned char *tmp = (unsigned char*)buf;
#else
    unsigned char  tmp[33];
#endif

    if (ms_timeout < 0) {
        ms_timeout = -1; // do a blocking call to hid_read_timeout
    }
    int const read = hid_read_timeout( hmdt, tmp, 33, ms_timeout );

    if (read < 0) {
        struct unpacked_report error_report;
        error_report.version = -1;
        return error_report;
    } else if (read == 0) {
        struct unpacked_report no_report;
        no_report.version = 0;
        return no_report;
    } else {
        return rz_hmd_tracker_parse( tmp, read );
    }
}



struct usb {
    unsigned short const id[2][2] = {
        { 0x1532U, 0x0b00U },   // OSVR HMD
        { 0x03EBU, 0x2421U }    // old OSVR
    };  // vendor,  device
    int const n = sizeof(id)/sizeof(*id);
};


class rz_OSVRHackerDevKit {
public:

    struct HardwareDetection {
        HardwareDetection( std::string && name, int read_timeout, int reconnect_timeout )
            : _driver( NULL )
            , _name( name )
            , _read_timeout( read_timeout )
            , _reconnect_timeout( reconnect_timeout ) {
        }

        OSVR_ReturnCode HardwareDetection::operator() ( OSVR_PluginRegContext ctx );

        rz_OSVRHackerDevKit        *_driver;
        // driver prameters
        std::string _name;
        int         _read_timeout;
        int         _reconnect_timeout;
    };

    rz_OSVRHackerDevKit( OSVR_PluginRegContext ctx, HardwareDetection const &params, hid_device *hmdt )
        : _hmdt( hmdt )
        , _params( params )
        , _state( CONNECTED ) {
        assert( _hmdt != NULL );
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions( ctx );
        osvrDeviceTrackerConfigure( opts, &_tracker );
        osvrDeviceAnalogConfigure( opts, &_analog, 2 );

        _dev.initSync( ctx, params._name, opts );
        _dev.sendJsonDescriptor( com_osvr_Multiserver_OSVRHackerDevKit_json );
        _dev.registerUpdateCallback( this );
    }

    ~rz_OSVRHackerDevKit() {
        if ( _hmdt != NULL ) {
            hid_close( _hmdt );
        }
    }

    void reconnect() {
        if (_state == DISCONNECTED) {
            assert( _hmdt == NULL );
            _hmdt = rz_get_first_device( usb().id, usb().n, NULL );
            if ( _hmdt != NULL ) {
                _state = CONNECTED;
            }
        }
    }

    OSVR_ReturnCode update() {
        // stay connected to the hmd
        if (_state == DISCONNECTED) {
            // wait some time before each hardware detection
            int const timeout = _params._reconnect_timeout;
            if ( timeout > 0 ) {
                OSVR_TimeValue now;
                osvrTimeValueGetNow( &now );
                OSVR_TimeValue elapsed = now;
                osvrTimeValueDifference( &elapsed, &_disconnect_time );
                if (elapsed.seconds*1000 + elapsed.microseconds/1000 < timeout) {
                    return OSVR_RETURN_FAILURE;
                }
                _disconnect_time = now;
            }
            // hardware detection
            assert( _hmdt == NULL );
            _hmdt = rz_get_first_device( usb().id, usb().n, NULL );
            if ( _hmdt == NULL ) {
                return OSVR_RETURN_FAILURE;
            } else {
                fprintf( stderr, "%s: reconnected\n", _params._name.c_str() );
                _state = CONNECTED;
            }
        }
        
        // read data from USB
        assert( _hmdt != NULL );
        assert( _state != DISCONNECTED );
        int const timeout = _params._read_timeout;
        unpacked_report const report = rz_hmd_tracker_read( _hmdt, timeout );
        if ( report.version == -1 ) { // something bad happened...
            // or maybe it's just that a signal was raised
            if ( _state == CONNECTED ) {
                _state = SUSPICIOUS;
            } else { // two bad reads in a row
                assert( _state == SUSPICIOUS );
                fprintf( stderr, "%s: %ls\n", _params._name.c_str(), hid_error(_hmdt) );
                hid_close( _hmdt );
                _hmdt = NULL;
                _state = DISCONNECTED;
                osvrTimeValueGetNow( &_disconnect_time );
            }
            return OSVR_RETURN_FAILURE;
        }
        
        // send data to OSVR
        assert( _state == CONNECTED );
        send_report_to_OSVR( report );

        return OSVR_RETURN_SUCCESS;
    }

  private:
    
    enum state_t {
        DISCONNECTED,   // disconnected from USB
        CONNECTED,      // connected and everything is fine
        SUSPICIOUS      // connected but got an error during last read
    };
    
    osvr::pluginkit::DeviceToken    _dev;
    OSVR_TrackerDeviceInterface     _tracker;
    OSVR_AnalogDeviceInterface      _analog;
    
    hid_device                     *_hmdt;    // usb device
    HardwareDetection        const &_params;  // holds json parameters
    enum state_t                    _state;   // connected/disconnected from usb
    OSVR_TimeValue                  _disconnect_time;
    
    void send_report_to_OSVR( struct unpacked_report const report ) const {
        OSVR_TimeValue timestamp;
        OSVR_TimeValue const * const t = &timestamp;
        // orientation
        if ( report.version >= 1 ) {
            osvrTimeValueGetNow( &timestamp );
            OSVR_OrientationState o;
            osvrQuatSetX( &o, report.rotq.x );
            osvrQuatSetY( &o, report.rotq.y );
            osvrQuatSetZ( &o, report.rotq.z );
            osvrQuatSetW( &o, report.rotq.w );
            osvrDeviceTrackerSendOrientationTimestamped( _dev, _tracker, &o, 0, t );
        }
        // angular velocity
        if ( report.version >= 2 ) {
            OSVR_AngularVelocityState v;
            osvrQuatSetX( &v.incrementalRotation, report.gyrq.x );
            osvrQuatSetY( &v.incrementalRotation, report.gyrq.y );
            osvrQuatSetZ( &v.incrementalRotation, report.gyrq.z );
            osvrQuatSetW( &v.incrementalRotation, report.gyrq.w );
            v.dt = report.gyr_dt;
            osvrDeviceTrackerSendAngularVelocityTimestamped( _dev, _tracker, &v, 0, t );
        }
        // analog (sic) output
        if ( report.version >= 1 ) {
            OSVR_AnalogState data[2] = {
                (double) report.version,
                (double) report.video_status
            };
            osvrDeviceAnalogSetValuesTimestamped( _dev, _analog, data, 2, t );
        }
    }
};



OSVR_ReturnCode
rz_OSVRHackerDevKit::HardwareDetection::operator() ( OSVR_PluginRegContext ctx ) {
    // We have already spawned a plugin and we don't want to spawn several
    if ( _driver != NULL ) {
        // The driver doesn't want to detect at each loop of the server.
        // If the HMD got disconnected, we want to reconenct to it before
        // multiserver can do it.
        // So we ask the existing driver to reconnect here since we are
        // (hopefully) called by the server before the multiserver detect.
        _driver->reconnect();
        return OSVR_RETURN_FAILURE;
    }
    
    // we want to know if we fail at finding a device or at opening one
    struct hid_device_info *info = rz_get_devices_info( usb().id, usb().n );
    hid_device             *hmdt = rz_open_device_info( &info );

    if ( hmdt == NULL ) {
        if ( info == NULL ) {
            fprintf( stderr, "IMU: Could not find a supported device.\n" );
        } else {
            fprintf( stderr, "IMU: Could not open a device. It was opened somewhere else or you don't have the rights.\n" );
        }
        hid_free_enumeration( info );
        return OSVR_RETURN_FAILURE;
    } else {
        //fprintf( stderr, "Opening device:\n" );
        //rz_print_device_info( info );
        hid_free_enumeration( info );
        
        _driver = new rz_OSVRHackerDevKit( ctx, *this, hmdt );
        osvr::pluginkit::registerObjectForDeletion( ctx, _driver );
        
        return OSVR_RETURN_SUCCESS;
    }
}

} // anonymous namespace





OSVR_ReturnCode rz_OSVRHackerDevKit_driver_callback(OSVR_PluginRegContext ctx,
    const char *params, void *path ) {
    Json::Reader reader;
    Json::Value  root;

    // get parameters from the json with those default values
    std::string name = "OSVRHackerDevKit";
    int         read_timeout = -1;          // blocking reads by default
    int         reconnect_timeout = 2000;   // 2 seconds
    bool        detect_now = true;
    bool        register_hwdetect = true;
    if ( params == NULL || !reader.parse(params, root) ) {
        // just use default values
    } else {
        name              = root.get("name"             , name             ).asString();
        read_timeout      = root.get("read_timeout"     , read_timeout     ).asInt();
        reconnect_timeout = root.get("reconnect_timeout", reconnect_timeout).asInt();
        detect_now        = root.get("detect_now"       , detect_now       ).asBool();
        register_hwdetect = root.get("register_hwdetect", register_hwdetect).asBool();
    }

    if ( !detect_now && !register_hwdetect ) {
        // this has no effect
        return OSVR_RETURN_FAILURE;
    }

    rz_OSVRHackerDevKit::HardwareDetection *hwd =
        new rz_OSVRHackerDevKit::HardwareDetection(
            std::move(name), read_timeout, reconnect_timeout
    );
    // We'd want to override multiserver by registering a hardware detection
    // callback first. This would ensure we get the device even if it's not
    // plugged in at launch, or if it's unplugged/replugged.
    if ( register_hwdetect ) {
        osvr::pluginkit::PluginContext(ctx).registerHardwareDetectCallback(hwd);
    }
    // It's enough on windows if the plugin name precedes com_osvr_Multiserver
    // in the alphabetical order, but it is not on other platforms.
    // The order in which plugins are loaded is actually unspecified (sic), so
    // we detect the device right away. This ensures we always get the device
    // if it's plugged in during the startup.
    bool spawned = false;
    if ( detect_now ) {
        spawned = (*hwd)(ctx) == OSVR_RETURN_SUCCESS;
    }
    // if we dont register the hardware detection, we still have to delete it
    if (!register_hwdetect) {
        if (spawned) {
            osvr::pluginkit::PluginContext( ctx ).registerObjectForDeletion( hwd );
        } else {
            delete hwd;
        }
    }

    return OSVR_RETURN_SUCCESS;
}

