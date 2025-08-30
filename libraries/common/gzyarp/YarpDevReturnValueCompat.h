#ifndef YARP_DEV_RETURN_VALUE_COMPAT_H
#define YARP_DEV_RETURN_VALUE_COMPAT_H

// Defines YARP_VERSION_* macro
#include <yarp/conf/version.h>

// These macros simplify the migration of the devices implementation from YARP 3.11 to YARP 3.12,
// where the interfaces migrated from using bool as return values to use yarp::dev::ReturnValue,
// see https://github.com/robotology/yarp/discussions/3168
//
// For the latest version of this header, check https://gist.github.com/traversaro/ab7ab377c70c50d312a7b5edd4da6242
//

#if (YARP_VERSION_MAJOR > 3) || \
    (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 11) || \
    (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 11 && YARP_VERSION_PATCH >= 100)

// Macro used to easily check if YARP is >= 3.12.0
#define YARP_DEV_RETURN_VALUE_IS_GE_312

// Macros for YARP 3.11 and 3.12 compatibility
// The _CH312 suffix is used as this macro are used in the interface that migrated from bool
// to yarp::dev::ReturnValue in YARP 3.12
#define YARP_DEV_RETURN_VALUE_TYPE_CH312 yarp::dev::ReturnValue
#define YARP_DEV_RETURN_VALUE_OK_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_ok)
#define YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_generic)
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device)
#define YARP_DEV_RETURN_VALUE_ERROR_NWS_NWC_COMMUNICATION_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_nws_nwc_communication_error)
#define YARP_DEV_RETURN_VALUE_ERROR_DEPRECATED_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_deprecated)
#define YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_method_failed)
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_not_ready)
#define YARP_DEV_RETURN_VALUE_ERROR_UNITIALIZED_CH312 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_uninitialized)

#else

#define YARP_DEV_RETURN_VALUE_TYPE_CH312 bool
#define YARP_DEV_RETURN_VALUE_OK_CH312 true
#define YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH312 false
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH312 false
#define YARP_DEV_RETURN_VALUE_ERROR_NWS_NWC_COMMUNICATION_CH312 false
#define YARP_DEV_RETURN_VALUE_ERROR_DEPRECATED_CH312 false
#define YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH312 false
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH312 false
#define YARP_DEV_RETURN_VALUE_ERROR_UNITIALIZED_CH312 false

#endif

#if (YARP_VERSION_MAJOR > 3) || \
    (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || \
    (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)

// Macro used to easily check if YARP is >= 4.0.0
#define YARP_DEV_RETURN_VALUE_IS_GE_40

// Macros for YARP 3.12 and 4.0 compatibility
// The _CH4 suffix is used as this macro are used in the interface that migrated from bool
// to yarp::dev::ReturnValue from YARP 3.12 to YARP 4.0
#define YARP_DEV_RETURN_VALUE_TYPE_CH40 yarp::dev::ReturnValue
#define YARP_DEV_RETURN_VALUE_OK_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_ok)
#define YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_generic)
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device)
#define YARP_DEV_RETURN_VALUE_ERROR_NWS_NWC_COMMUNICATION_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_nws_nwc_communication_error)
#define YARP_DEV_RETURN_VALUE_ERROR_DEPRECATED_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_deprecated)
#define YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_method_failed)
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_error_not_ready)
#define YARP_DEV_RETURN_VALUE_ERROR_UNITIALIZED_CH40 yarp::dev::ReturnValue(yarp::dev::ReturnValue::return_code::return_value_uninitialized)

#else

#define YARP_DEV_RETURN_VALUE_TYPE_CH40 bool
#define YARP_DEV_RETURN_VALUE_OK_CH40 true
#define YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40 false
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40 false
#define YARP_DEV_RETURN_VALUE_ERROR_NWS_NWC_COMMUNICATION_CH40 false
#define YARP_DEV_RETURN_VALUE_ERROR_DEPRECATED_CH40 false
#define YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40 false
#define YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40 false
#define YARP_DEV_RETURN_VALUE_ERROR_UNITIALIZED_CH40 false

#endif


#endif