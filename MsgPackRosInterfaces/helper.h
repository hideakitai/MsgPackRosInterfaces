#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_HELPER_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_HELPER_H

#define ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_NAMESPACE_BEGIN \
    namespace arduino {                                               \
    namespace msgpack {                                               \
        namespace rcl_interfaces {
#define ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_NAMESPACE_END \
    }                                                               \
    }                                                               \
    }

#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN \
    namespace arduino {                                                  \
    namespace msgpack {                                                  \
        namespace common_interfaces {
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END \
    }                                                                  \
    }                                                                  \
    }

ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_NAMESPACE_BEGIN
ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_NAMESPACE_END

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN
ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

namespace MsgPackRosIF = arduino::msgpack::common_interfaces;

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_HELPER_H
