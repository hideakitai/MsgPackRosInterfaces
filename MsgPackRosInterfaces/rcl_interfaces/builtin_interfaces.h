#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_BUILTIN_INTERFACES_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_BUILTIN_INTERFACES_H

#ifdef ROS
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_NAMESPACE_BEGIN

namespace builtin_interfaces {

namespace msg {

    // # Duration defines a period between two time points.
    // # Messages of this datatype are of ROS Time following this design:
    // # https://design.ros2.org/articles/clock_and_time.html
    //
    // # Seconds component, range is valid over any possible int32 value.
    // int32 sec
    //
    // # Nanoseconds component in the range of [0, 10e9).
    // uint32 nanosec
    struct Duration {
        int32_t sec {0};
        uint32_t nanosec {0};
        MSGPACK_DEFINE(sec, nanosec);

#ifdef ROS
        Duration& operator=(const ::builtin_interfaces::msg::Duration& rhs) {
            sec = rhs.sec;
            nanosec = rhs.nanosec;
            return *this;
        }
        Duration& operator=(::builtin_interfaces::msg::Duration&& rhs) {
            sec = std::move(rhs.sec);
            nanosec = std::move(rhs.nanosec);
            return *this;
        }
#endif
    };

    // # This message communicates ROS Time defined here:
    // # https://design.ros2.org/articles/clock_and_time.html
    //
    // # The seconds component, valid over all int32 values.
    // int32 sec
    //
    // # The nanoseconds component, valid in the range [0, 10e9).
    // uint32 nanosec
    struct Time {
        int32_t sec {0};
        uint32_t nanosec {0};
        MSGPACK_DEFINE(sec, nanosec);

#ifdef ROS
        Time& operator=(const ::builtin_interfaces::msg::Time& rhs) {
            sec = rhs.sec;
            nanosec = rhs.nanosec;
            return *this;
        }
        Time& operator=(::builtin_interfaces::msg::Time&& rhs) {
            sec = std::move(rhs.sec);
            nanosec = std::move(rhs.nanosec);
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace builtin_interfaces

ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_RCL_INTERFACES_BUILTIN_INTERFACES_H
