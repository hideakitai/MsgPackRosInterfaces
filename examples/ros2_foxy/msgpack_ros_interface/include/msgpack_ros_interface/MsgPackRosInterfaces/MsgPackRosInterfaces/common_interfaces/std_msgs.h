#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_STD_MSGS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_STD_MSGS_H

#include "../rcl_interfaces/builtin_interfaces.h"

#ifdef ROS
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace std_msgs {

namespace msg {

    struct Empty {};

    // # Standard metadata for higher-level stamped data types.
    // # This is generally used to communicate timestamped data
    // # in a particular coordinate frame.
    //
    // # Two-integer timestamp that is expressed as seconds and nanoseconds.
    // builtin_interfaces/Time stamp
    //
    // # Transform frame with which this data is associated.
    // string frame_id
    struct Header {
        rcl_interfaces::builtin_interfaces::msg::Time stamp;
        MsgPack::str_t frame_id;
        MSGPACK_DEFINE(stamp, frame_id);

#ifdef ROS
        Header& operator=(const ::std_msgs::msg::Header& rhs) {
            stamp = rhs.stamp;
            frame_id = rhs.frame_id;
            return *this;
        }
        Header& operator=(::geometry_msgs::msg::Header&& rhs) {
            stamp = std::move(rhs.stamp);
            frame_id = std::move(rhs.frame_id);
            return *this;
        }
#endif
    };

    // float32 r
    // float32 g
    // float32 b
    // float32 a
    struct ColorRGBA {
        float r;
        float g;
        float b;
        float a;
        MSGPACK_DEFINE(r, g, b, a);

#ifdef ROS
        ColorRGBA& operator=(const ::std_msgs::msg::ColorRGBA& rhs) {
            r = rhs.r;
            g = rhs.g;
            b = rhs.b;
            a = rhs.a;
            return *this;
        }
        ColorRGBA& operator=(::geometry_msgs::msg::ColorRGBA&& rhs) {
            r = std::move(rhs.r);
            g = std::move(rhs.g);
            b = std::move(rhs.b);
            a = std::move(rhs.a);
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace std_msgs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_STD_MSGS_H
