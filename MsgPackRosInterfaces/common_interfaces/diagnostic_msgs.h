#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_DIAGNOSTIC_MSGS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_DIAGNOSTIC_MSGS_H

#include "std_msgs.h"

#ifdef ROS
#include <std_msgs/msg/header.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace diagnostic_msgs {

namespace msg {
    // # What to label this value when viewing.
    // string key
    // # A value to track over time.
    // string value
    struct KeyValue {
        MsgPack::str_t key;
        MsgPack::str_t value;
        MSGPACK_DEFINE(key, value);

#ifdef ROS
        KeyValue& operator=(const ::diagnostic_msgs::msg::KeyValue& rhs) {
            key = rhs.key;
            value = rhs.value;
            return *this;
        }
        KeyValue& operator=(::diagnostic_msgs::msg::KeyValue&& rhs) {
            key = std::move(rhs.key);
            value = std::move(rhs.value);
            return *this;
        }
#endif
    };

    // # This message holds the status of an individual component of the robot.
    //
    // # Possible levels of operations.
    // byte OK=0
    // byte WARN=1
    // byte ERROR=2
    // byte STALE=3
    //
    // # Level of operation enumerated above.
    // byte level
    // # A description of the test/component reporting.
    // string name
    // # A description of the status.
    // string message
    // # A hardware unique string.
    // string hardware_id
    // # An array of values associated with the status.
    // KeyValue[] values
    struct DiagnosticStatus {
        static constexpr uint8_t OK = 0;
        static constexpr uint8_t WARN = 1;
        static constexpr uint8_t ERROR = 2;
        static constexpr uint8_t STALE = 3;

        uint8_t level;
        MsgPack::str_t name;
        MsgPack::str_t message;
        MsgPack::str_t hardware_id;
        KeyValue values;
        MSGPACK_DEFINE(level, name, message, hardware_id);

#ifdef ROS
        DiagnosticStatus& operator=(const ::diagnostic_msgs::msg::DiagnosticStatus& rhs) {
            level = rhs.level;
            name = rhs.name;
            message = rhs.message;
            hardware_id = rhs.hardware_id;
            values = rhs.values;
            return *this;
        }
        DiagnosticStatus& operator=(::diagnostic_msgs::msg::DiagnosticStatus&& rhs) {
            level = std::move(rhs.leval);
            name = std::move(rhs.name);
            message = std::move(rhs.message);
            hardware_id = std::move(rhs.hardware_id);
            values = std::move(rhs.values);
            return *this;
        }
#endif
    };

    // # This message is used to send diagnostic information about the state of the robot.
    // std_msgs/Header header # for timestamp
    // DiagnosticStatus[] status # an array of components being reported on
    struct DiagnosticArray {
        std_msgs::msg::Header header;
        MsgPack::arr_t<DiagnosticStatus> status;
        MSGPACK_DEFINE(header, status);

#ifdef ROS
        DiagnosticArray& operator=(const ::diagnostic_msgs::msg::DiagnosticArray& rhs) {
            header = rhs.header;
            status = rhs.status;
            return *this;
        }
        DiagnosticArray& operator=(::diagnostic_msgs::msg::DiagnosticArray&& rhs) {
            header = std::move(rhs.header);
            status = std::move(rhs.status);
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace diagnostic_msgs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_DIAGNOSTIC_MSGS_H
