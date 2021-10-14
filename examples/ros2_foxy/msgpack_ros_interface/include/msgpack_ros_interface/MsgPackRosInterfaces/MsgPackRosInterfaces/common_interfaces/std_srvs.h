#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_STD_SRVS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_STD_SRVS_H

#ifdef ROS
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace std_srvs {

namespace msg {}

namespace srv {

    struct Empty {
        struct Request {
            MSGPACK_DEFINE();
        };

        struct Response {
            MSGPACK_DEFINE();
        };
    };

    // bool data # e.g. for hardware enabling / disabling
    // ---
    // bool success   # indicate successful run of triggered service
    // string message # informational, e.g. for error messages
    struct SetBool {
        struct Request {
            bool data;
            MSGPACK_DEFINE(data);

#ifdef ROS
            Request& operator=(const ::std_srvs::srv::SetBool::Request& rhs) {
                data = rhs.data;
                return *this;
            }
            Request& operator=(::std_srvs::srv::SetBool::Request&& rhs) {
                data = std::move(rhs.data);
                return *this;
            }
#endif
        };

        struct Response {
            bool success;
            MsgPack::str_t message;
            MSGPACK_DEFINE(success, message);

#ifdef ROS
            Response& operator=(const ::std_srvs::srv::SetBool::Response& rhs) {
                success = rhs.success;
                message = rhs.message;
                return *this;
            }
            Response& operator=(::std_srvs::srv::SetBool::Response&& rhs) {
                success = std::move(rhs.success);
                message = std::move(rhs.message);
                return *this;
            }
#endif
        };
    };

    // ---
    // bool success   # indicate successful run of triggered service
    // string message # informational, e.g. for error messages
    struct Trigger {
        struct Request {
            MSGPACK_DEFINE();
        };

        struct Response {
            bool success;
            MsgPack::str_t message;
            MSGPACK_DEFINE(success, message);

#ifdef ROS
            Response& operator=(const ::std_srvs::srv::SetBool::Response& rhs) {
                success = rhs.success;
                message = rhs.message;
                return *this;
            }
            Response& operator=(::std_srvs::srv::SetBool::Response&& rhs) {
                success = std::move(rhs.success);
                message = std::move(rhs.message);
                return *this;
            }
#endif
        };
    };

}  // namespace srv

}  // namespace std_srvs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_STD_SRVS_H
