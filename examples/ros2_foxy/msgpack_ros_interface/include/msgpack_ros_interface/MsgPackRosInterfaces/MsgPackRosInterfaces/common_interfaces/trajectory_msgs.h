#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_TRAJECTORY_MSGS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_TRAJECTORY_MSGS_H

#ifdef ROS
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace trajectory_msgs {

namespace msg {

    // # Each trajectory point specifies either positions[, velocities[, accelerations]]
    // # or positions[, effort] for the trajectory to be executed.
    // # All specified values are in the same order as the joint names in JointTrajectory.msg.
    //
    // # Single DOF joint positions for each joint relative to their "0" position.
    // # The units depend on the specific joint type: radians for revolute or
    // # continuous joints, and meters for prismatic joints.
    // float64[] positions
    //
    // # The rate of change in position of each joint. Units are joint type dependent.
    // # Radians/second for revolute or continuous joints, and meters/second for
    // # prismatic joints.
    // float64[] velocities
    //
    // # Rate of change in velocity of each joint. Units are joint type dependent.
    // # Radians/second^2 for revolute or continuous joints, and meters/second^2 for
    // # prismatic joints.
    // float64[] accelerations
    //
    // # The torque or the force to be applied at each joint. For revolute/continuous
    // # joints effort denotes a torque in newton-meters. For prismatic joints, effort
    // # denotes a force in newtons.
    // float64[] effort
    //
    // # Desired time from the trajectory start to arrive at this trajectory point.
    // builtin_interfaces/Duration time_from_start
    struct JointTrajectoryPoint {
        MsgPack::arr_t<double> positions;
        MsgPack::arr_t<double> velocities;
        MsgPack::arr_t<double> accelerations;
        MsgPack::arr_t<double> effort;
        rcl_interfaces::builtin_interfaces::msg::Duration time_from_start;
        MSGPACK_DEFINE(positions, velocities, accelerations, effort, time_from_start);

#ifdef ROS
        JointTrajectoryPoint& operator=(const ::trajectory_msgs::msg::JointTrajectoryPoint& rhs) {
            positions = rhs.positions;
            velocities = rhs.velocities;
            accelerations = rhs.accelerations;
            effort = rhs.effort;
            time_from_start = rhs.time_from_start;
            return *this;
        }
        JointTrajectoryPoint& operator=(::trajectory_msgs::msg::JointTrajectoryPoint&& rhs) {
            positions = std::move(rhs.positions);
            velocities = std::move(rhs.velocities);
            accelerations = std::move(rhs.accelerations);
            effort = std::move(rhs.effort);
            time_from_start = std::move(rhs.time_from_start);
            return *this;
        }
#endif
    };

    // # The header is used to specify the coordinate frame and the reference time for
    // # the trajectory durations
    // std_msgs/Header header
    //
    // # The names of the active joints in each trajectory point. These names are
    // # ordered and must correspond to the values in each trajectory point.
    // string[] joint_names
    //
    // # Array of trajectory points, which describe the positions, velocities,
    // # accelerations and/or efforts of the joints at each time point.
    // JointTrajectoryPoint[] points
    struct JointTrajectory {
        std_msgs::msg::Header header;
        MsgPack::arr_t<MsgPack::str_t> joint_names;
        MsgPack::arr_t<JointTrajectoryPoint> points;
        MSGPACK_DEFINE(header, joint_names, points);

#ifdef ROS
        JointTrajectory& operator=(const ::trajectory_msgs::msg::JointTrajectory& rhs) {
            header = rhs.header;
            joint_names = rhs.joint_names;
            points = rhs.points;
            return *this;
        }
        JointTrajectory& operator=(::trajectory_msgs::msg::JointTrajectory&& rhs) {
            header = std::move(rhs.header);
            joint_names = std::move(rhs.joint_names);
            points = std::move(rhs.points);
            return *this;
        }
#endif
    };

    // # Each multi-dof joint can specify a transform (up to 6 DOF).
    // geometry_msgs/Transform[] transforms
    //
    // # There can be a velocity specified for the origin of the joint.
    // geometry_msgs/Twist[] velocities
    //
    // # There can be an acceleration specified for the origin of the joint.
    // geometry_msgs/Twist[] accelerations
    //
    // # Desired time from the trajectory start to arrive at this trajectory point.
    // builtin_interfaces/Duration time_from_start
    struct MultiDOFJointTrajectoryPoint {
        MsgPack::arr_t<geometry_msgs::msg::Transform> transforms;
        MsgPack::arr_t<geometry_msgs::msg::Twist> velocities;
        MsgPack::arr_t<geometry_msgs::msg::Twist> accelerations;
        rcl_interfaces::builtin_interfaces::msg::Duration time_from_start;
        MSGPACK_DEFINE(transforms, velocities, accelerations, time_from_start);

#ifdef ROS
        MultiDOFJointTrajectoryPoint& operator=(const ::trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& rhs) {
            transforms = rhs.transforms;
            velocities = rhs.velocities;
            accelerations = rhs.accelerations;
            time_from_start = rhs.time_from_start;
            return *this;
        }
        MultiDOFJointTrajectoryPoint& operator=(::trajectory_msgs::msg::MultiDOFJointTrajectoryPoint&& rhs) {
            transforms = std::move(rhs.transforms);
            velocities = std::move(rhs.velocities);
            accelerations = std::move(rhs.accelerations);
            time_from_start = std::move(rhs.time_from_start);
            return *this;
        }
#endif
    };

    // # The header is used to specify the coordinate frame and the reference time for the trajectory durations
    // std_msgs/Header header
    //
    // # A representation of a multi-dof joint trajectory (each point is a transformation)
    // # Each point along the trajectory will include an array of positions/velocities/accelerations
    // # that has the same length as the array of joint names, and has the same order of joints as
    // # the joint names array.
    //
    // string[] joint_names
    // MultiDOFJointTrajectoryPoint[] points
    struct MultiDOFJointTrajectory {
        std_msgs::msg::Header header;
        MsgPack::arr_t<MsgPack::str_t> joint_names;
        MsgPack::arr_t<MultiDOFJointTrajectoryPoint> points;
        MSGPACK_DEFINE(header, joint_names, points);

#ifdef ROS
        MultiDOFJointTrajectory& operator=(const ::trajectory_msgs::msg::MultiDOFJointTrajectory& rhs) {
            header = rhs.header;
            joint_names = rhs.joint_names;
            points = rhs.points;
            return *this;
        }
        MultiDOFJointTrajectory& operator=(::trajectory_msgs::msg::MultiDOFJointTrajectory&& rhs) {
            header = std::move(rhs.header);
            joint_names = std::move(rhs.joint_names);
            points = std::move(rhs.points);
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace trajectory_msgs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_TRAJECTORY_MSGS_H
