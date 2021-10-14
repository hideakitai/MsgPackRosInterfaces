#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_GEOMETRY_MSGS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_GEOMETRY_MSGS_H

#include <array>
#include "std_msgs.h"

#ifdef ROS
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/intertia.hpp>
#include <geometry_msgs/msg/intertia_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose2d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace geometry_msgs {

namespace msg {
    // # This represents a vector in free space.
    //
    // # This is semantically different than a point.
    // # A vector is always anchored at the origin.
    // # When a transform is applied to a vector, only the rotational component is applied.
    //
    // float64 x
    // float64 y
    // float64 z
    struct Vector3 {
        double x {0.};
        double y {0.};
        double z {0.};
        MSGPACK_DEFINE(x, y, z);

#ifdef ROS
        Vector3& operator=(const ::geometry_msgs::msg::Vector3& rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
            return *this;
        }
        Vector3& operator=(::geometry_msgs::msg::Vector3&& rhs) {
            x = std::move(rhs.x);
            y = std::move(rhs.y);
            z = std::move(rhs.z);
            return *this;
        }
#endif
    };

    // # This expresses acceleration in free space broken into its linear and angular parts.
    // Vector3  linear
    // Vector3  angular
    struct Accel {
        Vector3 linear;
        Vector3 angular;
        MSGPACK_DEFINE(linear, angular);

#ifdef ROS
        Accel& operator=(const ::geometry_msgs::msg::Accel& rhs) {
            linear = rhs.linear;
            angular = rhs.angular;
            return *this;
        }
        Accel& operator=(::geometry_msgs::msg::Accel&& rhs) {
            linear = std::move(rhs.linear);
            angular = std::move(rhs.angular);
            return *this;
        }
#endif
    };

    // # An accel with reference coordinate frame and timestamp
    // std_msgs/Header header
    // Accel accel
    struct AccelStamped {
        std_msgs::msg::Header header;
        Accel accel;
        MSGPACK_DEFINE(header, accel);

#ifdef ROS
        AccelStamped& operator=(const ::geometry_msgs::msg::AccelStamped& rhs) {
            header = rhs.header;
            accel = rhs.accel;
            return *this;
        }
        AccelStamped& operator=(::geometry_msgs::msg::AccelStamped&& rhs) {
            header = std::move(rhs.header);
            accel = std::move(rhs.accel);
            return *this;
        }
#endif
    };

    // # This expresses acceleration in free space with uncertainty.
    //
    // Accel accel
    //
    // # Row-major representation of the 6x6 covariance matrix
    // # The orientation parameters use a fixed-axis representation.
    // # In order, the parameters are:
    // # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    // float64[36] covariance
    struct AccelWithCovariance {
        Accel accel;
        std::array<double, 36> covariance {};
        MSGPACK_DEFINE(accel, covariance);

#ifdef ROS
        AccelWithCovariance& operator=(const ::geometry_msgs::msg::AccelWithCovariance& rhs) {
            accel = rhs.accel;
            covariance = rhs.covariance;
            return *this;
        }
        AccelWithCovariance& operator=(::geometry_msgs::msg::AccelWithCovariance&& rhs) {
            accel = std::move(rhs.accel);
            covariance = std::move(rhs.covariance);
            return *this;
        }
#endif
    };

    // # This represents an estimated accel with reference coordinate frame and timestamp.
    // std_msgs/Header header
    // AccelWithCovariance accel
    struct AccelWithCovarianceStamped {
        std_msgs::msg::Header header;
        AccelWithCovariance accel;
        MSGPACK_DEFINE(header, accel);

#ifdef ROS
        AccelWithCovarianceStamped& operator=(const ::geometry_msgs::msg::AccelWithCovarianceStamped& rhs) {
            header = rhs.header;
            accel = rhs.accel;
            return *this;
        }
        AccelWithCovarianceStamped& operator=(::geometry_msgs::msg::AccelWithCovarianceStamped&& rhs) {
            header = std::move(rhs.header);
            accel = std::move(rhs.accel);
            return *this;
        }
#endif
    };

    // # Mass [kg]
    // float64 m
    //
    // # Center of mass [m]
    // geometry_msgs/Vector3 com
    //
    // # Inertia Tensor [kg-m^2]
    // #     | ixx ixy ixz |
    // # I = | ixy iyy iyz |
    // #     | ixz iyz izz |
    // float64 ixx
    // float64 ixy
    // float64 ixz
    // float64 iyy
    // float64 iyz
    // float64 izz
    struct Inertia {
        double m {0.};
        Vector3 com;
        double ixx {0.};
        double ixy {0.};
        double ixz {0.};
        double iyy {0.};
        double iyz {0.};
        double izz {0.};
        MSGPACK_DEFINE(m, com, ixx, ixy, ixz, iyy, iyz, izz);

#ifdef ROS
        Inertia& operator=(const ::geometry_msgs::msg::Inertia& rhs) {
            m = rhs.m;
            com = rhs.com;
            ixx = rhs.ixx;
            ixy = rhs.ixy;
            ixz = rhs.ixz;
            iyy = rhs.iyy;
            iyz = rhs.iyz;
            izz = rhs.izz;
            return *this;
        }
        Inertia& operator=(::geometry_msgs::msg::Inertia&& rhs) {
            m = std::move(rhs.m);
            com = std::move(rhs.com);
            ixx = std::move(rhs.ixx);
            ixy = std::move(rhs.ixy);
            ixz = std::move(rhs.ixz);
            iyy = std::move(rhs.iyy);
            iyz = std::move(rhs.iyz);
            izz = std::move(rhs.izz);
            return *this;
        }
#endif
    };

    // # An Inertia with a time stamp and reference frame.
    //
    // std_msgs/Header header
    // Inertia inertia
    struct InertiaStamped {
        std_msgs::msg::Header header;
        Inertia inertia;
        MSGPACK_DEFINE(header, inertia);

#ifdef ROS
        InertiaStamped& operator=(const ::geometry_msgs::msg::InertiaStamped& rhs) {
            header = rhs.header;
            inertia = rhs.inertia;
            return *this;
        }
        InertiaStamped& operator=(::geometry_msgs::msg::InertiaStamped&& rhs) {
            header = std::move(rhs.header);
            inertia = std::move(rhs.inertia);
            return *this;
        }
#endif
    };

    // # This contains the position of a point in free space
    // float64 x
    // float64 y
    // float64 z
    struct Point {
        double x {0.};
        double y {0.};
        double z {0.};
        MSGPACK_DEFINE(x, y, z);

#ifdef ROS
        Point& operator=(const ::geometry_msgs::msg::Point& rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
            return *this;
        }
        Point& operator=(::geometry_msgs::msg::Point&& rhs) {
            x = std::move(rhs.x);
            y = std::move(rhs.y);
            z = std::move(rhs.z);
            return *this;
        }
#endif
    };

    // # This contains the position of a point in free space(with 32 bits of precision).
    // # It is recommended to use Point wherever possible instead of Point32.
    // #
    // # This recommendation is to promote interoperability.
    // #
    // # This message is designed to take up less space when sending
    // # lots of points at once, as in the case of a PointCloud.
    //
    // float32 x
    // float32 y
    // float32 z
    struct Point32 {
        float x {0.f};
        float y {0.f};
        float z {0.f};
        MSGPACK_DEFINE(x, y, z);

#ifdef ROS
        Point32& operator=(const ::geometry_msgs::msg::Point32& rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
            return *this;
        }
        Point32& operator=(::geometry_msgs::msg::Point32&& rhs) {
            x = std::move(rhs.x);
            y = std::move(rhs.y);
            z = std::move(rhs.z);
            return *this;
        }
#endif
    };

    // # This represents a Point with reference coordinate frame and timestamp
    //
    // std_msgs/Header header
    // Point point
    struct PointStamped {
        std_msgs::msg::Header header;
        Point point;
        MSGPACK_DEFINE(header, point);

#ifdef ROS
        PointStamped& operator=(const ::geometry_msgs::msg::PointStamped& rhs) {
            header = rhs.header;
            point = rhs.point;
            return *this;
        }
        PointStamped& operator=(::geometry_msgs::msg::PointStamped&& rhs) {
            header = std::move(rhs.header);
            point = std::move(rhs.point);
            return *this;
        }
#endif
    };

    // # A specification of a polygon where the first and last points are assumed to be connected
    //
    // Point32[] points
    struct Polygon {
        MsgPack::arr_t<Point32> points;
        MSGPACK_DEFINE(points);

#ifdef ROS
        Polygon& operator=(const ::geometry_msgs::msg::Polygon& rhs) {
            points = rhs.points;
            return *this;
        }
        Polygon& operator=(::geometry_msgs::msg::Polygon&& rhs) {
            points = std::move(rhs.points);
            return *this;
        }
#endif
    };

    // # This represents a Polygon with reference coordinate frame and timestamp
    //
    // std_msgs/Header header
    // Polygon polygon
    struct PolygonStamped {
        std_msgs::msg::Header header;
        Polygon polygon;
        MSGPACK_DEFINE(header, polygon);

#ifdef ROS
        PolygonStamped& operator=(const ::geometry_msgs::msg::PolygonStamped& rhs) {
            header = rhs.header;
            polygon = rhs.polygon;
            return *this;
        }
        PolygonStamped& operator=(::geometry_msgs::msg::PolygonStamped&& rhs) {
            header = std::move(rhs.header);
            polygon = std::move(rhs.polygon);
            return *this;
        }
#endif
    };

    // # This represents an orientation in free space in quaternion form.
    //
    // float64 x 0
    // float64 y 0
    // float64 z 0
    // float64 w 1
    struct Quaternion {
        double x {0.};
        double y {0.};
        double z {0.};
        double w {1.};
        MSGPACK_DEFINE(x, y, z, w);

#ifdef ROS
        Quaternion& operator=(const ::geometry_msgs::msg::Quaternion& rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
            w = rhs.w;
            return *this;
        }
        Quaternion& operator=(::geometry_msgs::msg::Quaternion&& rhs) {
            x = std::move(rhs.x);
            y = std::move(rhs.y);
            z = std::move(rhs.z);
            w = std::move(rhs.w);
            return *this;
        }
#endif
    };

    // # A representation of pose in free space, composed of position and orientation.
    //
    // Point position
    // Quaternion orientation
    struct Pose {
        Point position;
        Quaternion orientation;
        MSGPACK_DEFINE(position, orientation);

#ifdef ROS
        Pose& operator=(const ::geometry_msgs::msg::Pose& rhs) {
            position = rhs.position;
            orientation = rhs.orientation;
            return *this;
        }
        Pose& operator=(::geometry_msgs::msg::Pose&& rhs) {
            position = std::move(rhs.position);
            orientation = std::move(rhs.orientation);
            return *this;
        }
#endif
    };

    // # Deprecated as of Foxy and will potentially be removed in any following release.
    // # Please use the full 3D pose.
    //
    // # In general our recommendation is to use a full 3D representation of everything and for 2D specific
    // applications make the appropriate projections into the plane for their calculations but optimally will preserve
    // the 3D information during processing.
    //
    // # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual
    // interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if
    // they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not
    // particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools
    // and libraries that can do this for you.# This expresses a position and orientation on a 2D manifold.
    //
    // float64 x
    // float64 y
    // float64 theta
    struct Pose2D {
        double x {0.};
        double y {0.};
        double theta {0.};
        MSGPACK_DEFINE(x, y, theta);

#ifdef ROS
        Pose2D& operator=(const ::geometry_msgs::msg::Pose2D& rhs) {
            x = rhs.x;
            y = rhs.y;
            theta = rhs.theta;
            return *this;
        }
        Pose2D& operator=(::geometry_msgs::msg::Pose2D&& rhs) {
            x = std::move(rhs.x);
            y = std::move(rhs.y);
            theta = std::move(rhs.theta);
            return *this;
        }
#endif
    };

    // # An array of poses with a header for global reference.
    //
    // std_msgs/Header header
    //
    // Pose[] poses
    struct PoseArray {
        std_msgs::msg::Header header;
        MsgPack::arr_t<Pose> poses;
        MSGPACK_DEFINE(header, poses);

#ifdef ROS
        PoseArray& operator=(const ::geometry_msgs::msg::PoseArray& rhs) {
            header = rhs.header;
            poses = rhs.poses;
            return *this;
        }
        PoseArray& operator=(::geometry_msgs::msg::PoseArray&& rhs) {
            header = std::move(rhs.header);
            poses = std::move(rhs.poses);
            return *this;
        }
#endif
    };

    // # A Pose with reference coordinate frame and timestamp
    //
    // std_msgs/Header header
    // Pose pose
    struct PoseStamped {
        std_msgs::msg::Header header;
        Pose pose;
        MSGPACK_DEFINE(header, pose);

#ifdef ROS
        PoseStamped& operator=(const ::geometry_msgs::msg::PoseStamped& rhs) {
            header = rhs.header;
            pose = rhs.pose;
            return *this;
        }
        PoseStamped& operator=(::geometry_msgs::msg::PoseStamped&& rhs) {
            header = std::move(rhs.header);
            pose = std::move(rhs.pose);
            return *this;
        }
#endif
    };

    // # This represents a pose in free space with uncertainty.
    //
    // Pose pose
    //
    // # Row-major representation of the 6x6 covariance matrix
    // # The orientation parameters use a fixed-axis representation.
    // # In order, the parameters are:
    // # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    // float64[36] covariance
    struct PoseWithCovariance {
        Pose pose;
        std::array<double, 36> covariance;
        MSGPACK_DEFINE(pose, covariance);

#ifdef ROS
        PoseWithCovariance& operator=(const ::geometry_msgs::msg::PoseWithCovariance& rhs) {
            pose = rhs.pose;
            covariance = rhs.covariance;
            return *this;
        }
        PoseWithCovariance& operator=(::geometry_msgs::msg::PoseWithCovariance&& rhs) {
            pose = std::move(rhs.pose);
            covariance = std::move(rhs.covariance);
            return *this;
        }
#endif
    };

    // # This expresses an estimated pose with a reference coordinate frame and timestamp
    //
    // std_msgs/Header header
    // PoseWithCovariance pose
    struct PoseWithCovarianceStamped {
        std_msgs::msg::Header header;
        PoseWithCovariance pose;
        MSGPACK_DEFINE(header, pose);

#ifdef ROS
        PoseWithCovarianceStamped& operator=(const ::geometry_msgs::msg::PoseWithCovarianceStamped& rhs) {
            header = rhs.header;
            pose = rhs.pose;
            return *this;
        }
        PoseWithCovarianceStamped& operator=(::geometry_msgs::msg::PoseWithCovarianceStamped&& rhs) {
            header = std::move(rhs.header);
            pose = std::move(rhs.pose);
            return *this;
        }
#endif
    };

    // # This represents an orientation with reference coordinate frame and timestamp.
    //
    // std_msgs/Header header
    // Quaternion quaternion
    struct QuaternionStamped {
        std_msgs::msg::Header header;
        Quaternion quaternion;
        MSGPACK_DEFINE(header, quaternion);

#ifdef ROS
        QuaternionStamped& operator=(const ::geometry_msgs::msg::QuaternionStamped& rhs) {
            header = rhs.header;
            quaternion = rhs.quaternion;
            return *this;
        }
        QuaternionStamped& operator=(::geometry_msgs::msg::QuaternionStamped&& rhs) {
            header = std::move(rhs.header);
            quaternion = std::move(rhs.quaternion);
            return *this;
        }
#endif
    };

    // # This represents the transform between two coordinate frames in free space.
    //
    // Vector3 translation
    // Quaternion rotation
    struct Transform {
        Vector3 translation;
        Quaternion rotation;
        MSGPACK_DEFINE(translation, rotation);

#ifdef ROS
        Transform& operator=(const ::geometry_msgs::msg::Transform& rhs) {
            translation = rhs.translation;
            rotation = rhs.rotation;
            return *this;
        }
        Transform& operator=(::geometry_msgs::msg::Transform&& rhs) {
            translation = std::move(rhs.translation);
            rotation = std::move(rhs.rotation);
            return *this;
        }
#endif
    };

    // # This expresses a transform from coordinate frame header.frame_id
    // # to the coordinate frame child_frame_id at the time of header.stamp
    // #
    // # This message is mostly used by the
    // # <a href="https://index.ros.org/p/tf2/">tf2</a> package.
    // # See its documentation for more information.
    // #
    // # The child_frame_id is necessary in addition to the frame_id
    // # in the Header to communicate the full reference for the transform
    // # in a self contained message.
    //
    // # The frame id in the header is used as the reference frame of this transform.
    // std_msgs/Header header
    //
    // # The frame id of the child frame to which this transform points.
    // string child_frame_id
    //
    // # Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
    // Transform transform
    struct TransformStamped {
        std_msgs::msg::Header header;
        MsgPack::str_t child_frame_id;
        Transform transform;
        MSGPACK_DEFINE(header, child_frame_id, transform);

#ifdef ROS
        TransformStamped& operator=(const ::geometry_msgs::msg::TransformStamped& rhs) {
            header = rhs.header;
            child_frame_id = rhs.child_frame_id;
            transform = rhs.transform;
            return *this;
        }
        TransformStamped& operator=(::geometry_msgs::msg::TransformStamped&& rhs) {
            header = std::move(rhs.header);
            child_frame_id = std::move(rhs.child_frame_id);
            transform = std::move(rhs.transform);
            return *this;
        }
#endif
    };

    // # This expresses velocity in free space broken into its linear and angular parts.
    //
    // Vector3  linear
    // Vector3  angular
    struct Twist {
        Vector3 linear;
        Vector3 angular;
        MSGPACK_DEFINE(linear, angular);

#ifdef ROS
        Twist& operator=(const ::geometry_msgs::msg::Twist& rhs) {
            linear = rhs.linear;
            angular = rhs.angular;
            return *this;
        }
        Twist& operator=(::geometry_msgs::msg::Twist&& rhs) {
            linear = std::move(rhs.linear);
            angular = std::move(rhs.angular);
            return *this;
        }
#endif
    };

    // # A twist with reference coordinate frame and timestamp
    //
    // std_msgs/Header header
    // Twist twist
    struct TwistStamped {
        std_msgs::msg::Header header;
        Twist twist;
        MSGPACK_DEFINE(header, twist);

#ifdef ROS
        TwistStamped& operator=(const ::geometry_msgs::msg::TwistStamped& rhs) {
            header = rhs.header;
            twist = rhs.twist;
            return *this;
        }
        TwistStamped& operator=(::geometry_msgs::msg::TwistStamped&& rhs) {
            header = std::move(rhs.header);
            twist = std::move(rhs.twist);
            return *this;
        }
#endif
    };

    // # This expresses velocity in free space with uncertainty.
    //
    // Twist twist
    //
    // # Row-major representation of the 6x6 covariance matrix
    // # The orientation parameters use a fixed-axis representation.
    // # In order, the parameters are:
    // # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    // float64[36] covariance
    struct TwistWithCovariance {
        Twist twist;
        std::array<double, 36> covariance;
        MSGPACK_DEFINE(twist, covariance);

#ifdef ROS
        TwistWithCovariance& operator=(const ::geometry_msgs::msg::TwistCovariance& rhs) {
            twist = rhs.twist;
            covariance = rhs.covariance;
            return *this;
        }
        TwistCovariance& operator=(::geometry_msgs::msg::TwistCovariance&& rhs) {
            header = std::move(rhs.header);
            covariance = std::move(rhs.covariance);
            return *this;
        }
#endif
    };

    // # This represents an estimated twist with reference coordinate frame and timestamp.
    //
    // std_msgs/Header header
    // TwistWithCovariance twist
    struct TwistWithCovarianceStamped {
        std_msgs::msg::Header header;
        TwistWithCovariance twist;
        MSGPACK_DEFINE(header, twist);

#ifdef ROS
        TwistWithCovarianceStamped& operator=(const ::geometry_msgs::msg::TwistWithCovarianceStamped& rhs) {
            header = rhs.header;
            twist = rhs.twist;
            return *this;
        }
        TwistWithCovarianceStamped& operator=(::geometry_msgs::msg::TwistWithCovarianceStamped&& rhs) {
            header = std::move(rhs.header);
            twist = std::move(rhs.twist);
            return *this;
        }
#endif
    };

    // # This represents a Vector3 with reference coordinate frame and timestamp
    //
    // # Note that this follows vector semantics with it always anchored at the origin,
    // # so the rotational elements of a transform are the only parts applied when transforming.
    //
    // std_msgs/Header header
    // Vector3 vector
    struct Vector3Stamped {
        std_msgs::msg::Header header;
        Vector3 vector;
        MSGPACK_DEFINE(header, vector);

#ifdef ROS
        Vector3Stamped& operator=(const ::geometry_msgs::msg::Vector3Stamped& rhs) {
            header = rhs.header;
            vector = rhs.vector;
            return *this;
        }
        Vector3Stamped& operator=(::geometry_msgs::msg::Vector3Stamped&& rhs) {
            header = std::move(rhs.header);
            vector = std::move(rhs.vector);
            return *this;
        }
#endif
    };

    // # This represents force in free space, separated into its linear and angular parts.
    //
    // Vector3  force
    // Vector3  torque
    struct Wrench {
        Vector3 force;
        Vector3 torque;
        MSGPACK_DEFINE(force, torque);

#ifdef ROS
        Wrench& operator=(const ::geometry_msgs::msg::Wrench& rhs) {
            force = rhs.force;
            torque = rhs.torque;
            return *this;
        }
        Wrench& operator=(::geometry_msgs::msg::Wrench&& rhs) {
            force = std::move(rhs.force);
            torque = std::move(rhs.torque);
            return *this;
        }
#endif
    };

    // # A wrench with reference coordinate frame and timestamp
    //
    // std_msgs/Header header
    // Wrench wrench
    struct WrenchStamped {
        std_msgs::msg::Header header;
        Wrench wrench;
        MSGPACK_DEFINE(header, wrench);

#ifdef ROS
        WrenchStamped& operator=(const ::geometry_msgs::msg::WrenchStamped& rhs) {
            header = rhs.header;
            wrench = rhs.wrench;
            return *this;
        }
        WrenchStamped& operator=(::geometry_msgs::msg::WrenchStamped&& rhs) {
            header = std::move(rhs.header);
            wrench = std::move(rhs.wrench);
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace geometry_msgs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_GEOMETRY_MSGS_H
