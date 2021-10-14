#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAV_MSGS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAV_MSGS_H

#include "../rcl_interfaces/builtin_interfaces.h"
#include "std_msgs.h"
#include "geometry_msgs.h"

#ifdef ROS
#include <std_msgs/msg/Header.hpp>
#include <nav_msgs/msg/grid_cell.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace nav_msgs {

namespace msg {

    // # An array of cells in a 2D grid
    //
    // std_msgs/Header header
    //
    // # Width of each cell
    // float32 cell_width
    //
    // # Height of each cell
    // float32 cell_height
    //
    // # Each cell is represented by the Point at the center of the cell
    // geometry_msgs/Point[] cells
    struct GridCells {
        std_msgs::msg::Header header;
        float cell_width;
        float cell_height;
        MsgPack::arr_t<geometry_msgs::msg::Point> cells;
        MSGPACK_DEFINE(header, cell_width, cell_height, cells);

#ifdef ROS
        GridCells& operator=(const ::nav_msgs::msg::GridCells& rhs) {
            header = rhs.header;
            cell_width = rhs.cell_width;
            cell_height = rhs.cell_height;
            cells = rhs.cells;
            return *this;
        }
        GridCells& operator=(::nav_msgs::msg::GridCells&& rhs) {
            header = std::move(rhs.header);
            cell_width = std::move(rhs.cell_width);
            cell_height = std::move(rhs.cell_height);
            cells = std::move(rhs.cells);
            return *this;
        }
#endif
    };

    // # This hold basic information about the characteristics of the OccupancyGrid
    //
    // # The time at which the map was loaded
    // builtin_interfaces/Time map_load_time
    //
    // # The map resolution [m/cell]
    // float32 resolution
    //
    // # Map width [cells]
    // uint32 width
    //
    // # Map height [cells]
    // uint32 height
    //
    // # The origin of the map [m, m, rad].  This is the real-world pose of the
    // # bottom left corner of cell (0,0) in the map.
    // geometry_msgs/Pose origin
    struct MapMetaData {
        rcl_interfaces::builtin_interfaces::msg::Time map_load_time;
        float resolution;
        uint32_t width;
        uint32_t height;
        geometry_msgs::msg::Pose origin;
        MSGPACK_DEFINE(resolution, width, height, origin);

#ifdef ROS
        MapMetaData& operator=(const ::nav_msgs::msg::MapMetaData& rhs) {
            map_load_time = rhs.map_load_time;
            resolution = rhs.resolution;
            width = rhs.width;
            height = rhs.height;
            origin = rhs.origin;
            return *this;
        }
        MapMetaData& operator=(::nav_msgs::msg::MapMetaData&& rhs) {
            map_load_time = std::move(rhs.map_load_time);
            resolution = std::move(rhs.resolution);
            width = std::move(rhs.width);
            height = std::move(rhs.height);
            origin = std::move(rhs.origin);
            return *this;
        }
#endif
    };

    // # This represents a 2-D grid map
    // std_msgs/Header header
    //
    // # MetaData for the map
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).
    // # Cell (1, 0) will be listed second, representing the next cell in the x direction.
    // # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
    // # The values inside are application dependent, but frequently,
    // # 0 represents unoccupied, 1 represents definitely occupied, and
    // # -1 represents unknown.
    // int8[] data
    struct OccupancyGrid {
        std_msgs::msg::Header header;
        MapMetaData info;
        MsgPack::arr_t<int8_t> data;
        MSGPACK_DEFINE(header, info, data);

#ifdef ROS
        OccupancyGrid& operator=(const ::nav_msgs::msg::OccupancyGrid& rhs) {
            header = rhs.header;
            info = rhs.info;
            data = rhs.data;
            return *this;
        }
        OccupancyGrid& operator=(::nav_msgs::msg::OccupancyGrid&& rhs) {
            header = std::move(rhs.header);
            info = std::move(rhs.info);
            data = std::move(rhs.data);
            return *this;
        }
#endif
    };

    // # This represents an estimate of a position and velocity in free space.
    // # The pose in this message should be specified in the coordinate frame given by header.frame_id
    // # The twist in this message should be specified in the coordinate frame given by the child_frame_id
    //
    // # Includes the frame id of the pose parent.
    // std_msgs/Header header
    //
    // # Frame id the pose points to. The twist is in this coordinate frame.
    // string child_frame_id
    //
    // # Estimated pose that is typically relative to a fixed world frame.
    // geometry_msgs/PoseWithCovariance pose
    //
    // # Estimated linear and angular velocity relative to child_frame_id.
    // geometry_msgs/TwistWithCovariance twist
    struct Odometry {
        std_msgs::msg::Header header;
        MsgPack::str_t child_frame_id;
        geometry_msgs::msg::PoseWithCovariance pose;
        geometry_msgs::msg::TwistWithCovariance twist;
        MSGPACK_DEFINE(header, child_frame_id, pose, twist);

#ifdef ROS
        Odometry& operator=(const ::nav_msgs::msg::Odometry& rhs) {
            header = rhs.header;
            child_frame_id = rhs.child_frame_id;
            pose = rhs.pose;
            twist = rhs.twist;
            return *this;
        }
        Odometry& operator=(::nav_msgs::msg::Odometry&& rhs) {
            header = std::move(rhs.header);
            child_frame_id = std::move(rhs.child_frame_id);
            pose = std::move(rhs.pose);
            twist = std::move(rhs.twist);
            return *this;
        }
#endif
    };

    // # An array of poses that represents a Path for a robot to follow.
    //
    // # Indicates the frame_id of the path.
    // std_msgs/Header header
    //
    // # Array of poses to follow.
    // geometry_msgs/PoseStamped[] poses
    struct Path {
        std_msgs::msg::Header header;
        MsgPack::arr_t<geometry_msgs::msg::PoseStamped> poses;
        MSGPACK_DEFINE(header, poses);

#ifdef ROS
        Path& operator=(const ::nav_msgs::msg::Path& rhs) {
            header = rhs.header;
            poses = rhs.poses;
            return *this;
        }
        Path& operator=(::nav_msgs::msg::Path&& rhs) {
            header = std::move(rhs.header);
            poses = rhs.poses;
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace nav_msgs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAV_MSGS_H
