#pragma once
#ifndef ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_SENSOR_MSGS_H
#define ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_SENSOR_MSGS_H

#ifdef ROS
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joint_feedback_array.hpp>
#include <sensor_msgs/msg/joint_feedback.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#endif

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_BEGIN

namespace sensor_msgs {

namespace msg {
    // # Constants are chosen to match the enums in the linux kernel
    // # defined in include/linux/power_supply.h as of version 3.7
    // # The one difference is for style reasons the constants are
    // # all uppercase not mixed case.
    //
    // # Power supply status constants
    // uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
    // uint8 POWER_SUPPLY_STATUS_CHARGING = 1
    // uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
    // uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
    // uint8 POWER_SUPPLY_STATUS_FULL = 4
    //
    // # Power supply health constants
    // uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
    // uint8 POWER_SUPPLY_HEALTH_GOOD = 1
    // uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
    // uint8 POWER_SUPPLY_HEALTH_DEAD = 3
    // uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
    // uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
    // uint8 POWER_SUPPLY_HEALTH_COLD = 6
    // uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
    // uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8
    //
    // # Power supply technology (chemistry) constants
    // uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
    // uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
    // uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
    // uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
    // uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
    // uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
    // uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6
    //
    // std_msgs/Header  header
    // float32 voltage          # Voltage in Volts (Mandatory)
    // float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
    // float32 current          # Negative when discharging (A)  (If unmeasured NaN)
    // float32 charge           # Current charge in Ah  (If unmeasured NaN)
    // float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
    // float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
    // float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
    // uint8   power_supply_status     # The charging status as reported. Values defined above
    // uint8   power_supply_health     # The battery health metric. Values defined above
    // uint8   power_supply_technology # The battery chemistry. Values defined above
    // bool    present          # True if the battery is present
    //
    // float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
    //                          # If individual voltages unknown but number of cells known set each to NaN
    // float32[] cell_temperature # An array of individual cell temperatures for each cell in the pack
    //                            # If individual temperatures unknown but number of cells known set each to NaN
    // string location          # The location into which the battery is inserted. (slot number or plug)
    // string serial_number     # The best approximation of the battery serial number
    struct BatteryState {
        // Power supply status constants
        static constexpr uint8_t POWER_SUPPLY_STATUS_UNKNOWN = 0;
        static constexpr uint8_t POWER_SUPPLY_STATUS_CHARGING = 1;
        static constexpr uint8_t POWER_SUPPLY_STATUS_DISCHARGING = 2;
        static constexpr uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
        static constexpr uint8_t POWER_SUPPLY_STATUS_FULL = 4;

        // Power supply health constants
        static constexpr uint8_t POWER_SUPPLY_HEALTH_UNKNOWN = 0;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_GOOD = 1;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERHEAT = 2;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_DEAD = 3;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_COLD = 6;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8;

        // Power supply technology (chemistry) constants
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NIMH = 1;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LION = 2;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIPO = 3;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIFE = 4;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NICD = 5;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIMN = 6;

        std_msgs::msg::Header header;
        float voltage;                    // Voltage in Volts (Mandatory)
        float temperature;                // Temperature in Degrees Celsius (If unmeasured NaN)
        float current;                    // Negative when discharging (A)  (If unmeasured NaN)
        float charge;                     // Current charge in Ah  (If unmeasured NaN)
        float capacity;                   // Capacity in Ah (last full capacity)  (If unmeasured NaN)
        float design_capacity;            // Capacity in Ah (design capacity)  (If unmeasured NaN)
        float percentage;                 // Charge percentage on 0 to 1 range  (If unmeasured NaN)
        uint8_t power_supply_status;      // The charging status as reported. Values defined above
        uint8_t power_supply_health;      // The battery health metric. Values defined above
        uint8_t power_supply_technology;  // The battery chemistry. Values defined above
        bool present;                     // True if the battery is present

        MsgPack::arr_t<float> cell_voltage;      // An array of individual cell voltages for each cell in the pack
                                                 // If individual voltages unknown but number of cells known set to NaN
        MsgPack::arr_t<float> cell_temperature;  // An array of individual cell temperatures for each cell in the pack
                                                 // If individual temp unknown but number of cells known set to NaN
        MsgPack::str_t location;       // The location into which the battery is inserted. (slot number or plug)
        MsgPack::str_t serial_number;  // The best approximation of the battery serial number

        MSGPACK_DEFINE(
            header,
            voltage,
            temperature,
            current,
            charge,
            capacity,
            design_capacity,
            percentage,
            power_supply_status,
            power_supply_health,
            power_supply_technology,
            present,
            cell_voltage,
            cell_temperature,
            location,
            serial_number);

#ifdef ROS
        BatteryState& operator=(const ::sensor_msgs::msg::BatteryState& rhs) {
            header = rhs.header;
            voltage = rhs.voltage;
            temperature = rhs.temperature;
            current = rhs.current;
            charge = rhs.charge;
            capacity = rhs.capacity;
            design_capacity = rhs.design_capacity;
            percentage = rhs.percentage;
            power_supply_status = rhs.power_supply_status;
            power_supply_health = rhs.power_supply_health;
            power_supply_technology = rhs.power_supply_technology;
            present = rhs.present;
            cell_voltage = rhs.cell_voltage;
            cell_temperature = rhs.cell_temperature;
            location = rhs.location;
            serial_number = rhs.serial_number;
            return *this;
        }
        BatteryState& operator=(::sensor_msgs::msg::BatteryState&& rhs) {
            header = std::move(rhs.header);
            voltage = std::move(rhs.voltage);
            temperature = std::move(rhs.temperature);
            current = std::move(rhs.current);
            charge = std::move(rhs.charge);
            capacity = std::move(rhs.capacity);
            design_capacity = std::move(rhs.design_capacity);
            percentage = std::move(rhs.percentage);
            power_supply_status = std::move(rhs.power_supply_status);
            power_supply_health = std::move(rhs.power_supply_health);
            power_supply_technology = std::move(rhs.power_supply_technology);
            present = std::move(rhs.present);
            cell_voltage = std::move(rhs.cell_voltage);
            cell_temperature = std::move(rhs.cell_temperature);
            location = std::move(rhs.location);
            serial_number = std::move(rhs.serial_number);
            return *this;
        }
#endif
    };

    // # This message is used to specify a region of interest within an image.
    // #
    // # When used to specify the ROI setting of the camera when the image was
    // # taken, the height and width fields should either match the height and
    // # width fields for the associated image; or height = width = 0
    // # indicates that the full resolution image was captured.
    //
    // uint32 x_offset  # Leftmost pixel of the ROI
    //                  # (0 if the ROI includes the left edge of the image)
    // uint32 y_offset  # Topmost pixel of the ROI
    //                  # (0 if the ROI includes the top edge of the image)
    // uint32 height    # Height of ROI
    // uint32 width     # Width of ROI
    //
    // # True if a distinct rectified ROI should be calculated from the "raw"
    // # ROI in this message. Typically this should be False if the full image
    // # is captured (ROI not used), and True if a subwindow is captured (ROI
    // # used).
    // bool do_rectify
    struct RegionOfInterest {
        uint32_t x_offset;
        uint32_t y_offset;
        uint32_t height;
        uint32_t width;
        bool do_rectify;
        MSGPACK_DEFINE(x_offset, y_offset, height, width, do_rectify);

#ifdef ROS
        RegionOfInterest& operator=(const ::sensor_msgs::msg::RegionOfInterest& rhs) {
            x_offset = rhs.x_offset;
            y_offset = rhs.y_offset;
            height = rhs.height;
            width = rhs.width;
            do_rectify = rhs.do_rectify;
            return *this;
        }
        RegionOfInterest& operator=(::sensor_msgs::msg::RegionOfInterest&& rhs) {
            x_offset = std::move(rhs.x_offset);
            y_offset = std::move(rhs.y_offset);
            height = std::move(rhs.height);
            width = std::move(rhs.width);
            do_rectify = std::move(rhs.do_rectify);
            return *this;
        }
#endif
    };

    // # This message defines meta information for a camera. It should be in a
    // # camera namespace on topic "camera_info" and accompanied by up to five
    // # image topics named:
    // #
    // #   image_raw - raw data from the camera driver, possibly Bayer encoded
    // #   image            - monochrome, distorted
    // #   image_color      - color, distorted
    // #   image_rect       - monochrome, rectified
    // #   image_rect_color - color, rectified
    // #
    // # The image_pipeline contains packages (image_proc, stereo_image_proc)
    // # for producing the four processed image topics from image_raw and
    // # camera_info. The meaning of the camera parameters are described in
    // # detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
    // #
    // # The image_geometry package provides a user-friendly interface to
    // # common operations using this meta information. If you want to, e.g.,
    // # project a 3d point into image coordinates, we strongly recommend
    // # using image_geometry.
    // #
    // # If the camera is uncalibrated, the matrices D, K, R, P should be left
    // # zeroed out. In particular, clients may assume that K[0] == 0.0
    // # indicates an uncalibrated camera.
    //
    // #######################################################################
    // #                     Image acquisition info                          #
    // #######################################################################
    //
    // # Time of image acquisition, camera coordinate frame ID
    // std_msgs/Header header # Header timestamp should be acquisition time of image
    //                              # Header frame_id should be optical frame of camera
    //                              # origin of frame should be optical center of camera
    //                              # +x should point to the right in the image
    //                              # +y should point down in the image
    //                              # +z should point into the plane of the image
    //
    //
    // #######################################################################
    // #                      Calibration Parameters                         #
    // #######################################################################
    // # These are fixed during camera calibration. Their values will be the #
    // # same in all messages until the camera is recalibrated. Note that    #
    // # self-calibrating systems may "recalibrate" frequently.              #
    // #                                                                     #
    // # The internal parameters can be used to warp a raw (distorted) image #
    // # to:                                                                 #
    // #   1. An undistorted image (requires D and K)                        #
    // #   2. A rectified image (requires D, K, R)                           #
    // # The projection matrix P projects 3D points into the rectified image.#
    // #######################################################################
    //
    // # The image dimensions with which the camera was calibrated.
    // # Normally this will be the full camera resolution in pixels.
    // uint32 height
    // uint32 width
    //
    // # The distortion model used. Supported models are listed in
    // # sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
    // # simple model of radial and tangential distortion - is sufficent.
    // string distortion_model
    //
    // # The distortion parameters, size depending on the distortion model.
    // # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    // float64[] d
    //
    // # Intrinsic camera matrix for the raw (distorted) images.
    // #     [fx  0 cx]
    // # K = [ 0 fy cy]
    // #     [ 0  0  1]
    // # Projects 3D points in the camera coordinate frame to 2D pixel
    // # coordinates using the focal lengths (fx, fy) and principal point
    // # (cx, cy).
    // float64[9]  k # 3x3 row-major matrix
    //
    // # Rectification matrix (stereo cameras only)
    // # A rotation matrix aligning the camera coordinate system to the ideal
    // # stereo image plane so that epipolar lines in both stereo images are
    // # parallel.
    // float64[9]  r # 3x3 row-major matrix
    //
    // # Projection/camera matrix
    // #     [fx'  0  cx' Tx]
    // # P = [ 0  fy' cy' Ty]
    // #     [ 0   0   1   0]
    // # By convention, this matrix specifies the intrinsic (camera) matrix
    // #  of the processed (rectified) image. That is, the left 3x3 portion
    // #  is the normal camera intrinsic matrix for the rectified image.
    // # It projects 3D points in the camera coordinate frame to 2D pixel
    // #  coordinates using the focal lengths (fx', fy') and principal point
    // #  (cx', cy') - these may differ from the values in K.
    // # For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
    // #  also have R = the identity and P[1:3,1:3] = K.
    // # For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    // #  position of the optical center of the second camera in the first
    // #  camera's frame. We assume Tz = 0 so both cameras are in the same
    // #  stereo image plane. The first camera always has Tx = Ty = 0. For
    // #  the right (second) camera of a horizontal stereo pair, Ty = 0 and
    // #  Tx = -fx' * B, where B is the baseline between the cameras.
    // # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
    // #  the rectified image is given by:
    // #  [u v w]' = P * [X Y Z 1]'
    // #         x = u / w
    // #         y = v / w
    // #  This holds for both images of a stereo pair.
    // float64[12] p # 3x4 row-major matrix
    //
    //
    // #######################################################################
    // #                      Operational Parameters                         #
    // #######################################################################
    // # These define the image region actually captured by the camera       #
    // # driver. Although they affect the geometry of the output image, they #
    // # may be changed freely without recalibrating the camera.             #
    // #######################################################################
    //
    // # Binning refers here to any camera setting which combines rectangular
    // #  neighborhoods of pixels into larger "super-pixels." It reduces the
    // #  resolution of the output image to
    // #  (width / binning_x) x (height / binning_y).
    // # The default values binning_x = binning_y = 0 is considered the same
    // #  as binning_x = binning_y = 1 (no subsampling).
    // uint32 binning_x
    // uint32 binning_y
    //
    // # Region of interest (subwindow of full camera resolution), given in
    // #  full resolution (unbinned) image coordinates. A particular ROI
    // #  always denotes the same window of pixels on the camera sensor,
    // #  regardless of binning settings.
    // # The default setting of roi (all values 0) is considered the same as
    // #  full resolution (roi.width = width, roi.height = height).
    // RegionOfInterest roi
    struct CameraInfo {
        std_msgs::msg::Header header;

        uint32_t height;
        uint32_t width;
        MsgPack::arr_t<double> d;  // The distortion parameters, size depending on the distortion model
        std::array<double, 9> k;   // 3x3 row-major matrix
        std::array<double, 9> r;   // 3x3 row - major matrix
        std::array<double, 12> p;  // 3x4 row - major matrix

        uint32_t binning_x;
        uint32_t binning_y;
        RegionOfInterest roi;

        MSGPACK_DEFINE(header, height, width, d, k, r, p, binning_x, binning_y, roi);

#ifdef ROS
        CameraInfo& operator=(const ::sensor_msgs::msg::CameraInfo& rhs) {
            header = rhs.header;
            height = rhs.height;
            width = rhs.width;
            d = rhs.d;
            k = rhs.k;
            r = rhs.r;
            p = rhs.p;
            binning_x = rhs.binning_x;
            binning_y = rhs.binning_y;
            roi = rhs.roi;
            return *this;
        }
        CameraInfo& operator=(::sensor_msgs::msg::CameraInfo&& rhs) {
            header = std::move(rhs.header);
            height = std::move(rhs.height);
            width = std::move(rhs.width);
            d = std::move(rhs.d);
            k = std::move(rhs.k);
            r = std::move(rhs.r);
            p = std::move(rhs.p);
            binning_x = std::move(rhs.binning_x);
            binning_y = std::move(rhs.binning_y);
            roi = std::move(rhs.roi);
            return *this;
        }
#endif
    };

    // # This message is used by the PointCloud message to hold optional data
    // # associated with each point in the cloud. The length of the values
    // # array should be the same as the length of the points array in the
    // # PointCloud, and each value should be associated with the corresponding
    // # point.
    // #
    // # Channel names in existing practice include:
    // #   "u", "v" - row and column (respectively) in the left stereo image.
    // #              This is opposite to usual conventions but remains for
    // #              historical reasons. The newer PointCloud2 message has no
    // #              such problem.
    // #   "rgb" - For point clouds produced by color stereo cameras. uint8
    // #           (R,G,B) values packed into the least significant 24 bits,
    // #           in order.
    // #   "intensity" - laser or pixel intensity.
    // #   "distance"
    //
    // # The channel name should give semantics of the channel (e.g.
    // # "intensity" instead of "value").
    // string name
    //
    // # The values array should be 1-1 with the elements of the associated
    // # PointCloud.
    // float32[] values
    struct ChannelFloat32 {
        MsgPack::str_t name;
        MsgPack::arr_t<float> values;
        MSGPACK_DEFINE(name, values);

#ifdef ROS
        ChannelFloat32& operator=(const ::sensor_msgs::msg::ChannelFloat32& rhs) {
            name = rhs.name;
            values = rhs.values;
            return *this;
        }
        ChannelFloat32& operator=(::sensor_msgs::msg::ChannelFloat32&& rhs) {
            name = std::move(rhs.name);
            values = std::move(rhs.values);
            return *this;
        }
#endif
    };

    // # This message contains a compressed image.
    //
    // std_msgs/Header header # Header timestamp should be acquisition time of image
    //                              # Header frame_id should be optical frame of camera
    //                              # origin of frame should be optical center of cameara
    //                              # +x should point to the right in the image
    //                              # +y should point down in the image
    //                              # +z should point into to plane of the image
    //
    // string format                # Specifies the format of the data
    //                              #   Acceptable values:
    //                              #     jpeg, png
    //
    // uint8[] data                 # Compressed image buffer
    struct CompressedImage {
        std_msgs::msg::Header header;
        MsgPack::str_t format;         // Acceptable values: jpeg, png
        MsgPack::arr_t<uint8_t> data;  // Compressed image buffer
        MSGPACK_DEFINE(header, format, data);

#ifdef ROS
        CompressedImage& operator=(const ::sensor_msgs::msg::CompressedImage& rhs) {
            header = rhs.header;
            format = rhs.format;
            data = rhs.data;
            return *this;
        }
        CompressedImage& operator=(::sensor_msgs::msg::CompressedImage&& rhs) {
            header = std::move(rhs.header);
            format = std::move(rhs.format);
            data = std::move(rhs.data);
            return *this;
        }
#endif
    };

    // # Single pressure reading.  This message is appropriate for measuring the
    // # pressure inside of a fluid (air, water, etc).  This also includes
    // # atmospheric or barometric pressure.
    // #
    // # This message is not appropriate for force/pressure contact sensors.
    //
    // std_msgs/Header header # timestamp of the measurement
    //                              # frame_id is the location of the pressure sensor
    //
    // float64 fluid_pressure       # Absolute pressure reading in Pascals.
    //
    // float64 variance             # 0 is interpreted as variance unknown
    struct FluidPressure {
        std_msgs::msg::Header header;  // frame_id is the location of the pressure sensor
        double fluid_pressure;         // Absolute pressure reading in Pascals.
        double variance;               // 0 is interpreted as variance unknown
        MSGPACK_DEFINE(header, fluid_pressure, variance);

#ifdef ROS
        FluidPressure& operator=(const ::sensor_msgs::msg::FluidPressure& rhs) {
            header = rhs.header;
            fluid_pressure = rhs.fluid_pressure;
            variance = rhs.variance;
            return *this;
        }
        FluidPressure& operator=(::sensor_msgs::msg::FluidPressure&& rhs) {
            header = std::move(rhs.header);
            fluid_pressure = std::move(rhs.fluid_pressure);
            variance = std::move(rhs.variance);
            return *this;
        }
#endif
    };

    // # Single photometric illuminance measurement.  Light should be assumed to be
    // # measured along the sensor's x-axis (the area of detection is the y-z plane).
    // # The illuminance should have a 0 or positive value and be received with
    // # the sensor's +X axis pointing toward the light source.
    // #
    // # Photometric illuminance is the measure of the human eye's sensitivity of the
    // # intensity of light encountering or passing through a surface.
    // #
    // # All other Photometric and Radiometric measurements should not use this message.
    // # This message cannot represent:
    // #  - Luminous intensity (candela/light source output)
    // #  - Luminance (nits/light output per area)
    // #  - Irradiance (watt/area), etc.
    //
    // std_msgs/Header header # timestamp is the time the illuminance was measured
    //                              # frame_id is the location and direction of the reading
    //
    // float64 illuminance          # Measurement of the Photometric Illuminance in Lux.
    //
    // float64 variance             # 0 is interpreted as variance unknown
    struct Illuminance {
        std_msgs::msg::Header header;  // frame_id is the location and direction of the reading
        double illuminance;            // Measurement of the Photometric Illuminance in Lux.
        double variance;               // 0 is interpreted as variance unknown
        MSGPACK_DEFINE(header, illuminance, variance);

#ifdef ROS
        Illuminance& operator=(const ::sensor_msgs::msg::Illuminance& rhs) {
            header = rhs.header;
            illuminance = rhs.illuminance;
            variance = rhs.variance;
            return *this;
        }
        Illuminance& operator=(::sensor_msgs::msg::Illuminance&& rhs) {
            header = std::move(rhs.header);
            illuminance = std::move(rhs.illuminance);
            variance = std::move(rhs.variance);
            return *this;
        }
#endif
    };

    // # This message contains an uncompressed image
    // # (0, 0) is at top-left corner of image
    //
    // std_msgs/Header header # Header timestamp should be acquisition time of image
    //                              # Header frame_id should be optical frame of camera
    //                              # origin of frame should be optical center of cameara
    //                              # +x should point to the right in the image
    //                              # +y should point down in the image
    //                              # +z should point into to plane of the image
    //                              # If the frame_id here and the frame_id of the CameraInfo
    //                              # message associated with the image conflict
    //                              # the behavior is undefined
    //
    // uint32 height                # image height, that is, number of rows
    // uint32 width                 # image width, that is, number of columns
    //
    // # The legal values for encoding are in file src/image_encodings.cpp
    // # If you want to standardize a new string format, join
    // # ros-users@lists.ros.org and send an email proposing a new encoding.
    //
    // string encoding       # Encoding of pixels -- channel meaning, ordering, size
    //                       # taken from the list of strings in include/sensor_msgs/image_encodings.hpp
    //
    // uint8 is_bigendian    # is this data bigendian?
    // uint32 step           # Full row length in bytes
    // uint8[] data          # actual matrix data, size is (step * rows)
    struct Image {
        std_msgs::msg::Header header;
        uint32_t height;
        uint32_t width;
        MsgPack::str_t encoding;  // taken from the list of strings in include/sensor_msgs/image_encodings.hpp
        uint8_t is_bigendian;
        uint32_t step;                 // Full row length in bytes
        MsgPack::arr_t<uint8_t> data;  // actual matrix data, size is (step * rows)
        MSGPACK_DEFINE(header, height, width, encoding, is_bigendian, step, data);

#ifdef ROS
        Image& operator=(const ::sensor_msgs::msg::Image& rhs) {
            header = rhs.header;
            height = rhs.height;
            width = rhs.width;
            encoding = rhs.encoding;
            is_bigendian = rhs.is_bigendian;
            step = rhs.step;
            data = rhs.data;
            return *this;
        }
        Image& operator=(::sensor_msgs::msg::Image&& rhs) {
            header = std::move(rhs.header);
            height = std::move(rhs.height);
            width = std::move(rhs.width);
            encoding = std::move(rhs.encoding);
            is_bigendian = std::move(rhs.is_bigendian);
            step = std::move(rhs.step);
            data = std::move(rhs.data);
            return *this;
        }
#endif
    };

    // # This is a message to hold data from an IMU (Inertial Measurement Unit)
    // #
    // # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    // #
    // # If the covariance of the measurement is known, it should be filled in (if all you know is the
    // # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    // # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    // # data a covariance will have to be assumed or gotten from some other source
    // #
    // # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
    // # orientation estimate), please set element 0 of the associated covariance matrix to -1
    // # If you are interpreting this message, please check for a value of -1 in the first element of each
    // # covariance matrix, and disregard the associated estimate.
    //
    // std_msgs/Header header
    //
    // geometry_msgs/Quaternion orientation
    // float64[9] orientation_covariance # Row major about x, y, z axes
    //
    // geometry_msgs/Vector3 angular_velocity
    // float64[9] angular_velocity_covariance # Row major about x, y, z axes
    //
    // geometry_msgs/Vector3 linear_acceleration
    // float64[9] linear_acceleration_covariance # Row major x, y z
    struct Imu {
        std_msgs::msg::Header header;
        geometry_msgs::msg::Quaternion orientation;
        std::array<double, 9> orientation_covariance;
        geometry_msgs::msg::Vector3 angular_velocity;
        std::array<double, 9> angular_velocity_covariance;
        geometry_msgs::msg::Vector3 linear_acceleration;
        std::array<double, 9> linear_acceleration_covariance;

        MSGPACK_DEFINE(
            header,
            orientation,
            orientation_covariance,
            angular_velocity,
            angular_velocity_covariance,
            linear_acceleration,
            linear_acceleration_covariance);

#ifdef ROS
        Imu& operator=(const ::geometry_msgs::msg::Imu& rhs) {
            header = rhs.header;
            orientation = rhs.orientation;
            orientation_covariance = rhs.orientation_covariance;
            angular_velocity = rhs.angular_velocity;
            angular_velocity_covariance = rhs.angular_velocity_covariance;
            linear_acceleration = rhs.linear_acceleration;
            linear_acceleration_covariance = rhs.linear_acceleration_covariance;
            return *this;
        }
        Imu& operator=(::geometry_msgs::msg::Imu&& rhs) {
            header = std::move(rhs.header);
            orientation = std::move(rhs.orientation);
            orientation_covariance = std::move(rhs.orientation_covariance);
            angular_velocity = std::move(rhs.angular_velocity);
            angular_velocity_covariance = std::move(rhs.angular_velocity_covariance);
            linear_acceleration = std::move(rhs.linear_acceleration);
            linear_acceleration_covariance = std::move(rhs.linear_acceleration_covariance);
            return *this;
        }
#endif
    };

    // # This is a message that holds data to describe the state of a set of torque controlled joints.
    // #
    // # The state of each joint (revolute or prismatic) is defined by:
    // #  * the position of the joint (rad or m),
    // #  * the velocity of the joint (rad/s or m/s) and
    // #  * the effort that is applied in the joint (Nm or N).
    // #
    // # Each joint is uniquely identified by its name
    // # The header specifies the time at which the joint states were recorded. All the joint states
    // # in one message have to be recorded at the same time.
    // #
    // # This message consists of a multiple arrays, one for each part of the joint state.
    // # The goal is to make each of the fields optional. When e.g. your joints have no
    // # effort associated with them, you can leave the effort array empty.
    // #
    // # All arrays in this message should have the same size, or be empty.
    // # This is the only way to uniquely associate the joint name with the correct
    // # states.
    //
    // std_msgs/Header header
    //
    // string[] name
    // float64[] position
    // float64[] velocity
    // float64[] effort
    struct JointState {
        std_msgs::msg::Header header;
        MsgPack::arr_t<MsgPack::str_t> name;
        MsgPack::arr_t<double> position;
        MsgPack::arr_t<double> velocity;
        MsgPack::arr_t<double> effort;
        MSGPACK_DEFINE(header, name, position, velocity, effort);

#ifdef ROS
        JointState& operator=(const ::sensor_msgs::msg::JointState& rhs) {
            header = rhs.header;
            name = rhs.name;
            position = rhs.position;
            velocity = rhs.velocity;
            effort = rhs.effort;
            return *this;
        }
        JointState& operator=(::sensor_msgs::msg::JointState&& rhs) {
            header = std::move(rhs.header);
            name = std::move(rhs.name);
            position = std::move(rhs.position);
            velocity = std::move(rhs.velocity);
            effort = std::move(rhs.effort);
            return *this;
        }
#endif
    };

    // # Reports the state of a joystick's axes and buttons.
    //
    // # The timestamp is the time at which data is received from the joystick.
    // std_msgs/Header header
    //
    // # The axes measurements from a joystick.
    // float32[] axes
    //
    // # The buttons measurements from a joystick.
    // int32[] buttons
    struct Joy {
        std_msgs::msg::Header header;
        MsgPack::arr_t<float> axes;
        MsgPack::arr_t<int32_t> buttons;
        MSGPACK_DEFINE(header, axes, buttons);

#ifdef ROS
        Joy& operator=(const ::sensor_msgs::msg::Joy& rhs) {
            header = rhs.header;
            axes = rhs.axes;
            buttons = rhs.buttons;
            return *this;
        }
        Joy& operator=(::sensor_msgs::msg::Joy&& rhs) {
            header = std::move(rhs.header);
            axes = std::move(rhs.axes);
            buttons = std::move(rhs.buttons);
            return *this;
        }
#endif
    };

    // # Declare of the type of feedback
    // uint8 TYPE_LED    = 0
    // uint8 TYPE_RUMBLE = 1
    // uint8 TYPE_BUZZER = 2
    //
    // uint8 type
    //
    // # This will hold an id number for each type of each feedback.
    // # Example, the first led would be id=0, the second would be id=1
    // uint8 id
    //
    // # Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
    // # actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
    // float32 intensity
    struct JoyFeedback {
        static constexpr uint8_t TYPE_LED = 0;
        static constexpr uint8_t TYPE_RUMBLE = 1;
        static constexpr uint8_t TYPE_BUZZER = 2;

        uint8_t type;
        uint8_t id;
        float intensity;

        MSGPACK_DEFINE(type, id, intensity);

#ifdef ROS
        JoyFeedback& operator=(const ::sensor_msgs::msg::JoyFeedback& rhs) {
            type = rhs.type;
            id = rhs.id;
            intensity = rhs.intensity;
            return *this;
        }
        JoyFeedback& operator=(::sensor_msgs::msg::JoyFeedback&& rhs) {
            type = std::move(rhs.type);
            id = std::move(rhs.id);
            intensity = std::move(rhs.intensity);
            return *this;
        }
#endif
    };

    // # This message publishes values for multiple feedback at once.
    // JoyFeedback[] array
    struct JoyFeedbackArray {
        MsgPack::arr_t<JoyFeedback> array;
        MSGPACK_DEFINE(array);

#ifdef ROS
        JoyFeedbackArray& operator=(const ::sensor_msgs::msg::JoyFeedbackArray& rhs) {
            array = rhs.array;
            return *this;
        }
        JoyFeedbackArray& operator=(::sensor_msgs::msg::JoyFeedbackArray&& rhs) {
            array = std::move(rhs.array);
            return *this;
        }
#endif
    };

    // # This message is a submessage of MultiEchoLaserScan and is not intended
    // # to be used separately.
    //
    // float32[] echoes  # Multiple values of ranges or intensities.
    //                   # Each array represents data from the same angle increment.
    struct LaserEcho {
        MsgPack::arr_t<float> echoes;
        MSGPACK_DEFINE(echoes);

#ifdef ROS
        LaserEcho& operator=(const ::sensor_msgs::msg::LaserEcho& rhs) {
            echoes = rhs.echoes;
            return *this;
        }
        LaserEcho& operator=(::sensor_msgs::msg::LaserEcho&& rhs) {
            echoes = std::move(rhs.echoes);
            return *this;
        }
#endif
    };

    // # Single scan from a planar laser range-finder
    // #
    // # If you have another ranging device with different behavior (e.g. a sonar
    // # array), please find or create a different message, since applications
    // # will make fairly laser-specific assumptions about this data
    //
    // std_msgs/Header header # timestamp in the header is the acquisition time of
    //                              # the first ray in the scan.
    //                              #
    //                              # in frame frame_id, angles are measured around
    //                              # the positive Z axis (counterclockwise, if Z is up)
    //                              # with zero angle being forward along the x axis
    //
    // float32 angle_min            # start angle of the scan [rad]
    // float32 angle_max            # end angle of the scan [rad]
    // float32 angle_increment      # angular distance between measurements [rad]
    //
    // float32 time_increment       # time between measurements [seconds] - if your scanner
    //                              # is moving, this will be used in interpolating position
    //                              # of 3d points
    // float32 scan_time            # time between scans [seconds]
    //
    // float32 range_min            # minimum range value [m]
    // float32 range_max            # maximum range value [m]
    //
    // float32[] ranges             # range data [m]
    //                              # (Note: values < range_min or > range_max should be discarded)
    // float32[] intensities        # intensity data [device-specific units].  If your
    //                              # device does not provide intensities, please leave
    //                              # the array empty.
    struct LaserScan {
        std_msgs::msg::Header header;
        float angle_min;
        float angle_max;
        float angle_increment;
        float time_increment;
        float scan_time;
        float range_min;
        float range_max;
        MsgPack::arr_t<float> ranges;
        MsgPack::arr_t<float> intensities;

        MSGPACK_DEFINE(
            header,
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges,
            intensities);

#ifdef ROS
        LaserScan& operator=(const ::sensor_msgs::msg::LaserScan& rhs) {
            header = rhs.header;
            angle_min = rhs.angle_min;
            angle_max = rhs.angle_max;
            angle_increment = rhs.angle_increment;
            time_increment = rhs.time_increment;
            scan_time = rhs.scan_time;
            range_min = rhs.range_min;
            range_max = rhs.range_max;
            ranges = rhs.ranges;
            intensities = rhs.intensities;
            return *this;
        }
        LaserScan& operator=(::sensor_msgs::msg::LaserScan&& rhs) {
            header = std::move(rhs.header);
            angle_min = std::move(rhs.angle_min);
            angle_max = std::move(rhs.angle_max);
            angle_increment = std::move(rhs.angle_increment);
            time_increment = std::move(rhs.time_increment);
            scan_time = std::move(rhs.scan_time);
            range_min = std::move(rhs.range_min);
            range_max = std::move(rhs.range_max);
            ranges = std::move(rhs.ranges);
            intensities = std::move(rhs.intensities);
            return *this;
        }
#endif
    };

    // # Measurement of the Magnetic Field vector at a specific location.
    // #
    // # If the covariance of the measurement is known, it should be filled in.
    // # If all you know is the variance of each measurement, e.g. from the datasheet,
    // # just put those along the diagonal.
    // # A covariance matrix of all zeros will be interpreted as "covariance unknown",
    // # and to use the data a covariance will have to be assumed or gotten from some
    // # other source.
    //
    // std_msgs/Header header               # timestamp is the time the
    //                                            # field was measured
    //                                            # frame_id is the location and orientation
    //                                            # of the field measurement
    //
    // geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
    //                                            # field vector in Tesla
    //                                            # If your sensor does not output 3 axes,
    //                                            # put NaNs in the components not reported.
    //
    // float64[9] magnetic_field_covariance       # Row major about x, y, z axes
    //                                            # 0 is interpreted as variance unknown
    struct MagneticField {
        std_msgs::msg::Header header;
        geometry_msgs::msg::Vector3 magnetic_field;
        std::array<double, 9> magnetic_field_covariance;
        MSGPACK_DEFINE(header, magnetic_field, magnetic_field_covariance);

#ifdef ROS
        MagneticField& operator=(const ::sensor_msgs::msg::MagneticField& rhs) {
            header = rhs.header;
            magnetic_field = rhs.magnetic_field;
            magnetic_field_covariance = rhs.magnetic_field_covariance;
            return *this;
        }
        MagneticField& operator=(::sensor_msgs::msg::MagneticField&& rhs) {
            header = std::move(rhs.header);
            magnetic_field = std::move(rhs.magnetic_field);
            magnetic_field_covariance = std::move(rhs.magnetic_field_covariance);
            return *this;
        }
#endif
    };

    // # Representation of state for joints with multiple degrees of freedom,
    // # following the structure of JointState which can only represent a single degree of freedom.
    // #
    // # It is assumed that a joint in a system corresponds to a transform that gets applied
    // # along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
    // # and those 3DOF can be expressed as a transformation matrix, and that transformation
    // # matrix can be converted back to (x, y, yaw)
    // #
    // # Each joint is uniquely identified by its name
    // # The header specifies the time at which the joint states were recorded. All the joint states
    // # in one message have to be recorded at the same time.
    // #
    // # This message consists of a multiple arrays, one for each part of the joint state.
    // # The goal is to make each of the fields optional. When e.g. your joints have no
    // # wrench associated with them, you can leave the wrench array empty.
    // #
    // # All arrays in this message should have the same size, or be empty.
    // # This is the only way to uniquely associate the joint name with the correct
    // # states.
    //
    // std_msgs/Header header
    //
    // string[] joint_names
    // geometry_msgs/Transform[] transforms
    // geometry_msgs/Twist[] twist
    // geometry_msgs/Wrench[] wrench
    struct MultiDOFJointState {
        std_msgs::msg::Header header;
        MsgPack::arr_t<MsgPack::str_t> joint_names;
        MsgPack::arr_t<geometry_msgs::msg::Transform> transforms;
        MsgPack::arr_t<geometry_msgs::msg::Twist> twist;
        MsgPack::arr_t<geometry_msgs::msg::Wrench> wrench;
        MSGPACK_DEFINE(header, joint_names, transforms, twist, wrench);

#ifdef ROS
        MultiDOFJointState& operator=(const ::sensor_msgs::msg::MultiDOFJointState& rhs) {
            header = rhs.header;
            joint_names = rhs.joint_names;
            transforms = rhs.transforms;
            twist = rhs.twist;
            wrench = rhs.wrench;
            return *this;
        }
        MultiDOFJointState& operator=(::sensor_msgs::msg::MultiDOFJointState&& rhs) {
            header = std::move(rhs.header);
            joint_names = std::move(rhs.joint_names);
            transforms = std::move(rhs.transforms);
            twist = std::move(rhs.twist);
            wrench = std::move(rhs.wrench);
            return *this;
        }
#endif
    };

    // # Single scan from a multi-echo planar laser range-finder
    // #
    // # If you have another ranging device with different behavior (e.g. a sonar
    // # array), please find or create a different message, since applications
    // # will make fairly laser-specific assumptions about this data
    //
    // std_msgs/Header header # timestamp in the header is the acquisition time of
    //                              # the first ray in the scan.
    //                              #
    //                              # in frame frame_id, angles are measured around
    //                              # the positive Z axis (counterclockwise, if Z is up)
    //                              # with zero angle being forward along the x axis
    //
    // float32 angle_min            # start angle of the scan [rad]
    // float32 angle_max            # end angle of the scan [rad]
    // float32 angle_increment      # angular distance between measurements [rad]
    //
    // float32 time_increment       # time between measurements [seconds] - if your scanner
    //                              # is moving, this will be used in interpolating position
    //                              # of 3d points
    // float32 scan_time            # time between scans [seconds]
    //
    // float32 range_min            # minimum range value [m]
    // float32 range_max            # maximum range value [m]
    //
    // LaserEcho[] ranges           # range data [m]
    //                              # (Note: NaNs, values < range_min or > range_max should be discarded)
    //                              # +Inf measurements are out of range
    //                              # -Inf measurements are too close to determine exact distance.
    // LaserEcho[] intensities      # intensity data [device-specific units].  If your
    //                              # device does not provide intensities, please leave
    //                              # the array empty.
    struct MultiEchoLaserScan {
        std_msgs::msg::Header header;
        float angle_min;
        float angle_max;
        float angle_increment;
        float time_increment;
        float scan_time;
        float range_min;
        float range_max;
        MsgPack::arr_t<LaserEcho> ranges;
        MsgPack::arr_t<LaserEcho> intensities;

        MSGPACK_DEFINE(
            header,
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges,
            intensities);

#ifdef ROS
        MultiEchoLaserScan& operator=(const ::sensor_msgs::msg::MultiEchoLaserScan& rhs) {
            header = rhs.header;
            angle_min = rhs.angle_min;
            angle_max = rhs.angle_max;
            angle_increment = rhs.angle_increment;
            time_increment = rhs.time_increment;
            scan_time = rhs.scan_time;
            range_min = rhs.range_min;
            range_max = rhs.range_max;
            ranges = rhs.ranges;
            intensities = rhs.intensities;
            return *this;
        }
        MultiEchoLaserScan& operator=(::sensor_msgs::msg::MultiEchoLaserScan&& rhs) {
            header = std::move(rhs.header);
            angle_min = std::move(rhs.angle_min);
            angle_max = std::move(rhs.angle_max);
            angle_increment = std::move(rhs.angle_increment);
            time_increment = std::move(rhs.time_increment);
            scan_time = std::move(rhs.scan_time);
            range_min = std::move(rhs.range_min);
            range_max = std::move(rhs.range_max);
            ranges = std::move(rhs.ranges);
            intensities = std::move(rhs.intensities);
            return *this;
        }
#endif
    };

    // # Navigation Satellite fix status for any Global Navigation Satellite System.
    // #
    // # Whether to output an augmented fix is determined by both the fix
    // # type and the last time differential corrections were received.  A
    // # fix is valid when status >= STATUS_FIX.
    //
    // int8 STATUS_NO_FIX =  -1        # unable to fix position
    // int8 STATUS_FIX =      0        # unaugmented fix
    // int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
    // int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation
    //
    // int8 status
    //
    // # Bits defining which Global Navigation Satellite System signals were
    // # used by the receiver.
    //
    // uint16 SERVICE_GPS =     1
    // uint16 SERVICE_GLONASS = 2
    // uint16 SERVICE_COMPASS = 4      # includes BeiDou.
    // uint16 SERVICE_GALILEO = 8
    //
    // uint16 service
    struct NavSatStatus {
        static constexpr int8_t STATUS_NO_FIX = -1;
        static constexpr int8_t STATUS_FIX = 0;
        static constexpr int8_t STATUS_SBAS_FIX = 1;
        static constexpr int8_t STATUS_GBAS_FIX = 2;

        static constexpr uint16_t SERVICE_GPS = 1;
        static constexpr uint16_t SERVICE_GLONASS = 2;
        static constexpr uint16_t SERVICE_COMPASS = 4;
        static constexpr uint16_t SERVICE_GALILEO = 8;

        int8_t status;
        uint16_t service;

        MSGPACK_DEFINE(status, service);

#ifdef ROS
        NavSatStatus& operator=(const ::sensor_msgs::msg::NavSatStatus& rhs) {
            status = rhs.status;
            service = rhs.service;
            return *this;
        }
        NavSatStatus& operator=(::sensor_msgs::msg::NavSatStatus&& rhs) {
            status = std::move(rhs.status);
            service = std::move(rhs.service);
            return *this;
        }
#endif
    };

    // # Navigation Satellite fix for any Global Navigation Satellite System
    // #
    // # Specified using the WGS 84 reference ellipsoid
    //
    // # header.stamp specifies the ROS time for this measurement (the
    // #        corresponding satellite time may be reported using the
    // #        sensor_msgs/TimeReference message).
    // #
    // # header.frame_id is the frame of reference reported by the satellite
    // #        receiver, usually the location of the antenna.  This is a
    // #        Euclidean frame relative to the vehicle, not a reference
    // #        ellipsoid.
    // std_msgs/Header header
    //
    // # Satellite fix status information.
    // NavSatStatus status
    //
    // # Latitude [degrees]. Positive is north of equator; negative is south.
    // float64 latitude
    //
    // # Longitude [degrees]. Positive is east of prime meridian; negative is west.
    // float64 longitude
    //
    // # Altitude [m]. Positive is above the WGS 84 ellipsoid
    // # (quiet NaN if no altitude is available).
    // float64 altitude
    //
    // # Position covariance [m^2] defined relative to a tangential plane
    // # through the reported position. The components are East, North, and
    // # Up (ENU), in row-major order.
    // #
    // # Beware: this coordinate system exhibits singularities at the poles.
    // float64[9] position_covariance
    //
    // # If the covariance of the fix is known, fill it in completely. If the
    // # GPS receiver provides the variance of each measurement, put them
    // # along the diagonal. If only Dilution of Precision is available,
    // # estimate an approximate covariance from that.
    //
    // uint8 COVARIANCE_TYPE_UNKNOWN = 0
    // uint8 COVARIANCE_TYPE_APPROXIMATED = 1
    // uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
    // uint8 COVARIANCE_TYPE_KNOWN = 3
    //
    // uint8 position_covariance_type
    struct NavSatFix {
        static constexpr uint8_t COVARIANCE_TYPE_UNKNOWN = 0;
        static constexpr uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;
        static constexpr uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
        static constexpr uint8_t COVARIANCE_TYPE_KNOWN = 3;

        std_msgs::msg::Header header;
        NavSatStatus status;
        double latitude;
        double longitude;
        double altitude;
        std::array<double, 9> position_covariance;
        uint8_t position_covariance_type;

        MSGPACK_DEFINE(header, status, latitude, longitude, altitude, position_covariance, position_covariance_type);

#ifdef ROS
        NavSatFix& operator=(const ::sensor_msgs::msg::NavSatFix& rhs) {
            header = rhs.header;
            status = rhs.status;
            latitude = rhs.latitude;
            longitude = rhs.longitude;
            altitude = rhs.altitude;
            position_covariance = rhs.position_covariance;
            position_covariance_type = rhs.position_covariance_type;
            return *this;
        }
        NavSatFix& operator=(::sensor_msgs::msg::NavSatFix&& rhs) {
            header = std::move(rhs.header);
            status = std::move(rhs.status);
            latitude = std::move(rhs.latitude);
            longitude = std::move(rhs.longitude);
            altitude = std::move(rhs.altitude);
            position_covariance = std::move(rhs.position_covariance);
            position_covariance_type = std::move(rhs.position_covariance_type);
            return *this;
        }
#endif
    };

    // # This message holds the description of one point entry in the
    // # PointCloud2 message format.
    // uint8 INT8    = 1
    // uint8 UINT8   = 2
    // uint8 INT16   = 3
    // uint8 UINT16  = 4
    // uint8 INT32   = 5
    // uint8 UINT32  = 6
    // uint8 FLOAT32 = 7
    // uint8 FLOAT64 = 8
    //
    // # Common PointField names are x, y, z, intensity, rgb, rgba
    // string name      # Name of field
    // uint32 offset    # Offset from start of point struct
    // uint8  datatype  # Datatype enumeration, see above
    // uint32 count     # How many elements in the field
    struct PointField {
        static constexpr uint8_t INT8 = 1;
        static constexpr uint8_t UINT8 = 2;
        static constexpr uint8_t INT16 = 3;
        static constexpr uint8_t UINT16 = 4;
        static constexpr uint8_t INT32 = 5;
        static constexpr uint8_t UINT32 = 6;
        static constexpr uint8_t FLOAT32 = 7;
        static constexpr uint8_t FLOAT64 = 8;

        MsgPack::str_t name;
        uint32_t offset;
        uint8_t datatype;
        uint32_t count;

        MSGPACK_DEFINE(name, offset, datatype, count);

#ifdef ROS
        PointField& operator=(const ::sensor_msgs::msg::PointField& rhs) {
            name = rhs.name;
            offset = rhs.offset;
            datatype = rhs.datatype;
            count = rhs.count;
            return *this;
        }
        PointField& operator=(::sensor_msgs::msg::PointField&& rhs) {
            name = std::move(rhs.name);
            offset = std::move(rhs.offset);
            datatype = std::move(rhs.datatype);
            count = std::move(rhs.count);
            return *this;
        }
#endif
    };

    // ## THIS MESSAGE IS DEPRECATED AS OF FOXY
    // ## Please use sensor_msgs/PointCloud2
    //
    // # This message holds a collection of 3d points, plus optional additional
    // # information about each point.
    //
    // # Time of sensor data acquisition, coordinate frame ID.
    // std_msgs/Header header
    //
    // # Array of 3d points. Each Point32 should be interpreted as a 3d point
    // # in the frame given in the header.
    // geometry_msgs/Point32[] points
    //
    // # Each channel should have the same number of elements as points array,
    // # and the data in each channel should correspond 1:1 with each point.
    // # Channel names in common practice are listed in ChannelFloat32.msg.
    // ChannelFloat32[] channels
    struct PointCloud {
        std_msgs::msg::Header header;
        MsgPack::arr_t<geometry_msgs::msg::Point32> points;
        MsgPack::arr_t<ChannelFloat32> channels;
        MSGPACK_DEFINE(header, points, channels);

#ifdef ROS
        PointCloud& operator=(const ::sensor_msgs::msg::PointCloud& rhs) {
            header = rhs.header;
            points = rhs.points;
            channels = rhs.channels;
            return *this;
        }
        PointCloud& operator=(::sensor_msgs::msg::PointCloud&& rhs) {
            header = std::move(rhs.header);
            points = std::move(rhs.points);
            channels = std::move(rhs.channels);
            return *this;
        }
#endif
    };

    // # This message holds a collection of N-dimensional points, which may
    // # contain additional information such as normals, intensity, etc. The
    // # point data is stored as a binary blob, its layout described by the
    // # contents of the "fields" array.
    // #
    // # The point cloud data may be organized 2d (image-like) or 1d (unordered).
    // # Point clouds organized as 2d images may be produced by camera depth sensors
    // # such as stereo or time-of-flight.
    //
    // # Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
    // std_msgs/Header header
    //
    // # 2D structure of the point cloud. If the cloud is unordered, height is
    // # 1 and width is the length of the point cloud.
    // uint32 height
    // uint32 width
    //
    // # Describes the channels and their layout in the binary data blob.
    // PointField[] fields
    //
    // bool    is_bigendian # Is this data bigendian?
    // uint32  point_step   # Length of a point in bytes
    // uint32  row_step     # Length of a row in bytes
    // uint8[] data         # Actual point data, size is (row_step*height)
    //
    // bool is_dense        # True if there are no invalid points
    struct PointCloud2 {
        std_msgs::msg::Header header;
        uint32_t height;
        uint32_t width;
        MsgPack::arr_t<PointField> fields;
        bool is_bigendian;
        uint32_t point_step;
        uint32_t row_step;
        MsgPack::arr_t<uint8_t> data;
        bool is_dense;

        MSGPACK_DEFINE(header, height, width, fields, is_bigendian, point_step, row_step, data, is_dense);

#ifdef ROS
        PointCloud2& operator=(const ::sensor_msgs::msg::PointCloud2& rhs) {
            header = rhs.header;
            height = rhs.height;
            width = rhs.width;
            fields = rhs.fields;
            is_bigendian = rhs.is_bigendian;
            point_step = rhs.point_step;
            row_step = rhs.row_step;
            data = rhs.data;
            is_dense = rhs.is_dense;
            return *this;
        }
        PointCloud2& operator=(::sensor_msgs::msg::PointCloud2&& rhs) {
            header = std::move(rhs.header);
            height = std::move(rhs.height);
            width = std::move(rhs.width);
            fields = std::move(rhs.fields);
            is_bigendian = std::move(rhs.is_bigendian);
            point_step = std::move(rhs.point_step);
            row_step = std::move(rhs.row_step);
            data = std::move(rhs.data);
            is_dense = std::move(rhs.is_dense);
            return *this;
        }
#endif
    };

    // # Single range reading from an active ranger that emits energy and reports
    // # one range reading that is valid along an arc at the distance measured.
    // # This message is  not appropriate for laser scanners. See the LaserScan
    // # message if you are working with a laser scanner.
    // #
    // # This message also can represent a fixed-distance (binary) ranger.  This
    // # sensor will have min_range===max_range===distance of detection.
    // # These sensors follow REP 117 and will output -Inf if the object is detected
    // # and +Inf if the object is outside of the detection range.
    //
    // std_msgs/Header header # timestamp in the header is the time the ranger
    //                              # returned the distance reading
    //
    // # Radiation type enums
    // # If you want a value added to this list, send an email to the ros-users list
    // uint8 ULTRASOUND=0
    // uint8 INFRARED=1
    //
    // uint8 radiation_type    # the type of radiation used by the sensor
    //                         # (sound, IR, etc) [enum]
    //
    // float32 field_of_view   # the size of the arc that the distance reading is
    //                         # valid for [rad]
    //                         # the object causing the range reading may have
    //                         # been anywhere within -field_of_view/2 and
    //                         # field_of_view/2 at the measured range.
    //                         # 0 angle corresponds to the x-axis of the sensor.
    //
    // float32 min_range       # minimum range value [m]
    // float32 max_range       # maximum range value [m]
    //                         # Fixed distance rangers require min_range==max_range
    //
    // float32 range           # range data [m]
    //                         # (Note: values < range_min or > range_max should be discarded)
    //                         # Fixed distance rangers only output -Inf or +Inf.
    //                         # -Inf represents a detection within fixed distance.
    //                         # (Detection too close to the sensor to quantify)
    //                         # +Inf represents no detection within the fixed distance.
    //                         # (Object out of range)
    struct Range {
        static constexpr uint8_t ULTRASOUND = 0;
        static constexpr uint8_t INFRARED = 1;

        std_msgs::msg::Header header;
        uint8_t radiation_type;
        float field_of_view;
        float min_range;
        float max_range;
        float range;

        MSGPACK_DEFINE(header, radiation_type, field_of_view, min_range, max_range, range);

#ifdef ROS
        Range& operator=(const ::sensor_msgs::msg::Range& rhs) {
            header = rhs.header;
            radiation_type = rhs.radiation_type;
            field_of_view = rhs.field_of_view;
            min_range = rhs.min_range;
            max_range = rhs.max_range;
            range = rhs.range;
            return *this;
        }
        Range& operator=(::sensor_msgs::msg::Range&& rhs) {
            header = std::move(rhs.header);
            radiation_type = std::move(rhs.radiation_type);
            field_of_view = std::move(rhs.field_of_view);
            min_range = std::move(rhs.min_range);
            max_range = std::move(rhs.max_range);
            range = std::move(rhs.range);
            return *this;
        }
#endif
    };

    // # Single reading from a relative humidity sensor.
    // # Defines the ratio of partial pressure of water vapor to the saturated vapor
    // # pressure at a temperature.
    //
    // std_msgs/Header header # timestamp of the measurement
    //                              # frame_id is the location of the humidity sensor
    //
    // float64 relative_humidity    # Expression of the relative humidity
    //                              # from 0.0 to 1.0.
    //                              # 0.0 is no partial pressure of water vapor
    //                              # 1.0 represents partial pressure of saturation
    //
    // float64 variance             # 0 is interpreted as variance unknown
    struct RelativeHumidity {
        std_msgs::msg::Header header;
        double relative_humidity;
        double variance;
        MSGPACK_DEFINE(header, relative_humidity, variance);

#ifdef ROS
        RelativeHumidity& operator=(const ::sensor_msgs::msg::RelativeHumidity& rhs) {
            header = rhs.header;
            relative_humidity = rhs.relative_humidity;
            variance = rhs.variance;
            return *this;
        }
        RelativeHumidity& operator=(::sensor_msgs::msg::RelativeHumidity&& rhs) {
            header = std::move(rhs.header);
            relative_humidity = std::move(rhs.relative_humidity);
            variance = std::move(rhs.variance);
            return *this;
        }
#endif
    };

    // # Single temperature reading.
    //
    // std_msgs/Header header # timestamp is the time the temperature was measured
    //                              # frame_id is the location of the temperature reading
    //
    // float64 temperature          # Measurement of the Temperature in Degrees Celsius.
    //
    // float64 variance             # 0 is interpreted as variance unknown.
    struct Temperature {
        std_msgs::msg::Header header;
        double temperature;
        double variance;
        MSGPACK_DEFINE(header, temperature, variance);

#ifdef ROS
        Temperature& operator=(const ::sensor_msgs::msg::Temperature& rhs) {
            header = rhs.header;
            temperature = rhs.temperature;
            variance = rhs.variance;
            return *this;
        }
        Temperature& operator=(::sensor_msgs::msg::Temperature&& rhs) {
            header = std::move(rhs.header);
            temperature = std::move(rhs.temperature);
            variance = std::move(rhs.variance);
            return *this;
        }
#endif
    };

    // # Measurement from an external time source not actively synchronized with the system clock.
    //
    // std_msgs/Header header      # stamp is system time for which measurement was valid
    //                                   # frame_id is not used
    //
    // builtin_interfaces/Time time_ref  # corresponding time from this external source
    // string source                     # (optional) name of time source
    struct TimeReference {
        std_msgs::msg::Header header;
        rcl_interfaces::builtin_interfaces::msg::Time time_ref;
        MsgPack::str_t source;
        MSGPACK_DEFINE(header, time_ref, source);

#ifdef ROS
        TimeReference& operator=(const ::sensor_msgs::msg::TimeReference& rhs) {
            header = rhs.header;
            time_ref = rhs.time_ref;
            source = rhs.source;
            return *this;
        }
        TimeReference& operator=(::sensor_msgs::msg::TimeReference&& rhs) {
            header = std::move(rhs.header);
            time_ref = std::move(rhs.time_ref);
            source = std::move(rhs.source);
            return *this;
        }
#endif
    };

}  // namespace msg

namespace srv {}

}  // namespace sensor_msgs

ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_NAMESPACE_END

#endif  // ARDUINO_MSGPACK_ROS_INTERFACES_COMMON_INTERFACES_SENSOR_MSGS_H
