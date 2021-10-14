// ROS2 core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// ROS2 messages
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

// libraries
#include "serial/serial.h"
#include "msgpack_ros_interface/MsgPacketizer/MsgPacketizer.h"
#include "msgpack_ros_interface/MsgPackRosInterfaces/MsgPackRosInterfaces.h"

class OmniCore : public rclcpp::Node {
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    std::string port {"/dev/ttyACM0"};
    serial::Serial serial;

    MsgPackRosIF::geometry_msgs::msg::Twist twist_cmd;
    MsgPackRosIF::geometry_msgs::msg::Twist twist_res;

public:
    explicit OmniCore(const std::string& node_name) : Node(node_name) {
        // setup serial
        this->serial.setBaudrate(115200);
        this->serial.setPort(this->port);
        this->serial.open();

        if (!this->serial.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "serial port", this->port, "not found");
        } else {
            RCLCPP_ERROR(this->get_logger(), "serial port", this->port, "opend");
        }

        // setup publishers
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        this->pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

        // setup publishers for serial
        // twist_cmd
        MsgPacketizer::publish(this->serial, 0x01, twist_cmd)->setFrameRate(20);

        // setup subscribers for serial
        // twist res
        MsgPacketizer::subscribe(this->serial, 0x02, twist_res);

        // pose_res
        MsgPacketizer::subscribe(this->serial, 0x03, [this](const MsgPackRosIF::geometry_msgs::msg::Pose& pose_res) {
            rclcpp::Time now = this->now();

            RCLCPP_INFO(
                this->get_logger(),
                "twist: %.2lf %.2lf %.2lf / pos: %.2lf %.2lf / quat: %.2lf %.2lf %.2lf %.2lf",
                twist_res.linear.x,
                twist_res.linear.y,
                twist_res.angular.z,
                pose_res.position.x,
                pose_res.position.y,
                pose_res.orientation.x,
                pose_res.orientation.y,
                pose_res.orientation.z,
                pose_res.orientation.w);

            // set odom message
            auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
            odom_msg->header.frame_id = "odom";
            odom_msg->child_frame_id = "base_footprint";
            odom_msg->header.stamp = now;

            odom_msg->pose.pose.position.x = pose_res.position.x;
            odom_msg->pose.pose.position.y = pose_res.position.y;
            odom_msg->pose.pose.position.z = pose_res.position.z;
            odom_msg->pose.pose.orientation.x = pose_res.orientation.x;
            odom_msg->pose.pose.orientation.y = pose_res.orientation.y;
            odom_msg->pose.pose.orientation.z = pose_res.orientation.z;
            odom_msg->pose.pose.orientation.w = pose_res.orientation.w;

            odom_msg->twist.twist.linear.x = twist_res.linear.x;
            odom_msg->twist.twist.linear.y = twist_res.linear.y;
            odom_msg->twist.twist.linear.z = twist_res.linear.z;
            odom_msg->twist.twist.angular.x = twist_res.angular.x;
            odom_msg->twist.twist.angular.y = twist_res.angular.y;
            odom_msg->twist.twist.angular.z = twist_res.angular.z;

            // publish /odom
            pub->publish(std::move(odom_msg));
        });

        // periodic update in 200fps
        using namespace std::chrono_literals;
        this->timer = create_wall_timer(5ms, [this]() {
            rclcpp::Time now = this->now();

            // set dummy twist_cmd
            twist_cmd.linear.x = now.seconds();
            twist_cmd.linear.y = now.seconds() * 0.5;
            twist_cmd.angular.z = now.seconds() * 0.1;

            MsgPacketizer::update();  // send/receive serial packets
        });
    }
};

int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OmniCore>("msgpack_ros_interface");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
