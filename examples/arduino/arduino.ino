#include <Arduino.h>
#include <MsgPacketizer.h>
#include <MsgPackRosInterfaces.h>

MsgPackRosIF::geometry_msgs::msg::Twist twist_cmd;
MsgPackRosIF::geometry_msgs::msg::Twist twist_res;
MsgPackRosIF::geometry_msgs::msg::Pose pose_res;

static constexpr float PUBLISH_RATE {20.f};  // 20 Hz

void setup() {
    Serial.begin(115200);
    delay(2000);

    // subscribe twist_cmd (automatically updated)
    MsgPacketizer::subscribe(Serial, 0x01, twist_cmd);

    // publish twist_res and pose_res
    MsgPacketizer::publish(Serial, 0x02, twist_res)->setFrameRate(PUBLISH_RATE);
    MsgPacketizer::publish(Serial, 0x03, pose_res)->setFrameRate(PUBLISH_RATE);
}

void loop() {
    // update twist_res
    twist_res.linear.x = twist_cmd.linear.x;
    twist_res.linear.y = twist_cmd.linear.y;
    twist_res.linear.z = 0.;
    twist_res.angular.x = 0.;
    twist_res.angular.y = 0.;
    twist_res.angular.z = twist_cmd.angular.z;

    // update odometry
    pose_res.position.x = millis() * 0.001;
    pose_res.position.y = millis() * 0.001 * 0.5;
    pose_res.position.z = 0.;
    pose_res.orientation.x = millis() * 0.001;
    pose_res.orientation.y = millis() * 0.001 * 0.5;
    pose_res.orientation.z = millis() * 0.001 * 0.1;
    pose_res.orientation.w = millis() * 1.;

    // receive packets and send packets in PUBLISH_RATE automatically
    MsgPacketizer::update();

    delay(5);
}
