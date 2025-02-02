#ifndef JOY_TO_CMD_VEL_HPP_
#define JOY_TO_CMD_VEL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToCmdVel : public rclcpp::Node {
public:
    JoyToCmdVel();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void update_cmd_vel();

    //cmd_vel関連
    double target_linear_x_, target_angular_z_;
    double current_linear_x_, current_angular_z_;
    double linear_max_, angular_max_;
    double linear_acceleration_, angular_acceleration_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

#endif  // JOY_TO_CMD_VEL_HPP_
