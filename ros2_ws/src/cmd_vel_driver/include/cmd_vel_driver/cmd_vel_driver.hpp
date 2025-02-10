#pragma once

#ifndef CMD_VEL_DRIVER_HPP_
#define CMD_VEL_DRIVER_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>


class CmdVelDriver : public rclcpp::Node {
public:
    // Constractor and Destructor
    CmdVelDriver();
    ~CmdVelDriver();

private:
    // constants
    static constexpr double PI = 3.141592653589793;
    static constexpr double WHEEL_BASE = 1.0;      // 車軸間距離(m) (URDFと統一)
    static constexpr double TREAD_WIDTH = 0.856;   // 左右車輪間距離(m) (URDFと統一)
    static constexpr double WHEEL_RADIUS = 0.3;    // 車輪半径(m) (URDFと統一)

    // Member Variables
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double linear_velocity_, angular_z_;

    double fl_steering_angle_, fr_steering_angle_;
    double rl_linear_velocity_, rr_linear_velocity_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_wheel_pub_;
    
    // Member functions
    void initializeSubscribers();
    void initializePublishers();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void calcCommand();
    void publishCommand();
    void publishSteeringAngles(double phi_l, double phi_r);
    void publishWheelAngularVelocities(double omega_l, double omega_r);

};

#endif // CMD_VEL_DRIVER_HPP_