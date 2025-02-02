#include "cmd_vel_driver/joy_to_cmd_vel.hpp"

JoyToCmdVel::JoyToCmdVel() : Node("joy_to_cmd_vel") {
    // パラメータの宣言
    this->declare_parameter("linear_scale", 0.5);
    this->declare_parameter("angular_scale", 0.5);

    // Joy メッセージのサブスクライバー
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

    // cmd_vel メッセージのパブリッシャー
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 1);
}

void JoyToCmdVel::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto twist = geometry_msgs::msg::Twist();

    // パラメータの取得
    double linear_scale = this->get_parameter("linear_scale").as_double();
    double angular_scale = this->get_parameter("angular_scale").as_double();

    // 左スティックのY軸（前後移動）
    twist.linear.x = msg->axes[1] * linear_scale;

    // 右スティックのX軸（左右回転）
    twist.angular.z = msg->axes[3] * angular_scale;

    cmd_vel_pub_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear=%.2f, angular=%.2f", twist.linear.x, twist.angular.z);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
