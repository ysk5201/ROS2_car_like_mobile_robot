#include "cmd_vel_driver/joy_to_cmd_vel.hpp"

JoyToCmdVel::JoyToCmdVel() : Node("joy_to_cmd_vel") {
     // パラメータ宣言
    this->declare_parameter("linear_max", 1.0);
    this->declare_parameter("angular_max", 1.0);
    this->declare_parameter("linear_acceleration", 1.0);
    this->declare_parameter("angular_acceleration", 1.0);
    this->declare_parameter("publish_rate", 50.0);  //Hz

    // パラメータ取得
    linear_max_ = this->get_parameter("linear_max").as_double();
    angular_max_ = this->get_parameter("angular_max").as_double();
    linear_acceleration_ = this->get_parameter("linear_acceleration").as_double();
    angular_acceleration_ = this->get_parameter("angular_acceleration").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // Joy メッセージのサブスクライバー
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

    // cmd_vel メッセージのパブリッシャー
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 1);

    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);   //周期に変換
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&JoyToCmdVel::update_cmd_vel, this));
    
    pre_time_ = this->now();

}

void JoyToCmdVel::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
     // 左スティックのY軸（前後移動）
    target_linear_x_ = msg->axes[1] * linear_max_;

    // 右スティックのX軸（左右旋回）
    target_angular_z_ = msg->axes[3] * angular_max_;
}

void JoyToCmdVel::update_cmd_vel() {
    rclcpp::Time current_time = this->now();

    double dt = (current_time - pre_time_).seconds();

    // 直進速度の補間
    if (current_linear_x_ < target_linear_x_) {
        current_linear_x_ = std::min(current_linear_x_ + linear_acceleration_ * dt, target_linear_x_);
    } else if (current_linear_x_ > target_linear_x_) {
        current_linear_x_ = std::max(current_linear_x_ - linear_acceleration_ * dt, target_linear_x_);
    }

    // 旋回速度の補間
    if (current_angular_z_ < target_angular_z_) {
        current_angular_z_ = std::min(current_angular_z_ + angular_acceleration_ * dt, target_angular_z_);
    } else if (current_angular_z_ > target_angular_z_) {
        current_angular_z_ = std::max(current_angular_z_ - angular_acceleration_ * dt, target_angular_z_);
    }

    // cmd_vel メッセージの作成とパブリッシュ
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = current_linear_x_;
    twist.angular.z = current_angular_z_;
    cmd_vel_pub_->publish(twist);

    RCLCPP_INFO(this->get_logger(), "cmd_vel: linear=%.2f, angular=%.2f", twist.linear.x, twist.angular.z);

    pre_time_ = current_time;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
