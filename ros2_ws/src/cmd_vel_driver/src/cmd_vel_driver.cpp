#include "cmd_vel_driver/cmd_vel_driver.hpp"


CmdVelDriver::CmdVelDriver() : Node("cmd_vel_driver"),
    pre_time_(0.0),
    linear_velocity_(0.0), angular_z_(0.0),
    // x_(0.0), y_(0.0), th_(0.0), phi_(0.0),
    fl_steering_angle_(0.0), fr_steering_angle_(0.0),
    rl_linear_velocity_(0.0), rr_linear_velocity_(0.0) {
    initializeSubscribers();
    initializePublishers();
}

CmdVelDriver::~CmdVelDriver() {
    RCLCPP_INFO(this->get_logger(), "CmdVelDriver Node Shutting Down.");
}

void CmdVelDriver::initializeSubscribers() {
    // true_state_variables_sub_ = nh_.subscribe("/true_state_variables", 1, &CmdVelDriver::truePositionCallback, this);
}

void CmdVelDriver::getCmdVel() {
    // コールバック関数
    linear_velocity_ = 1.0;
    angular_z_ = 0.5;
}

void CmdVelDriver::calcCommand(double dt) {

    double velocity = linear_velocity_; // 後輪間中点の速度
    double omega    = angular_z_;       // 現在の旋回角速度

    double current_r = velocity / omega; // 現在の旋回半径
    double omega_l = (velocity - (0.5 * TREAD_WIDTH * omega)) / WHEEL_RADIUS;
    double omega_r = (velocity + (0.5 * TREAD_WIDTH * omega)) / WHEEL_RADIUS;

    double current_phi = atan(WHEEL_BASE / current_r); // 現在の前輪ステアリング角度
    double future_phi  = current_phi + (omega * dt);   // 未来の前輪ステアリング角度

    double future_r = WHEEL_BASE / tan(future_phi); // 未来の旋回半径
    double phi_l    = atan(WHEEL_BASE / (future_r - (0.5 * TREAD_WIDTH)));
    double phi_r    = atan(WHEEL_BASE / (future_r + (0.5 * TREAD_WIDTH)));

    // 未来の前輪ステアリング角度
    fl_steering_angle_ = phi_l;
    fr_steering_angle_ = phi_r;
    // 現在の後輪回転速度
    rl_linear_velocity_ = omega_l;
    rr_linear_velocity_ = omega_r;
}

void CmdVelDriver::publishCommand() {
    publishSteeringAngles(fl_steering_angle_, fr_steering_angle_);
    publishWheelAngularVelocities(rl_linear_velocity_, rr_linear_velocity_);
}

// std::vector<double> CmdVelDriver::getOutputVariables(double t) {
// 	std::array<double, 2> form_closure_score = evaluateFormClosureScore(true_vehicle1_pos_, true_vehicle2_pos_, true_vehicle3_pos_);
//     return {t, x_, y_, th_, phi_, s_, d_, dd, th1_ - thetat_, thetap1d};
// }


void CmdVelDriver::initializePublishers() {
    front_steering_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/front_steering_position_controller/commands", 1);
    rear_wheel_pub_      = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rear_wheel_speed_controller/commands", 1);
}

void CmdVelDriver::publishSteeringAngles(double phi_l, double phi_r) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {phi_l, phi_r};
    front_steering_pub_->publish(msg);
}

void CmdVelDriver::publishWheelAngularVelocities(double omega_l, double omega_r) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {omega_l, omega_r};
    rear_wheel_pub_->publish(msg);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto cmd_vel_driver = std::make_shared<CmdVelDriver>();

    rclcpp::Rate loop_rate(50);

    rclcpp::Time start_time = cmd_vel_driver->now();
    double pre_t = 0.0;

    while (rclcpp::ok()) {

        double t = (cmd_vel_driver->now() - start_time).seconds();
        double dt = t - pre_t;

        // RCLCPP_INFO(cmd_vel_driver->get_logger(), "dt = %f", dt);

        cmd_vel_driver->getCmdVel();      // cmd_velを取得
        cmd_vel_driver->calcCommand(dt);  // cmd_velを後輪回転角速度とステアリング角度に変換
        cmd_vel_driver->publishCommand(); // 後輪回転角速度とステアリング角度をpublsih

        pre_t = t;

        rclcpp::spin_some(cmd_vel_driver);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}