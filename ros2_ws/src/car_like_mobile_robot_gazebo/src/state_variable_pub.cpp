#include "car_like_mobile_robot_gazebo/state_variable_pub.hpp"

SubPubNode::SubPubNode() : Node("state_variable_pub"), x_(0.0), y_(0.0), theta_(0.0), phi_(0.0)
{
    this->declare_parameter<double>("publish_rate", 50.0);
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // サブスクライバーを作成
    ground_truth_sub_= this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ground_truth", 1,
        std::bind(&SubPubNode::groundTruthCallback, this, std::placeholders::_1));

    joint_states_sub_= this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1,
        std::bind(&SubPubNode::jointStatesCallback, this, std::placeholders::_1));

    // パブリッシャーを作成
    state_variable_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_variable", 1);

    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);  //周期に変換
    // タイマーを作成（1秒ごとにパブリッシュ）
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&SubPubNode::timer_callback, this));
}

// サブスクライバーのコールバック
void SubPubNode::groundTruthCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    x_ = msg->poses[0].position.x;
    y_ = msg->poses[0].position.y;
    theta_ = msg->poses[0].orientation.z;
}

//[0]:front_left_wheel_joint, [1]:front_right_wheel_joint
//[2]:front_left_steering_joint, [3]:front_right_steering_joint
//[4]:rear_left_wheel_joint, [5]:rear_right_wheel_joint

void SubPubNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    double front_left_steering_angle = msg->position[2];
    double front_right_steering_angle = msg->position[3];

    phi_ = (front_left_steering_angle + front_right_steering_angle) / 2.0;
    
}

// タイマーのコールバック（定期的にパブリッシュ）
void SubPubNode::timer_callback()
{
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {x_, y_, theta_, phi_};  // 送信するデータ

    state_variable_pub_->publish(message);
}

// メイン関数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubPubNode>());
    rclcpp::shutdown();
    return 0;
}
