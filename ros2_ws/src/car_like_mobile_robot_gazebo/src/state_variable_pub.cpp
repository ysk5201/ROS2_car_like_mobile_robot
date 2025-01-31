#include "car_like_mobile_robot_gazebo/state_variable_pub.hpp"

SubPubNode::SubPubNode() : Node("state_variable_pub"), latest_data_("No data yet")
{
    // サブスクライバーを作成
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/input_topic", 10,
        std::bind(&SubPubNode::topic_callback, this, std::placeholders::_1));

    // パブリッシャーを作成
    publisher_ = this->create_publisher<std_msgs::msg::String>("/output_topic", 10);

    // タイマーを作成（1秒ごとにパブリッシュ）
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SubPubNode::timer_callback, this));
}

// サブスクライバーのコールバック
void SubPubNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    latest_data_ = msg->data;
}

// タイマーのコールバック（定期的にパブリッシュ）
void SubPubNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Published: " + latest_data_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

// メイン関数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubPubNode>());
    rclcpp::shutdown();
    return 0;
}
