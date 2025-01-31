#ifndef SUB_PUB_NODE_HPP_
#define SUB_PUB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class SubPubNode : public rclcpp::Node
{
public:
    SubPubNode();

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string latest_data_;
};

#endif  // SUB_PUB_NODE_HPP_
