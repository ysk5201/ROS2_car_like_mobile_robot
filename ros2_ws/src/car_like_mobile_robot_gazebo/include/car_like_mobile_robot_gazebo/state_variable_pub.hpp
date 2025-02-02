#ifndef SUB_PUB_NODE_HPP_
#define SUB_PUB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>


class SubPubNode : public rclcpp::Node
{
public:
    SubPubNode();

private:
    void groundTruthCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ground_truth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_;
    double y_;
    double theta_;
    double phi_;
    std::string latest_data_;
};

#endif  // SUB_PUB_NODE_HPP_
