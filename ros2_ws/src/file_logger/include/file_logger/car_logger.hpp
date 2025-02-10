#ifndef DATA_LOGGER_HPP_
#define DATA_LOGGER_HPP_

#include <fstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class DataLogger : public rclcpp::Node
{
public:
  DataLogger();
  virtual ~DataLogger();

private:
  // 各トピックのコールバック宣言
  void debugInfoCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // サブスクライバー
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr debug_info_sub_;

  // 出力ファイル用ストリーム（両データをまとめて出力）
  std::ofstream data_file_;
};

#endif  // DATA_LOGGER_HPP_
