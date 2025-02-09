#include "file_logger/car_logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>

DataLogger::DataLogger() 
  : Node("car_logger")
{
  //パッケージ内のlogディレクトリを取得
  std::string package_directory = __FILE__;
  std::filesystem::path package_dir = std::filesystem::path(package_directory).parent_path().parent_path();
  std::string log_dir = package_dir.string() + "/log_files/";

  // /state_variable トピックのサブスクライバー作成
  debug_info_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/debug_info", 10,
    std::bind(&DataLogger::debugInfoCallback, this, std::placeholders::_1)
  );
  // ファイルを追記モードでオープン
  data_file_.open(log_dir+OUTPUT_FILE_NAME_, std::ios::out | std::ios::app);
  if (!data_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open data_log.txt");
  }
}

DataLogger::~DataLogger()
{
  if (data_file_.is_open()) {
    data_file_.close();
  }
}

void DataLogger::debugInfoCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // state変数は [0]: x, [1]: y, [2]: theta, [3]: phi が必要
  if (msg->data.size() < 6) {
    RCLCPP_WARN(this->get_logger(), "Received state_variable message with insufficient data.");
    return;
  }
  
  data_file_  << msg->data[0] << ", "   //t
              << msg->data[1] << ", "   //x
              << msg->data[2] << ", "   //y
              << msg->data[3] << ", "   //theta
              << msg->data[4] << ", "   //phi
              << msg->data[5] << ", "   //u1
              << msg->data[6] << ", "   //u2
              << msg->data[7] << ", "   //q
              << msg->data[8] << ", "   //d
              << msg->data[9] << "\n";  //thetap
  data_file_.flush();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
