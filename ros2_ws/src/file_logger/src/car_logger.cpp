#include "file_logger/car_logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <chrono>

DataLogger::DataLogger() 
  : Node("car_logger")
{
  //パッケージ内のlogディレクトリを取得
  std::string package_directory = __FILE__;
  std::filesystem::path package_dir = std::filesystem::path(package_directory).parent_path().parent_path();
  std::string log_dir = package_dir.string() + "/log_files/";

  //現在の時間をYYMMDDHHMMSS形式に変換
  auto now = std::chrono::system_clock::now();
  auto now_time = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S");
  std::string time_str = ss.str();

  //ファイル名を作成
  std::string file_name = log_dir + "debug_log_" + time_str + ".csv";

  //log_filesディレクトリがあるかの確認
  if (!std::filesystem::exists(log_dir)) {
    if (!std::filesystem::create_directories(log_dir)) {
      std::cerr << "ディレクトリの作成に失敗しました: " << log_dir << std::endl;
    }      
  }

  // ファイルを新規作成（上書き）モードでオープン
  data_file_.open(file_name, std::ios::out | std::ios::trunc);
  if (!data_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open data_log.txt");
  }

  // debug_info トピックのサブスクライバー作成
  debug_info_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/debug_info", 10,
    std::bind(&DataLogger::debugInfoCallback, this, std::placeholders::_1)
  );  
}

DataLogger::~DataLogger()
{
  if (data_file_.is_open()) {
    data_file_.close();
  }
}

void DataLogger::debugInfoCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  int msg_size = msg->data.size();

  for (int i = 0; i < msg_size -1; i++) {
    data_file_  << msg->data[i] << ", " ;
  } 
  data_file_<< msg->data[msg_size-1] << "\n";

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
