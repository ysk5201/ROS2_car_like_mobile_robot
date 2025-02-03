cmake_minimum_required(VERSION 3.8)
project(cmd_vel_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# 実行ファイルのビルド
add_executable(cmd_vel_driver src/cmd_vel_driver.cpp)
add_executable(joy_to_cmd_vel src/joy_to_cmd_vel.cpp)

# 必要なライブラリをリンク
ament_target_dependencies(cmd_vel_driver rclcpp geometry_msgs)
ament_target_dependencies(joy_to_cmd_vel rclcpp geometry_msgs sensor_msgs)

install(TARGETS 
  cmd_vel_driver
  joy_to_cmd_vel
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
