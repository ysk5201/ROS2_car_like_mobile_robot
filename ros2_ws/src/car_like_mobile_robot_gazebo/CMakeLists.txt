cmake_minimum_required(VERSION 3.8)
project(car_like_mobile_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(state_variable_pub src/state_variable_pub.cpp)

ament_target_dependencies(state_variable_pub rclcpp std_msgs)

install(
  DIRECTORY 
  launch
  config
  # state_variable_pub
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  state_variable_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
