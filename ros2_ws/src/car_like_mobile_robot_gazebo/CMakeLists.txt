cmake_minimum_required(VERSION 3.8)
project(car_like_mobile_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
