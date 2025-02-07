cmake_minimum_required(VERSION 3.8)
project(file_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(car_logger
  src/car_logger.cpp
)

ament_target_dependencies(car_logger rclcpp std_msgs)

install(TARGETS
  car_logger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
