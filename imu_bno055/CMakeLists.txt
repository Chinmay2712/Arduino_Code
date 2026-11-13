cmake_minimum_required(VERSION 3.8)
project(imu_bno055)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(bno055_node src/bno055_node.cpp src/bno055.cpp)
ament_target_dependencies(bno055_node rclcpp sensor_msgs std_msgs yaml-cpp)

target_link_libraries(bno055_node yaml-cpp)

add_executable(bno055_calib_dump src/bno055_calib_dump.cpp src/bno055.cpp)
ament_target_dependencies(bno055_calib_dump rclcpp sensor_msgs std_msgs yaml-cpp)
  
install(TARGETS bno055_node
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS bno055_calib_dump
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
