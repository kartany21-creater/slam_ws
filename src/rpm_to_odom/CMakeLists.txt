cmake_minimum_required(VERSION 3.8)
project(rpm_to_odom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rpm_to_odom_node src/rpm_to_odom_node.cpp)
ament_target_dependencies(rpm_to_odom_node rclcpp std_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS
  rpm_to_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

