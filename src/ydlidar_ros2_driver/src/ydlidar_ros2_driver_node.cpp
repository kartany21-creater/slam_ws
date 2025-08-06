#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s", ROS2Verision);

  CYdLidar laser;

  std::string str_optvalue;
  str_optvalue = node->declare_parameter<std::string>("port", "/dev/USB0");
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  str_optvalue = node->declare_parameter<std::string>("ignore_array", "");
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = node->declare_parameter<std::string>("frame_id", "laser_frame");

  int optval;
  optval = node->declare_parameter<int>("baudrate", 512000);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

  optval = node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  optval = node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

  optval = node->declare_parameter<int>("sample_rate", 9);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

  optval = node->declare_parameter<int>("abnormal_check_count", 4);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  optval = node->declare_parameter<int>("intensity_bit", 0);
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  bool b_optvalue;
  b_optvalue = node->declare_parameter<bool>("fixed_resolution", false);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("reversion", true);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("inverted", true);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("auto_reconnect", true);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("isSingleChannel", false);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("intensity", false);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("support_motor_dtr", false);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  b_optvalue = node->declare_parameter<bool>("debug", false);
  laser.setEnableDebug(b_optvalue);

  float f_optvalue;
  f_optvalue = node->declare_parameter<float>("angle_max", 180.0f);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));

  f_optvalue = node->declare_parameter<float>("angle_min", -180.0f);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  f_optvalue = node->declare_parameter<float>("range_max", 64.f);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));

  f_optvalue = node->declare_parameter<float>("range_min", 0.1f);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

  f_optvalue = node->declare_parameter<float>("frequency", 10.f);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = node->declare_parameter<bool>("invalid_range_is_inf", false);

  bool ret = laser.initialize();

  if (ret) {
    int i_v;
    i_v = node->declare_parameter<int>("m1_mode", 0);
    laser.setWorkMode(i_v, 0x01);

    i_v = node->declare_parameter<int>("m2_mode", 0);
    laser.setWorkMode(i_v, 0x02);

    i_v = node->declare_parameter<int>("m3_mode", 1);
    laser.setWorkMode(i_v, 0x04);

    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
  }

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto stop_service = node->create_service<std_srvs::srv::Empty>(
    "stop_scan",
    [&laser](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool {
      return laser.turnOff();
    });

  auto start_service = node->create_service<std_srvs::srv::Empty>(
    "start_scan",
    [&laser](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool {
      return laser.turnOn();
    });

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for (size_t i = 0; i < scan.points.size(); i++) {
        const auto& p = scan.points.at(i);
        int index = std::ceil((p.angle - scan.config.min_angle) / scan.config.angle_increment);
        if (index >= 0 && index < size) {
          scan_msg->ranges[index] = p.range;
          scan_msg->intensities[index] = p.intensity;
        }
      }
      laser_pub->publish(*scan_msg);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }

    if (!rclcpp::ok()) {
      break;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}

