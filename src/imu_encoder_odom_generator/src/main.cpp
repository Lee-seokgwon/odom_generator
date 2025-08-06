#include "imu_encoder_odom_generator/imu_encoder_odom_generator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<imu_encoder_odom::OdomGenerator>();
  
  RCLCPP_INFO(node->get_logger(), "Starting IMU Encoder Odometry node...");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}