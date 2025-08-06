// src/main.cpp
#include "rclcpp/rclcpp.hpp"
#include "imu_encoder_odom_generator/imu_encoder_odom_generator.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // ROS 2 초기화

  // HelloWorldNode 실행
  rclcpp::spin(std::make_shared<OdomGeneratorNode>());

  rclcpp::shutdown();  // ROS 2 종료
  return 0;
}
