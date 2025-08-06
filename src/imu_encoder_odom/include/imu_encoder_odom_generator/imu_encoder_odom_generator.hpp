// include/odom_generator/odom_generator.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"

extern int a;  // extern 변수 a 선언

class OdomGeneratorNode : public rclcpp::Node
{
public:
  OdomGeneratorNode() : Node("odom_generator_node")
  {
    RCLCPP_INFO(this->get_logger(), "OdomGeneratorNode is running!");
    RCLCPP_INFO(this->get_logger(), "The value of a is: %d", a);  // extern 변수 a 출력
  }
};