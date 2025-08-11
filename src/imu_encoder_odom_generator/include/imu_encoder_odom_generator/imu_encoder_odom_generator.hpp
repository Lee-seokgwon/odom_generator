#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace imu_encoder_odom
{

class OdomGenerator : public rclcpp::Node
{
public:
  OdomGenerator();
  ~OdomGenerator() = default;

private:
  // Callback functions
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void leftEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void rightEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void odomTimerCallback();

  // Odometry calculation
  void calcOdometry();
  double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat);

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Robot parameters
  static constexpr double WHEEL_RADIUS = 0.0535;  // 바퀴 반지름 (m) - 실제 값으로 수정 필요 (mdh80)
  static constexpr double WHEEL_BASE = 0.35;    // 바퀴 간격 (m) - 실제 값으로 수정 필요
  static constexpr int32_t ONE_ROUND_ENC = 4096; // 한 바퀴당 엔코더 틱 - 실제 값으로 수정 필요

  // Sensor data
  geometry_msgs::msg::Quaternion imu_quaternion_;
  int32_t left_wheel_encoder_;
  int32_t right_wheel_encoder_;
  
  // Previous values for calculation
  int32_t prev_left_encoder_;
  int32_t prev_right_encoder_;
  double last_theta_;
  
  // Odometry state
  double odom_x_;
  double odom_y_;
  double odom_yaw_;
  double yaw_est_;
  
  // Velocity
  double left_wheel_vel_;
  double right_wheel_vel_;
  double linear_vel_;
  double angular_vel_;

  // Timing
  rclcpp::Time odom_prev_time_;
  rclcpp::Time min_dt_time_;
  
  // Status flags
  bool imu_received_;
  bool left_encoder_received_;
  bool right_encoder_received_;
  bool first_calculation_;
};

} // namespace imu_encoder_odom
