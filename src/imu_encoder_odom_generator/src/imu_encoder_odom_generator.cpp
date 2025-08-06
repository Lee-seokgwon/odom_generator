#include "imu_encoder_odom_generator/imu_encoder_odom_generator.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace imu_encoder_odom
{

OdomGenerator::OdomGenerator()
: Node("imu_encoder_odom_generator_node"),
  left_wheel_encoder_(0),
  right_wheel_encoder_(0),
  prev_left_encoder_(0),
  prev_right_encoder_(0),
  last_theta_(0.0),
  odom_x_(0.0),
  odom_y_(0.0),
  odom_yaw_(0.0),
  yaw_est_(0.0),
  left_wheel_vel_(0.0),
  right_wheel_vel_(0.0),
  linear_vel_(0.0),
  angular_vel_(0.0),
  imu_received_(false),
  left_encoder_received_(false),
  right_encoder_received_(false),
  first_calculation_(true)
{
  // Initialize quaternion
  imu_quaternion_.w = 1.0;
  imu_quaternion_.x = 0.0;
  imu_quaternion_.y = 0.0;
  imu_quaternion_.z = 0.0;

  // Initialize timing
  odom_prev_time_ = this->now();
  min_dt_time_ = this->now();

  // Subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&OdomGenerator::imuCallback, this, std::placeholders::_1));
  
  left_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/mdh250_l_en", 10, std::bind(&OdomGenerator::leftEncoderCallback, this, std::placeholders::_1));
  
  right_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/mdh250_r_en", 10, std::bind(&OdomGenerator::rightEncoderCallback, this, std::placeholders::_1));

  // Publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Timer for odometry calculation (50Hz)
  odom_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&OdomGenerator::odomTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "IMU Encoder Odometry node started");
}

void OdomGenerator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_quaternion_ = msg->orientation;
  imu_received_ = true;
}

void OdomGenerator::leftEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  left_wheel_encoder_ = msg->data;
  left_encoder_received_ = true;
}

void OdomGenerator::rightEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  right_wheel_encoder_ = msg->data;
  right_encoder_received_ = true;
}

void OdomGenerator::odomTimerCallback()
{
  if (!imu_received_ || !left_encoder_received_ || !right_encoder_received_) {
    return;
  }

  calcOdometry();
}

double OdomGenerator::quaternionToYaw(const geometry_msgs::msg::Quaternion& quat)
{
  // Convert quaternion to yaw angle
  // Based on the original Arduino code: atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3)
  return atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
               1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

void OdomGenerator::calcOdometry()
{
  rclcpp::Time current_time = this->now();
  
  // Calculate time difference
  double dt = (current_time - odom_prev_time_).seconds();
  odom_prev_time_ = current_time;

  // Skip calculation if dt is too small or too large
  if (dt <= 0.01 || dt > 1.0) {
    return;
  }

  // Get current theta from IMU
  double theta = quaternionToYaw(imu_quaternion_);
  double delta_theta = theta - last_theta_;

  // Handle angle wrap-around
  if (delta_theta > M_PI) {
    delta_theta -= 2.0 * M_PI;
  } else if (delta_theta < -M_PI) {
    delta_theta += 2.0 * M_PI;
  }

  // Calculate encoder deltas (similar to original min_dt logic)
  double min_dt = (current_time - min_dt_time_).seconds();
  int32_t delta_left_encoder, delta_right_encoder;
  
  if (min_dt >= 1.0) {
    // Update recorded encoder values every second
    delta_left_encoder = left_wheel_encoder_ - prev_left_encoder_;
    delta_right_encoder = right_wheel_encoder_ - prev_right_encoder_;
    
    prev_left_encoder_ = left_wheel_encoder_;
    prev_right_encoder_ = right_wheel_encoder_;
    min_dt_time_ = current_time;
  } else {
    // Use accumulated deltas
    delta_left_encoder = left_wheel_encoder_ - prev_left_encoder_;
    delta_right_encoder = right_wheel_encoder_ - prev_right_encoder_;
  }

  // Skip if this is first calculation
  if (first_calculation_) {
    prev_left_encoder_ = left_wheel_encoder_;
    prev_right_encoder_ = right_wheel_encoder_;
    last_theta_ = theta;
    first_calculation_ = false;
    return;
  }

  // Calculate wheel velocities (2pi * encoder delta ratio / encoder update dt)
  left_wheel_vel_ = (2.0 * M_PI * static_cast<double>(delta_left_encoder) / ONE_ROUND_ENC) / min_dt;
  right_wheel_vel_ = (2.0 * M_PI * static_cast<double>(delta_right_encoder) / ONE_ROUND_ENC) / min_dt;

  // Calculate linear distance (radius = banzeereum)
  double delta_s = WHEEL_RADIUS * (left_wheel_vel_ + right_wheel_vel_) / 2.0;

  // Update estimated yaw (sensor fusion)
  yaw_est_ += delta_theta;

  // Update position (sensor fusion: IMU orientation + encoder distance)
  odom_x_ += cos(yaw_est_) * delta_s * dt;
  odom_y_ += sin(yaw_est_) * delta_s * dt;
  odom_yaw_ += delta_theta;

  // Calculate velocities
  linear_vel_ = delta_s;
  angular_vel_ = delta_theta / dt;

  // Create and publish odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // Position
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (convert yaw to quaternion)
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_est_);
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  // Velocity
  odom_msg.twist.twist.linear.x = linear_vel_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_vel_;

  // Publish odometry
  odom_pub_->publish(odom_msg);

  // Broadcast transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = current_time;
  transform_stamped.header.frame_id = "odom";
  transform_stamped.child_frame_id = "base_link";
  
  transform_stamped.transform.translation.x = odom_x_;
  transform_stamped.transform.translation.y = odom_y_;
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(transform_stamped);

  // Update last theta
  last_theta_ = theta;
}

} // namespace imu_encoder_odom