#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
import math
import time
from geometry_msgs.msg import Quaternion

class DummyDataPublisher(Node):
    def __init__(self):
        super().__init__('dummy_data_publisher')
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.left_encoder_pub = self.create_publisher(Int32, '/mdh250_l_en', 10)
        self.right_encoder_pub = self.create_publisher(Int32, '/mdh250_r_en', 10)
        
        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.publish_dummy_data)
        
        # Simulation state
        self.current_yaw = 0.0
        self.left_encoder_value = 0
        self.right_encoder_value = 0
        self.time_elapsed = 0.0
        
        # Robot parameters
        self.WHEEL_RADIUS = 0.033  # m
        self.ONE_ROUND_ENC = 1000  # ticks per revolution
        self.SIMULATION_SPEED = 0.1  # m/s
        self.SIMULATION_ANGULAR_SPEED = 0.2  # rad/s
        
        self.get_logger().info('🎯 Dummy Data Publisher Started!')
        self.get_logger().info('Publishing dummy data to:')
        self.get_logger().info('  - /imu (IMU data with rotating yaw)')
        self.get_logger().info('  - /mdh250_l_en (Left encoder)')
        self.get_logger().info('  - /mdh250_r_en (Right encoder)')
        self.get_logger().info('Robot simulation: Moving forward with slow rotation')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        quat = Quaternion()
        quat.w = cr * cp * cy + sr * sp * sy
        quat.x = sr * cp * cy - cr * sp * sy
        quat.y = cr * sp * cy + sr * cp * sy
        quat.z = cr * cp * sy - sr * sp * cy
        
        return quat
    
    def publish_dummy_data(self):
        current_time = self.get_clock().now()
        dt = 0.02  # 50Hz = 0.02초
        self.time_elapsed += dt
        
        # 시뮬레이션: 직진하면서 천천히 회전하는 로봇
        linear_distance = self.SIMULATION_SPEED * dt  # 직진 거리
        self.current_yaw += self.SIMULATION_ANGULAR_SPEED * dt  # 회전
        
        # 엔코더 값 계산 (양쪽 바퀴가 같은 속도로 회전)
        wheel_rotation = linear_distance / self.WHEEL_RADIUS  # 바퀴 회전 각도 (라디안)
        encoder_ticks = int(wheel_rotation * self.ONE_ROUND_ENC / (2 * math.pi))
        
        self.left_encoder_value += encoder_ticks
        self.right_encoder_value += encoder_ticks
        
        # 1. IMU 메시지 발행
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # 쿼터니언 설정 (yaw만 변화)
        imu_msg.orientation = self.euler_to_quaternion(0.0, 0.0, self.current_yaw)
        
        # 각속도 설정
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = self.SIMULATION_ANGULAR_SPEED
        
        # 선형가속도 설정 (정지 상태)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # 중력
        
        self.imu_pub.publish(imu_msg)
        
        # 2. 왼쪽 엔코더 발행
        left_enc_msg = Int32()
        left_enc_msg.data = self.left_encoder_value
        self.left_encoder_pub.publish(left_enc_msg)
        
        # 3. 오른쪽 엔코더 발행
        right_enc_msg = Int32()
        right_enc_msg.data = self.right_encoder_value
        self.right_encoder_pub.publish(right_enc_msg)
        
        # 5초마다 상태 로그
        if int(self.time_elapsed) % 5 == 0 and int(self.time_elapsed * 50) % 250 == 0:
            self.get_logger().info(f'📊 Status - Yaw: {self.current_yaw:.2f}rad, '
                                 f'Left Enc: {self.left_encoder_value}, '
                                 f'Right Enc: {self.right_encoder_value}')

def main(args=None):
    rclpy.init(args=args)
    
    dummy_publisher = DummyDataPublisher()
    
    try:
        rclpy.spin(dummy_publisher)
    except KeyboardInterrupt:
        dummy_publisher.get_logger().info('🛑 Dummy publisher stopped')
    finally:
        dummy_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()