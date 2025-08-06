from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # IMU Encoder Odometry Node
        Node(
            package='imu_encoder_odom_generator',
            executable='imu_encoder_odom_generator_node',
            name='imu_encoder_odom_generator',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # 필요시 토픽 리맵핑 추가
                # ('/imu', '/your_imu_topic'),
                # ('/mdh250_l_en', '/your_left_encoder_topic'),
                # ('/mdh250_r_en', '/your_right_encoder_topic'),
            ]
        ),
        
        # Optional: Static transform publisher for base_link to sensor frames
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_imu_tf',
        #     arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        # ),
    ])