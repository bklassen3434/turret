"""Launch file for turret tracking demo without Gazebo.

This launch file runs the complete tracking pipeline:
- Target simulator (broadcasts drone target TF positions)
- Synthetic camera (reads target TFs, renders drone shapes in image)
- Vision detector (detects red drones in images)
- Radar simulators (x2, one per drone target)
- Sensor fusion (multi-target Kalman filter)
- Turret controller (PID)
- Joint simulator (simulates turret movement)
- RViz visualization

No Gazebo required - everything is simulated in ROS nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    turret_description_dir = get_package_share_directory('turret_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # URDF file
    urdf_file = os.path.join(turret_description_dir, 'urdf', 'turret.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization (set false to run headless)'
        ),

        # Robot state publisher (for URDF/TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),

        # Joint simulator (simulates turret joints)
        Node(
            package='turret_control',
            executable='joint_simulator_node',
            name='joint_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'update_rate': 100.0,
                'max_velocity': 3.0,  # rad/s
            }]
        ),

        # Target simulator (broadcasts 2 drone TF positions)
        Node(
            package='turret_bringup',
            executable='target_sim_node',
            name='target_sim',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_range': 5.0,
                'speed': 0.1,
                'pattern_width': 1.047,
                'pattern_height': 0.785,
                'num_targets': 2,
                'drone2_range': 7.0,
                'drone2_speed': 0.08,
                'drone2_phase_offset': 0.52,
            }]
        ),

        # Synthetic camera (reads drone TFs, renders drone shapes)
        Node(
            package='turret_vision',
            executable='synthetic_camera_node',
            name='synthetic_camera',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_width': 640,
                'image_height': 480,
                'frame_rate': 30.0,
                'drone_base_size': 30,
                'reference_distance': 5.0,
                'pixel_noise_std': 3.0,
                'image_noise_std': 15.0,
                'detection_probability': 0.95,
                'brightness_noise_std': 20.0,
                'num_targets': 2,
            }]
        ),

        # Vision detector (detects red drones in images)
        Node(
            package='turret_vision',
            executable='vision_detector_node',
            name='vision_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_detections_per_frame': 10,
            }]
        ),

        # Radar simulator instance 1 (watches drone 1)
        Node(
            package='turret_radar',
            executable='radar_sim_node',
            name='radar_sim_1',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'target_drone_1::link',
                'range_noise_std': 0.3,
                'range_rate_noise_std': 0.15,
                'azimuth_noise_std': 0.05,
                'elevation_noise_std': 0.05,
                'detection_probability': 0.90,
            }]
        ),

        # Radar simulator instance 2 (watches drone 2)
        Node(
            package='turret_radar',
            executable='radar_sim_node',
            name='radar_sim_2',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'target_drone_2::link',
                'range_noise_std': 0.3,
                'range_rate_noise_std': 0.15,
                'azimuth_noise_std': 0.05,
                'elevation_noise_std': 0.05,
                'detection_probability': 0.90,
            }]
        ),

        # Sensor fusion (multi-target Kalman filter)
        Node(
            package='turret_fusion',
            executable='fusion_node',
            name='fusion',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # Turret controller (PID) - conservative tuning
        Node(
            package='turret_control',
            executable='turret_controller_node',
            name='turret_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'control_rate': 100.0,
                'yaw_kp': 4.0,
                'yaw_ki': 0.0,
                'yaw_kd': 0.8,
                'pitch_kp': 4.0,
                'pitch_ki': 0.0,
                'pitch_kd': 0.8,
            }]
        ),

        # Target marker node (visualizes tracked targets in RViz)
        Node(
            package='turret_fusion',
            executable='target_marker_node',
            name='target_marker',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),

        # RViz for visualization (optional — disable with use_rviz:=false)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(turret_description_dir, 'config', 'turret_sim.rviz')],
        ),
    ])
