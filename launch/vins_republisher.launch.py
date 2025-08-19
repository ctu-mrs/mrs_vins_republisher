import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Example launch file demonstrating VINS Republisher functionality
    This example simulates a typical UAV setup with VINS odometry republishing
    """
    
    # Declare launch arguments
    uav_name_arg = DeclareLaunchArgument(
        'UAV_NAME',
        default_value=EnvironmentVariable('UAV_NAME', default_value='uav1'),
        description='UAV name for namespacing'
    )
    
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch',
        default_value='0.0',
        description='Camera pitch angle in radians (0.0=forward, -1.5708=down)'
    )
    
    vins_type_arg = DeclareLaunchArgument(
        'vins_type',
        default_value='default',
        description='VINS type: default, openvins, bluefox_down',
        choices=['default', 'openvins', 'bluefox_down']
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Whether running in simulation or real hardware'
    )

    # Frame arguments - these change based on VINS type
    mrs_vins_world_frame_arg = DeclareLaunchArgument(
        'mrs_vins_world_frame',
        default_value=[LaunchConfiguration('UAV_NAME'), '/mrs_vins_world'],
        description='MRS VINS world frame'
    )
    
    # Default VINS setup
    vins_world_frame_default = [LaunchConfiguration('UAV_NAME'), '/vins_world']
    vins_fcu_frame_default = [LaunchConfiguration('UAV_NAME'), '/vins_body']
    
    # OpenVINS setup  
    vins_world_frame_openvins = [LaunchConfiguration('UAV_NAME'), '/ov_global']
    vins_fcu_frame_openvins = [LaunchConfiguration('UAV_NAME'), '/ov_imu']
    
    vins_world_frame_arg = DeclareLaunchArgument(
        'vins_world_frame',
        default_value=vins_world_frame_default,
        description='VINS world frame'
    )
    
    fcu_frame_arg = DeclareLaunchArgument(
        'fcu_frame',
        default_value=[LaunchConfiguration('UAV_NAME'), '/fcu'],
        description='FCU frame'
    )
    
    vins_fcu_frame_arg = DeclareLaunchArgument(
        'vins_fcu_frame',
        default_value=vins_fcu_frame_default,
        description='VINS FCU frame'
    )
    
    vins_fcu_front_frame_arg = DeclareLaunchArgument(
        'vins_fcu_front_frame',
        default_value=[LaunchConfiguration('UAV_NAME'), '/vins_body_front'],
        description='VINS FCU front frame'
    )

    # Static transform publishers - these define the sensor mounting positions
    # Transform 1: FCU to VINS front frame (sensor mounting offset)
    tf_fcu_to_vins_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_vins_fcu_to_mrs_fcu',
        namespace=LaunchConfiguration('UAV_NAME'),
        arguments=[
            # Default camera mounting: 8.5cm forward, 13cm up, rotated -90deg in X and Z
            '0.085', '0.0', '0.13',
            '-1.5708', '0.0', '-1.5708',
            LaunchConfiguration('fcu_frame'),
            LaunchConfiguration('vins_fcu_front_frame')
        ]
    )
    
    # Transform 2: Variable camera pitch adjustment
    tf_camera_pitch = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_vins_fcu_to_mrs_fcu2', 
        namespace=LaunchConfiguration('UAV_NAME'),
        arguments=[
            '0', '0', '0',
            '0', '0', LaunchConfiguration('camera_pitch'),
            LaunchConfiguration('vins_fcu_front_frame'),
            LaunchConfiguration('vins_fcu_frame')
        ]
    )

    # Component container for the VINS republisher
    component_container = Node(
        package='rclcpp_components',
        executable='component_container',
        name=[LaunchConfiguration('UAV_NAME'), '_vinsrepublisher_manager'],
        namespace=LaunchConfiguration('UAV_NAME'),
        output='screen',
        parameters=[{
            'use_intra_process_comms': True,
        }]
    )

    # Get config file path - use default config for this example
    config_file = PathJoinSubstitution([
        FindPackageShare('mrs_vins_republisher'),
        'config',
        'default.yaml'
    ])

    # VINS Republisher as composable node
    vins_republisher_component = LoadComposableNodes(
        target_container=[LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('UAV_NAME'), '_vinsrepublisher_manager'],
        composable_node_descriptions=[
            ComposableNode(
                package='mrs_vins_republisher',
                plugin='vins_republisher::VinsRepublisher',
                name='vins_republisher',
                namespace=LaunchConfiguration('UAV_NAME'),
                parameters=[
                    # config_file,  # Comment out if config file is causing issues
                    {
                        'uav_name': LaunchConfiguration('UAV_NAME'),
                        'fcu_frame': LaunchConfiguration('fcu_frame'),
                        'mrs_vins_world_frame': LaunchConfiguration('mrs_vins_world_frame'),
                        'vins_fcu_frame': LaunchConfiguration('vins_fcu_frame'),
                        # Provide all required parameters directly
                        'rate_limiter/enabled': True,
                        'rate_limiter/max_rate': 30.0,
                        'velocity_in_body_frame': True,
                        'init_in_zero': True,
                        'compensate_initial_tilt': False,  # Set to false initially for simpler testing
                    }
                ],
                remappings=[
                    # Default VINS remapping - change based on your VINS system
                    ('~/vins_odom_in', 'vins_estimator/imu_propagate'),
                    ('~/vins_odom_out', '~/odom'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    # Note: For testing, you can manually publish to /uav1/vins_estimator/imu_propagate
    # or use: ros2 topic pub /uav1/vins_estimator/imu_propagate nav_msgs/msg/Odometry "{header: {frame_id: 'uav1/vins_world'}, child_frame_id: 'uav1/vins_body'}"

    # Example: Static world frame publisher (for visualization)
    world_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'world',
            LaunchConfiguration('mrs_vins_world_frame')
        ]
    )

    # Group all nodes under UAV namespace
    uav_group = GroupAction(
        actions=[
            tf_fcu_to_vins_front,
            tf_camera_pitch,
            component_container,
            vins_republisher_component,
        ]
    )

    return LaunchDescription([
        # Launch arguments
        uav_name_arg,
        camera_pitch_arg,
        vins_type_arg,
        simulation_arg,
        mrs_vins_world_frame_arg,
        vins_world_frame_arg,
        fcu_frame_arg,
        vins_fcu_frame_arg,
        vins_fcu_front_frame_arg,
        
        # World frame for visualization
        world_frame_publisher,
        
        # UAV-specific nodes
        uav_group,
    ])


# Additional helper function for different VINS configurations
def create_openvins_config():
    """Helper to create OpenVINS-specific configuration"""
    return {
        'vins_world_frame': 'ov_global',
        'vins_fcu_frame': 'ov_imu', 
        'input_topic': 'ov_msckf/odomimu',
        'config_file': 'open_vins.yaml'
    }

def create_bluefox_config():
    """Helper to create BlueHox downward camera configuration"""
    return {
        'camera_pitch': '-1.5708',  # -90 degrees for downward camera
        'static_transform': '0.1 0.0 -0.15 -1.5708 0.0 -1.5708',
        'config_file': 'vins_mono.yaml'
    }