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
    
    # vins_world_frame_arg = DeclareLaunchArgument(
    #     'vins_world_frame',
    #     default_value='vins_world',
    #     description='VINS world frame'
    # )
    
    # fcu_frame_arg = DeclareLaunchArgument(
    #     'fcu_frame',
    #     default_value=[LaunchConfiguration('UAV_NAME'), '/fcu'],
    #     description='FCU frame'
    # )
    
    # vins_fcu_frame_arg = DeclareLaunchArgument(
    #     'vins_fcu_frame',
    #     default_value='vins_body',
    #     description='VINS FCU frame'
    # )

    # Static transform publishers - these define the sensor mounting positions
    # Transform 1: FCU to VINS front frame (sensor mounting offset)
    tf_fcu_to_vins_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_vins_fcu_to_mrs_fcu',
        namespace=LaunchConfiguration('UAV_NAME'),
        arguments=[
            # Default camera mounting: 8.5cm forward, 13cm up, rotated -90deg in X and Z
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            [LaunchConfiguration('UAV_NAME'), '/fcu'],
            [LaunchConfiguration('UAV_NAME'), '/vins_body_front']
        ]
    )
    
    # Transform 2: Variable camera pitch adjustment
    tf_camera_pitch = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_vins_fcu_to_mrs_fcu2', 
        namespace=LaunchConfiguration('UAV_NAME'),
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            [LaunchConfiguration('UAV_NAME'), '/vins_body_front'],
            [LaunchConfiguration('UAV_NAME'), '/imu']
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
        }],
        #prefix=['xterm -e gdb -ex run --args']
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
                        'fcu_frame': [LaunchConfiguration('UAV_NAME'), '/fcu'],
                        'mrs_vins_world_frame': [LaunchConfiguration('UAV_NAME'), '/mrs_vins_world'],
                        'vins_fcu_frame': [LaunchConfiguration('UAV_NAME'), '/imu'],
                        # Provide all required parameters directly
                        'rate_limiter/enabled': True,
                        'rate_limiter/max_rate': 30.0,
                        'velocity_in_body_frame': True,
                        'init_in_zero': True,
                        'compensate_initial_tilt': False,  # Set to false initially for simpler testing
                        'use_sim_time': True
                    }
                ],
                remappings=[
                    # Default VINS remapping - change based on your VINS system
                    ('/uav1/vins_odom_in', '/uav1/odomimu'),
                    ('/uav1/vins_odom_out', '/odom'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
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
        
        # UAV-specific nodes
        uav_group,
    ])


# Additional helper function for different VINS configurations
#def create_openvins_config():
#    """Helper to create OpenVINS-specific configuration"""
#    return {
#        'vins_world_frame': 'ov_global',
#        'vins_fcu_frame': 'ov_imu', 
#        'input_topic': 'ov_msckf/odomimu',
#        'config_file': 'open_vins.yaml'
#    }

#def create_bluefox_config():
#    """Helper to create BlueHox downward camera configuration"""
#    return {
#        'camera_pitch': '-1.5708',  # -90 degrees for downward camera
#        'static_transform': '0.1 0.0 -0.15 -1.5708 0.0 -1.5708',
#        'config_file': 'vins_mono.yaml'
#    }
