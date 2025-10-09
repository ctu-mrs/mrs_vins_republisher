import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from mrs_lib import RemappingsCustomConfigParser, sanitize_custom_config_path
from launch.substitutions import PythonExpression

def generate_launch_description():
    """
    Example launch file demonstrating VINS Republisher functionality
    This example simulates a typical UAV setup with VINS odometry republishing
    """

    custom_config_arg = DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description='config from the user'
    )

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

    fcu_frame_arg = DeclareLaunchArgument(
        'fcu_frame',
        default_value='fcu',
        description='FCU frame - the root of the transform tree'
    )

    vins_world_frame_arg = DeclareLaunchArgument(
        'vins_world_frame',
        default_value='mrs_vins_world',
        description='VINS world frame - the "root" of the transform tree according to the VIO'
    )

    vins_fcu_frame_arg = DeclareLaunchArgument(
        'vins_fcu_frame',
        default_value='imu',
        description='VINS FCU frame - typically the IMU frame which is rigidly attached to the drone body'
    )

    vins_camera_mount_frame_arg = DeclareLaunchArgument(
        'camera_mount_frame',
        default_value='vins_body_front',
        description='VINS FCU frame - typically the IMU frame which is rigidly attached to the drone body'
    )

    # Declare launch argument for transform coordinates as array
    camera_mount_coords_arg = DeclareLaunchArgument(
        'camera_mount_coords',
        default_value='0.0 0.0 0.0 0.0 0.0 0.0',
        description='Transform coordinates as whitespace-separated values: x y z roll pitch yaw'
    )

    # Declare launch argument for using simulated time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Launch argument for using simulated time'
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
            # Parse coordinates from array argument with validation
            PythonExpression(["'", LaunchConfiguration('camera_mount_coords'), "'.split()[0]"]),
            PythonExpression(["'", LaunchConfiguration('camera_mount_coords'), "'.split()[1]"]),
            PythonExpression(["'", LaunchConfiguration('camera_mount_coords'), "'.split()[2]"]),
            PythonExpression(["'", LaunchConfiguration('camera_mount_coords'), "'.split()[3]"]),
            PythonExpression(["'", LaunchConfiguration('camera_mount_coords'), "'.split()[4]"]),
            PythonExpression(["'", LaunchConfiguration('camera_mount_coords'), "'.split()[5]"]),
            [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('fcu_frame')],
            [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('camera_mount_frame')]
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
            '0.0', LaunchConfiguration('camera_pitch'), '0.0',
            [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('camera_mount_frame')],
            [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('vins_fcu_frame')]
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

    republisher_node = ComposableNode(
        package='mrs_vins_republisher',
        plugin='vins_republisher::VinsRepublisher',
        name='vins_republisher',
        namespace=LaunchConfiguration('UAV_NAME'),
        parameters=[
            # config_file,  # Comment out if config file is causing issues
            {
                'uav_name': LaunchConfiguration('UAV_NAME'),
                'fcu_frame': [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('fcu_frame')],
                'mrs_vins_world_frame': [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('vins_world_frame')],
                'vins_fcu_frame': [LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('vins_fcu_frame')],
                # Provide all required parameters directly
                'rate_limiter/enabled': True,
                'rate_limiter/max_rate': 30.0,
                'velocity_in_body_frame': True,
                'init_in_zero': True,
                'compensate_initial_tilt': False,  # Set to false initially for simpler testing
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'custom_config': sanitize_custom_config_path(LaunchConfiguration('custom_config'))
            },
        ],
        remappings=[
            # subscribers
            ('~/odom_in', 'open_vins/odomimu'),
            # publishers
            ('~/odom_out', '~/odom'),
            ('~/status_string_out', '~/status_string'),
            # service servers
            ('~/calibrate_in', '~/calibrate'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    parser = RemappingsCustomConfigParser(republisher_node, LaunchConfiguration('custom_config'))

    # VINS Republisher as composable node
    vins_republisher_component = LoadComposableNodes(
        target_container=[LaunchConfiguration('UAV_NAME'), '/', LaunchConfiguration('UAV_NAME'), '_vinsrepublisher_manager'],
        composable_node_descriptions=[
            republisher_node
        ]
    )

    # Group all nodes under UAV namespace
    uav_group = GroupAction(
        actions=[
            tf_fcu_to_vins_front,
            tf_camera_pitch,
            component_container,
            parser,
            vins_republisher_component,
        ]
    )

    return LaunchDescription([
        # Launch arguments
        custom_config_arg,
        uav_name_arg,
        camera_pitch_arg,
        vins_type_arg,
        simulation_arg,
        fcu_frame_arg,
        vins_world_frame_arg,
        vins_fcu_frame_arg,
        vins_camera_mount_frame_arg,
        camera_mount_coords_arg,
        use_sim_time_arg,

        # UAV-specific nodes
        uav_group,
    ])
