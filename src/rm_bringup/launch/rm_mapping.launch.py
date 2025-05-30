import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch_ros.actions import Node

def generate_launch_description():

    sc_liorf_localization_share_dir = get_package_share_directory('sc_liorf_localization')
    rviz_config_file = os.path.join(sc_liorf_localization_share_dir, 'rviz', 'localization.rviz')
    xacro_path = os.path.join(sc_liorf_localization_share_dir, 'config', 'robot_livox.urdf.xacro')

    lio_config_file = LaunchConfiguration('lio_config_params_file')

    sc_liorf_localization_parameter_file = PathJoinSubstitution([sc_liorf_localization_share_dir, 'config', lio_config_file]), ".yaml"

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='work2',
        description='Full path to map yaml file to load'
        )

    lio_config_declare = DeclareLaunchArgument(
        'lio_config_params_file',
        default_value='lio_sam_mid360_mapping',
        description='FPath to the ROS2 parameters file to use.')

    bringup_cmd_group = GroupAction([

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        #     parameters=[sc_liorf_localization_parameter_file],
        #     output='screen'
        # ),
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_gain_node',
            output='screen',
            parameters=[
                {'do_bias_estimation': True},
                {'do_adaptive_gain': True},
                {'use_mag': False},
                {'gain_acc': 0.01},
                {'gain_mag': 0.01},
            ],
            remappings=[
                ('/imu/data_raw', '/livox/imu'),
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_imuPreintegration',
            name='sc_liorf_localization_imuPreintegration',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_imageProjection',
            name='sc_liorf_localization_imageProjection',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_featureExtraction',
            name='sc_liorf_localization_featureExtraction',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_rm_localization',
            name='sc_liorf_localization_localization',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
        

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_map_cmd)
    ld.add_action(lio_config_declare)
    ld.add_action(bringup_cmd_group)

    return ld
