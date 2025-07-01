#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_status"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='uav_status_acquisition'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ uav_type

    uav_type = LaunchConfiguration('uav_type')

    ld.add_action(DeclareLaunchArgument(
        'uav_type',
        default_value=os.getenv('UAV_TYPE', ""),
        description="The uav type.",
    ))

    # #} end of custom_config

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ platform_config

    platform_config = LaunchConfiguration('platform_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value="",
        description="Path to the platform configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     platform_config == "" => platform_config: ""
    #     platform_config == "/<path>" => platform_config: "/<path>"
    #     platform_config == "<path>" => platform_config: "$(pwd)/<path>"
    platform_config = IfElseSubstitution(
            condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
            else_value=platform_config
            )

    # #} end of platform_config

    # #{ env-based params

    run_type=os.getenv('RUN_TYPE', "realworld")

    if run_type == "simulation":
        simulation = True
    else:
        simulation = False

    # #} end of env-based params

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ status_acquisition_node

    status_acquisition_node = ComposableNode(

        package=pkg_name,
        plugin='mrs_uav_status::Acquisition',
        namespace=uav_name,
        name='uav_status_acquisition',

        parameters=[
            {"uav_name": uav_name},
            {"uav_type": uav_type},
            {"enable_profiler": False},
            {"use_sim_time": use_sim_time},
            {'config_public': this_pkg_path + '/config/public/default.yaml'},
            {'platform_config': platform_config},
            {'custom_config': custom_config},
            ],

        remappings=[
            # subscribers
            ("~/uav_state_in", "estimation_manager/uav_state"),
            ("~/cmd_tracker_in", "control_manager/tracker_cmd"),
            ("~/estimation_diag_in", "estimation_manager/diagnostics"),
            ("~/mpc_diag_in", "control_manager/mpc_tracker/diagnostics"),
            ("~/hw_api_status_in", "hw_api/status"),
            ("~/cmd_attitude_in", "control_manager/attitude_cmd"),
            ("~/battery_in", "hw_api/battery_state"),
            ("~/control_manager_in", "control_manager/diagnostics"),
            ("~/gain_manager_in", "gain_manager/diagnostics"),
            ("~/constraint_manager_in", "constraint_manager/diagnostics"),
            ("~/tf_static_in", "/tf_static"),
            ("~/gnss_in", "hw_api/gnss"),
            ("~/gnss_status_in", "hw_api/gnss_status"),
            ("~/odometry_in", "hw_api/odometry"),
            ("~/automatic_start_in", "automatic_start/can_takeoff"),
            ("~/throttle_in", "control_manager/throttle"),
            ("~/mass_estimate_in", "control_manager/mass_estimate"),
            ("~/mass_set_in", "control_manager/mass_nominal"),
            ("~/mag_in", "hw_api/magnetic_field"),
            ("~/string_in", "~/display_string"),
            # publishers
            ("~/uav_status_out", "~/uav_status"),
            ("~/uav_status_short_out", "~/uav_status_short"),
            ("~/profiler", "profiler"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[status_acquisition_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of status_acquisition_node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[status_acquisition_node],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of own container

    return ld
