#!/usr/bin/env python3

import launch
import os
import sys

import launch_ros
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
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
    namespace='uav_status_tui'

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

    # #{ rainbow

    rainbow = LaunchConfiguration('rainbow')

    ld.add_action(DeclareLaunchArgument(
        'rainbow',
        default_value="false",
        description="",
    ))

    # #} end of custom_config

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
    colorscheme=os.getenv('PROFILES', "COLORSCHEME_LIGHT")

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

    ld.add_action(launch_ros.actions.Node(

        package=pkg_name,
        executable='MrsUavStatus_Status',
        namespace=uav_name,
        name='uav_status_tui',
        output="screen",
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

        parameters=[
            {"uav_name": uav_name},
            {"pwd": EnvironmentVariable('PWD')},
            {"colorscheme": colorscheme},
            {"rainbox": rainbow},
            {"enable_profiler": False},
            {"use_sim_time": use_sim_time},
            {'config_public': this_pkg_path + '/config/public/default.yaml'},
            {'platform_config': platform_config},
            {'custom_config': custom_config},
            ],

        remappings=[
            # subscribers
            ("~/uav_status_in", "uav_status_acquisition/uav_status"),
            ("~/uav_status_short_in", "uav_status_acquisition/uav_status_short"),
            # publishers
            ("~/gimbal_command_out", "tarot_gimbal/gimbal_command"),
            # services
            ("~/reference_out", "control_manager/reference"),
            ("~/trajectory_reference_out", "control_manager/trajectory_reference"),
            ("~/set_constraints_out", "constraint_manager/set_constraints"),
            ("~/set_estimator_out", "estimation_manager/change_estimator"),
            ("~/set_gains_out", "gain_manager/set_gains"),
            ("~/set_controller_out", "control_manager/switch_controller"),
            ("~/set_tracker_out", "control_manager/switch_tracker"),
            ("~/hover_out", "control_manager/hover"),
            ("~/profiler", "profiler"),
        ],
        )
    )

    return ld
