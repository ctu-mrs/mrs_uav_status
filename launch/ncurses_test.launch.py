#!/usr/bin/env python3

import launch

import launch_ros

import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_status"

    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    ld.add_action(launch_ros.actions.Node(

        package=pkg_name,
        executable='MrsUavStatus_NcursesTest',
        namespace="",
        name='ncurses_test',
        output="screen",
        emulate_tty=True,
        env=proc_env,
        prefix='gnome-terminal --',
        )
    )

    return ld
