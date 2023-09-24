#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
	db_proxy = launch_ros.actions.Node(package='wili_db', executable='db_proxy')
	socket_bridge = launch_ros.actions.Node(package='wili_bridge', executable='socket_bridge', output='screen')

	return launch.LaunchDescription([db_proxy, socket_bridge])