# Copyright 2022 Christophe Bedard
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for 1-to-1 pipeline with end-to-end timestamps (tracing)."""

import os

import launch
import launch_ros
from tracetools_launch.action import Trace


def generate_launch_description():
    length_arg = launch.actions.DeclareLaunchArgument(
        'length',
        default_value='1200.0',
        description='length of execution in seconds',
    )
    return launch.LaunchDescription([
        length_arg,
        Trace(
            session_name='end_to_end_overhead_tracing',
            append_timestamp=True,
            base_path=os.path.dirname(os.path.realpath(__file__)),
            events_kernel=[],
            events_ust=[
                'dds:*',
                'ros2:*',
            ],
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='source_t',
            arguments=['a', '100'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='sync_one_to_one_t',
            arguments=['a', 'b', '60'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='sync_one_to_one_t',
            arguments=['b', 'c', '80'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='sync_one_to_one_t',
            arguments=['c', 'd', '70'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='sync_one_to_one_t',
            arguments=['d', 'e', '50'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='sink_t',
            arguments=['e', 'latencies_tracing'],
            output='screen',
        ),
        # Shut down after some time, otherwise the system would run indefinitely
        launch.actions.TimerAction(
            period=launch.substitutions.LaunchConfiguration(length_arg.name),
            actions=[launch.actions.Shutdown(reason='stopping system')],
        ),
    ])
