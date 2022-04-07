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

"""Launch file for the periodic async N-to-M message link type."""

import os

import launch
import launch_ros
from tracetools_launch.action import Trace


def generate_launch_description():
    length_arg = launch.actions.DeclareLaunchArgument(
        'length',
        default_value='30.0',
        description='length of execution in seconds',
    )
    return launch.LaunchDescription([
        Trace(
            session_name='example-3_single_periodic_async_n-to-m',
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
            executable='source',
            arguments=['a', '5'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='source',
            arguments=['b', '12'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='periodic_async_n_to_m',
            arguments=['ab', 'cd', '8'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='sink',
            arguments=['cd'],
            output='screen',
        ),
        # Shut down after some time, otherwise the system would run indefinitely
        launch.actions.TimerAction(
            period=launch.substitutions.LaunchConfiguration(length_arg.name),
            actions=[launch.actions.Shutdown(reason='stopping system')],
        ),
    ])
