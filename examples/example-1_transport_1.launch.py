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

"""Launch file for host 1 of 2 of a multi-host sync 1-to-1 ping pong case."""

import os

import launch
import launch_ros
from tracetools_launch.action import Trace


def generate_launch_description():
    length_arg = launch.actions.DeclareLaunchArgument(
        'length',
        default_value='10.0',
        description='length of execution in seconds',
    )
    return launch.LaunchDescription([
        length_arg,
        Trace(
            session_name='trace-example-1-a',
            append_timestamp=True,
            base_path=os.path.dirname(os.path.realpath(__file__)),
            events_ust=[
                'dds:*',
                'ros2:*',
            ],
            # For trace synchronization
            events_kernel=[
                'net_dev_queue',
                'net_if_receive_skb',
            ],
        ),
        launch_ros.actions.Node(
            package='ros2_message_flow_testcases',
            executable='source',
            arguments=['a', '5'],
            output='screen',
        ),
        # Shut down after some time, otherwise the system would run indefinitely
        launch.actions.TimerAction(
            period=launch.substitutions.LaunchConfiguration(length_arg.name),
            actions=[launch.actions.Shutdown(reason='stopping system')],
        ),
    ])
