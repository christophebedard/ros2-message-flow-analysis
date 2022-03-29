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

"""Launch file for system 1 for the reference_system experiment."""

import os

import launch
import launch_ros
from tracetools_launch.action import Trace


def get_launch_description(group_id: int, group_num_nodes: int) -> launch.LaunchDescription:
    """Get reference system launch description given group ID and total number of nodes."""
    length_arg = launch.actions.DeclareLaunchArgument(
        'length',
        default_value='60.0',
        description='length of execution in seconds',
    )

    nodes = [
        launch_ros.actions.Node(
            package='autoware_reference_system',
            executable=f'autoware_default_singlethreaded_{group_id}{node_id}',
        )
        for node_id in [str(chr(97 + ni)) for ni in range(group_num_nodes)]
    ]

    return launch.LaunchDescription([
        length_arg,
        Trace(
            session_name=f'reference-system-{group_id}',
            append_timestamp=True,
            base_path=os.path.dirname(os.path.realpath(__file__)),
            events_ust=[
                'dds:*',
                'ros2:*',
            ],
            events_kernel=[
                'net_dev_queue',
                'net_if_receive_skb',
            ],
        ),
        *nodes,
        # Shut down after some time, otherwise the system would run indefinitely
        launch.actions.TimerAction(
            period=launch.substitutions.LaunchConfiguration(length_arg.name),
            actions=[launch.actions.Shutdown(reason='stopping system')],
        ),
    ])


def generate_launch_description():  # noqa: D103
    return get_launch_description(2, 7)
