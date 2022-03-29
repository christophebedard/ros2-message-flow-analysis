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

"""Launch file for reference_system experiment on a single host."""

import os

import launch
import launch_ros
from tracetools_launch.action import Trace


def generate_launch_description():  # noqa: D103
    length_arg = launch.actions.DeclareLaunchArgument(
        'length',
        default_value='60.0',
        description='length of execution in seconds',
    )

    return launch.LaunchDescription([
        length_arg,
        Trace(
            session_name='reference-system',
            append_timestamp=True,
            base_path=os.path.dirname(os.path.realpath(__file__)),
            events_ust=[
                'dds:*',
                'ros2:*',
            ],
            events_kernel=[],
        ),
        launch_ros.actions.Node(
            package='autoware_reference_system',
            executable='autoware_default_singlethreaded',
        ),
        # Shut down after some time, otherwise the system would run indefinitely
        launch.actions.TimerAction(
            period=launch.substitutions.LaunchConfiguration(length_arg.name),
            actions=[launch.actions.Shutdown(reason='stopping system')],
        ),
    ])
