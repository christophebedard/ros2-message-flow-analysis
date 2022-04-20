#!/usr/bin/env bash
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

# Workspace setup script for the ROS 2 message flow analysis overhead evaluation experiment
#
# See README for the full instructions.

ws="exp-3_ws"

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# If workspace doesn't exist, create it;
# otherwise, pull all repos
if [ ! -d "${ws}" ]; then
    mkdir -p ${ws}/src
    cd ${ws}
    vcs import src --input https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
    # Use non-default branches/forks for some of the repos in the ros2.repos file
    vcs import src --input "${SCRIPT_DIR}/tracing_message_flow.repos" --force
    # Import additional repos for this experiment
    vcs import src --input "${SCRIPT_DIR}/exp-1.repos" --force
else
    cd ${ws}
    vcs pull src
fi

# Build and disable tracing
colcon build --packages-up-to ros2_message_flow_testcases autoware_reference_system --mixin release --cmake-args " -DTRACETOOLS_DISABLED=ON"
