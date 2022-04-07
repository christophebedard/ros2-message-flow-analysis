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

# Examples using ros2_message_flow_testcases
# * Run without arguments to run the single-system examples
# * Run with '1' or '2' as an argument to run the distributed system example on system 1 and 2

ws="exp-1_ws"

if [ ! -d "${ws}" ]; then
    echo "Error: make sure to setup workspace: exp-1_setup_workspace.sh"
    exit 1
fi

source ${ws}/install/setup.bash

cd examples/
if [ "$#" -eq "0" ]; then
    # Run launch files one at a time, they should exit by themselves
    ros2 launch example-2_trivial.launch.py
    ros2 launch example-3_periodic_async.launch.py
    ros2 launch example-4_partial_sync.launch.py
else
    # Run distributed example
    system_id="$1"
    launch_file="example-1_transport_${system_id}.launch.py"
    ros2 launch ${launch_file}
fi
