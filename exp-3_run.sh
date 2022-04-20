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

# Overhead evaluation experiment

ws_notracing="exp-3_ws"
ws_tracing="exp-1_ws"
launch_file_notracing="end_to_end_no-tracing.launch.py"
launch_file_tracing="end_to_end_tracing.launch.py"

if [ ! -d "${ws_notracing}" ]; then
    echo "Error: make sure to setup workspace: exp-3_setup_workspace.sh"
    exit 1
fi
if [ ! -d "${ws_tracing}" ]; then
    echo "Error: make sure to setup workspace: exp-1_setup_workspace.sh"
    exit 1
fi

echo "Launching: ${launch_file_notracing}"
( source ${ws_notracing}/install/setup.bash; cd overhead/; ros2 launch ${launch_file_notracing} )
echo ""
echo "Launching: ${launch_file_tracing}"
( source ${ws_tracing}/install/setup.bash; cd overhead/; ros2 launch ${launch_file_tracing} )
