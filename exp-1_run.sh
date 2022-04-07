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

# Autoware reference system experiment
# Run with the system ID (1 or 2) as an argument

ws="exp-1_ws"

if [ ! -d "${ws}" ]; then
    echo "Error: make sure to setup workspace: exp-1_setup_workspace.sh"
    exit 1
fi

source ${ws}/install/setup.bash

if [ "$#" -eq "0" ]; then
    launch_file="reference_system.launch.py"
else
    system_id="$1"
    launch_file="reference_system_${system_id}.launch.py"
fi

echo "Launching: ${launch_file}"
ros2 launch ${launch_file}
