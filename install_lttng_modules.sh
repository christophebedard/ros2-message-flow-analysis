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

# Build and install LTTng-modules (kernel tracer) against a custom kernel
# See: https://github.com/lttng/lttng-modules

# The LTTng version
LTTNG_VERSION="2.11"
# The directory containing the full source tree of the custom kernel
KERNEL_DIRECTORY="/home/chris/kernel/linux-5.4.3/"
# The actual custom kernel version, i.e., `uname -r`
KERNEL_VERSION="5.4.3-rt1"

# Clone
cd ~/
git clone https://github.com/lttng/lttng-modules.git --branch stable-${LTTNG_VERSION}
cd lttng-modules/

# Build and install
export KERNELDIR="${KERNEL_DIRECTORY}"
make
sudo make modules_install
sudo depmod -a "${KERNEL_VERSION}"
