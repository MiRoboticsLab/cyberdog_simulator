#!/usr/bin/python3
#
# Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

import launch
import subprocess
import launch_ros.actions
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
# from ament_index_python.packages import get_package_library_directory

def generate_launch_description():
    # Get the launch directory
    library_dir = get_package_share_directory('cyberdog_locomotion')
    cmd_path = os.path.join(library_dir, "../../lib/cyberdog_locomotion")

    # cd 
    # open_cmd = os.path.join("cd ", cmd_path)
    open_cmd = "cd " + cmd_path + " && ./cyberdog_control m s"
    print(open_cmd)

    os.system(open_cmd)

    ld = launch.LaunchDescription()
    return ld

if __name__ == '__main__':
    generate_launch_description()
