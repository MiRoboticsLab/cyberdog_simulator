#!/usr/bin/env python
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

import sys, signal, subprocess, time


timeout_before_kill = 1.0  # [s]
timeout_after_kill = 1.0  # [s]


def signal_handler(sig, frame):
    print('killing gazebo')
    time.sleep(timeout_before_kill)
    subprocess.call("killall -q gzclient & killall -q gzserver", shell=True)
    time.sleep(timeout_after_kill)
    subprocess.call("killall -9 -q gzclient & killall -9 -q gzserver", shell=True)
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    cmd = ' '.join(sys.argv[1:])
    print(cmd)
    subprocess.call(cmd, shell=True)