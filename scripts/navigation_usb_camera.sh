#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
    LOG="${APOLLO_ROOT_DIR}/data/log/usb_cam.out"
#    CMD="roslaunch usb_cam start_obstacle_camera.launch"
#    CMD="roslaunch zed_wrapper zed.launch"
    CMD="${APOLLO_ROOT_DIR}/modules/drivers/zkhy/src/Bin/StereoCamera"
#    NUM_PROCESSES="$(pgrep -c -f "camera_nodelet_manager")"
#    NUM_PROCESSES="$(pgrep -c -f "zed_wrapper_node")"
    NUM_PROCESSES="$(pgrep -c -f 'StereoCamera')"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
#    pkill -9 -f start_obstacle_camera
#    pkill -9 -f camera_nodelet_manager
#    pkill -9 -f zed_wrapper_node
    pkill -9 -f StereoCamera
    pkill -9 -f robot_state_publisher
}

# run command_name module_name
function run() {
    case $1 in
        start)
            start
            ;;
        stop)
            stop
            ;;
        *)
            start
            ;;
    esac
}

run "$1"
