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

# TODO(all): This is just an initial commit. Velodyne 64 and 16 share lots of
# things. We need to select their processes precisely in 'pgrep', 'pkill' and
# the monitor module.
function start() {
	/apollo/modules/tools/extrinsics_broadcaster/extrinsics_broadcaster.sh publish /apollo/velodyne16_novatel_extrinsics_example.yaml &
    LOG="${APOLLO_ROOT_DIR}/data/log/lslidar_c16.out"
    CMD="roslaunch lslidar_c16_decoder lslidar_c16.launch"
    NUM_PROCESSES="$(pgrep -c -f "lslidar_c16.launch")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
	echo ${NUM_PROCESSES}
}

function stop() {
    pkill -SIGTERM -f lslidar_c16.launch
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
