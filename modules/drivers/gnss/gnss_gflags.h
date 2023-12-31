/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_DRIVERS_GNSS_GNSS_GFLAGS_H_
#define MODULES_DRIVERS_GNSS_GNSS_GFLAGS_H_

#include "gflags/gflags.h"

// System gflags
DECLARE_string(node_name);
DECLARE_string(adapter_config_filename);

// Config file
DECLARE_string(sensor_conf_file);

// System gflags
DECLARE_string(sensor_node_name);

DECLARE_string(gpsbin_folder);

DECLARE_bool(indoor_localization);

DECLARE_int32(indoor_velocity_update);

DECLARE_bool(indoor_useposimu);

DECLARE_bool(use_100d2imu);

DECLARE_bool(use_100d2imu_euler);
#endif
