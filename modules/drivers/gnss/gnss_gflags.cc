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

#include "modules/drivers/gnss/gnss_gflags.h"

// System gflags
DEFINE_string(node_name, "gnss", "The gnss driver module name");

DEFINE_string(adapter_config_filename, "modules/drivers/gnss/conf/adapter.conf",
              "The adapter config file");
// Config file
DEFINE_string(sensor_conf_file, "modules/drivers/gnss/conf/gnss_conf.pb.txt",
              "Sensor conf file");

// System gflags
DEFINE_string(sensor_node_name, "gnss", "Sensor node name.");

DEFINE_string(gpsbin_folder, "/apollo/data/gpsbin",
              "gpsbin rawdata folder name.");

DEFINE_bool(indoor_localization, false,
              "indoor localization like marvelmind");

DEFINE_int32(indoor_velocity_update, 10,
              "indoor velocity update by marvelmind Pos");

DEFINE_bool(indoor_useposimu, false,
              "indoor imu use marvelmind not 100d2");

DEFINE_bool(use_100d2imu, false,
	"when we used newtonm2,we can use imu100d2 for angular_velocity and linear_acceleration and euler_angles");

DEFINE_bool(use_100d2imu_euler, false,
	"if we used newtonm2 and imu100d2,we can choose to use 100d2 or newtonm2 for euler");
