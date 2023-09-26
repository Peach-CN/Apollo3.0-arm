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

/**
 * @file
 */

#ifndef MODULES_DREAMVIEW_BACKEND_MAP_MAP_GENERATOR_H_
#define MODULES_DREAMVIEW_BACKEND_MAP_MAP_GENERATOR_H_

#include <string>
#include <vector>
#include <atomic>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/localization/proto/localization.pb.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

class MapGenerator {
 public:
  explicit MapGenerator();

  // Reload map from current FLAGS_map_dir.
  bool Start();
  void OnFusionLocalization(const apollo::localization::LocalizationEstimate &message);

  bool OnStartRecordCommand(std::string mapName);
  std::string OnStopRecordCommand();

 private:
  bool OnGenerateNavigationMap(std::string& traj);
  bool OnGenerateRoutingMap(std::string& map_dir_name,
                            std::string& traj);
  void CreateDefaultEndWayPoint(std::string& map_dir_name);
  void UpdateNavigationDefaultYaml(std::string& smooth_name);
  apollo::common::monitor::MonitorLogger monitor_logger_;

  std::string map_name_;
  double x_offset_ = 0.0;
  double y_offset_ = 0.0;
  std::vector<apollo::common::PointENU> trajectory_;
  std::atomic<bool> recording_map_;
  // RW lock to protect map data
  mutable boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_MAP_MAP_GENERATOR_H_
