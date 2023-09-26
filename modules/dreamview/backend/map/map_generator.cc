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

#include "modules/dreamview/backend/map/map_generator.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/string_util.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/proto/poi.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"
#include "modules/planning/proto/reference_line_smoother_config.pb.h"

DEFINE_double(smooth_length, 200.0, "Smooth this amount of length ");
DEFINE_double(minimum_point_spacing, 5.0,
    "The minimum distance for input points.");
	
namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::PointENU;
using apollo::common::util::JsonUtil;
using apollo::hdmap::ClearAreaInfoConstPtr;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::Id;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::Map;
using apollo::hdmap::Path;
using apollo::hdmap::PncMap;
using apollo::hdmap::RoadInfoConstPtr;
using apollo::hdmap::RouteSegments;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::hdmap::SimMapFile;
using apollo::hdmap::StopSignInfoConstPtr;
using apollo::hdmap::YieldSignInfoConstPtr;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using google::protobuf::RepeatedPtrField;
using apollo::common::util::GetProtoFromASCIIFile;
using apollo::planning::SpiralReferenceLineSmoother;
using apollo::planning::ReferenceLineSmootherConfig;

namespace planning {

class SpiralSmootherUtil {
 public:
  static std::vector<Eigen::Vector2d>
  ReadCoordinatesFromFile(const std::string& filename) {
    std::vector<Eigen::Vector2d> raw_points;
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    std::string point_str;

    auto spacing_thres = FLAGS_minimum_point_spacing *
        FLAGS_minimum_point_spacing;

    while (std::getline(ifs, point_str)) {
      std::size_t idx = point_str.find(',');
      if (idx == std::string::npos) {
        continue;
      }
      auto x_str = point_str.substr(0, idx);
      auto y_str = point_str.substr(idx + 1);

      auto x = std::stod(x_str);
      auto y = std::stod(y_str);

      if (raw_points.size() > 0) {
        auto last_x = raw_points.back().x();
        auto last_y = raw_points.back().y();

        auto dx = x - last_x;
        auto dy = y - last_y;
        if (dx * dx + dy * dy < spacing_thres) {
          continue;
        }
      }
      raw_points.emplace_back(x, y);
    }
    return raw_points;
  }

  static bool Smooth(std::vector<Eigen::Vector2d> raw_points,
      std::vector<common::PathPoint>* ptr_smooth_points) {
    if (raw_points.size() <= 2) {
      AERROR << "the original point size is " << raw_points.size();
      return false;
    }

    Eigen::Vector2d start_point = raw_points.front();
    std::for_each(raw_points.begin(), raw_points.end(),
        [&start_point](Eigen::Vector2d& p) {
          p = p - start_point;});

    ReferenceLineSmootherConfig config;
    CHECK(common::util::GetProtoFromFile(
        "/apollo/modules/planning/conf/spiral_smoother_config.pb.txt", &config));

    std::vector<double> opt_theta;
    std::vector<double> opt_kappa;
    std::vector<double> opt_dkappa;
    std::vector<double> opt_s;
    std::vector<double> opt_x;
    std::vector<double> opt_y;

    SpiralReferenceLineSmoother spiral_smoother(config);
    auto res = spiral_smoother.SmoothStandAlone(raw_points,
        &opt_theta, &opt_kappa, &opt_dkappa, &opt_s, &opt_x, &opt_y);

    if (!res) {
      AWARN << "Optimization failed; the result may not be smooth";
    } else {
      AINFO << "Optimal solution found";
    }

    std::for_each(opt_x.begin(), opt_x.end(),
        [&start_point](double& x) { x += start_point.x();});
    std::for_each(opt_y.begin(), opt_y.end(),
        [&start_point](double& y) { y += start_point.y();});

    *ptr_smooth_points =
    spiral_smoother.Interpolate(opt_theta, opt_kappa, opt_dkappa,
        opt_s, opt_x, opt_y, config.resolution());

    return true;
  }

  static void Export(const std::string& filename,
      const std::vector<common::PathPoint>& smoothed_points) {
    std::ofstream ofs(filename.c_str());
    if (ofs.fail()) {
      AERROR << "Fail to open file " << filename;
      return;
    }
    ofs.precision(12);
    // skip the first point and the last point
    for (std::size_t i = 1; i + 1 < smoothed_points.size(); ++i) {
      const auto& point = smoothed_points[i];
      ofs << std::fixed << "{\"kappa\": " << point.kappa()
          << ", \"s\": " << point.s()
          << ", \"theta\": " << point.theta() << ", \"x\":" << point.x()
          << ", \"y\":" << point.y() << ", \"dkappa\":" << point.dkappa() << "}"
          << std::endl;
    }
    ofs.close();
    AINFO << "Smoothed result saved to " << filename;
  }
};

}  // namespace planning

MapGenerator::MapGenerator() 
  : recording_map_(false)
  , monitor_logger_(apollo::common::monitor::MonitorMessageItem::HMI)  {

}

bool MapGenerator::OnStartRecordCommand(std::string mapName) {
  
  std::time_t ctime = std::time(0);
  char sz_time[256] = {0};
  std::strftime(sz_time, sizeof(sz_time), "%Y-%m-%d-%H-%M-%S", std::localtime(&ctime));

  map_name_ = mapName;//sz_time;
  
  recording_map_ = true;
  
  return true;
}

bool MapGenerator::Start() {
  //boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  // Fusion localization
  if (!AdapterManager::GetLocalization()) {
    buffer.ERROR(
        "Localization input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return false;
  }
  AdapterManager::AddLocalizationCallback(
      &MapGenerator::OnFusionLocalization, this);

  return true;
}

void MapGenerator::OnFusionLocalization(
        const apollo::localization::LocalizationEstimate &message) {
  if (recording_map_) {
    if (std::fabs(x_offset_ - message.pose().position().x()) < 1e-6 &&
        std::fabs(y_offset_ - message.pose().position().y()) < 1e-6)
      return ;
    trajectory_.push_back(message.pose().position());
    x_offset_ = message.pose().position().x();
    y_offset_ = message.pose().position().y();
	if (trajectory_.size() % 100 == 0) {
      common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
      std::stringstream msg;

      msg << "recording [" << trajectory_.size() << "] localization point";
      buffer.INFO(msg.str());
    }
  }
}

bool MapGenerator::OnGenerateRoutingMap(
                   std::string& map_dir_name, std::string& traj) {
#if 1
  std::string command;
  char line[2048] = {0};
  command = "python /apollo/modules/tools/map_gen/map_gen_single_lane.py ";
  command+= traj;
  command+= " " + map_dir_name + "/base_map.txt 0";

  FILE *fp = popen( command.c_str(), "r");
  while (fgets(line, sizeof line, fp)) {AERROR << line;}
  pclose(fp);
  
  command = "/apollo/bazel-bin/modules/map/tools/sim_map_generator --map_dir=";
  command+= map_dir_name;
  command+= " --output_dir=";
  command+= map_dir_name;

  fp = popen(command.c_str(), "r");
  while (fgets(line, sizeof line, fp)) {AERROR << line;}
  pclose(fp);

  command = "/apollo/bazel-bin/modules/routing/topo_creator/topo_creator \
		   --flagfile=/apollo/modules/routing/conf/routing.conf --map_dir=";
  command+= map_dir_name;
  fp = popen(command.c_str(), "r");
  while (fgets(line, sizeof line, fp)) {AERROR << line;}
  pclose(fp);
#endif

  return true;
}

bool MapGenerator::OnGenerateNavigationMap(std::string& traj) {
#if 1
  std::string command;
  char line[2048] = {0};

  command = "/apollo/bazel-bin/modules/planning/reference_line/spiral_smoother_util \
						--input_file ";
  command+= traj + " --smooth_length 200";

  FILE *fp = popen( command.c_str(), "r");
  while (fgets(line, sizeof line, fp)) {AERROR << line;}
  pclose(fp);

  traj = traj + ".smoothed";
#else
  std::vector<Eigen::Vector2d> raw_points =
      planning::SpiralSmootherUtil::ReadCoordinatesFromFile(traj);

  std::vector<apollo::common::PathPoint> smooth_points;
  auto res = planning::SpiralSmootherUtil::Smooth(
      raw_points, &smooth_points);
  if (!res) {
    AERROR << "Failed to smooth a the line";
  }

  std::string output_file = traj + ".smoothed";

  planning::SpiralSmootherUtil::Export(output_file, smooth_points);

  traj = output_file;
#endif
  return true;
}
  
std::string MapGenerator::OnStopRecordCommand() {
  std::string map_dir_name, map_path;
  std::ofstream map_stream;
  Status retval = Status::OK();
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  recording_map_ = false;

  if (map_name_.empty() || trajectory_.size() < 100) {
    buffer.ERROR("no trajectory was recorded. please try to record again.");
    buffer.PrintLog();
    map_name_.clear();
    trajectory_.clear();

    return "";
  }

  if (FLAGS_use_navigation_mode) {
    map_dir_name = "/apollo/modules/tools/navigator";
  } else {
	if (!map_name_.empty())
      map_dir_name = std::string("/apollo/modules/map/data/") + map_name_;
  }

  while (!map_dir_name.empty()) {
    apollo::common::util::EnsureDirectory(map_dir_name);
	std::string extract_path = map_dir_name + "/" + map_name_ + ".txt";

	map_stream.open(extract_path, std::ios_base::trunc | std::ios_base::out);

	if (!map_stream.is_open()) {
      buffer.ERROR("Open file to record trajectory failed. Maybe the disk is full.");
      buffer.PrintLog();
	  break;
    }

    for (auto &v : trajectory_) {
      map_stream << std::setprecision(15) << v.x() << "," 
                 << std::setprecision(15) << v.y() << std::endl;
    }
    map_stream.close();

	if (FLAGS_use_navigation_mode) {
      OnGenerateNavigationMap(extract_path);
	  map_path = extract_path;
	  UpdateNavigationDefaultYaml(map_path);
	} else {
	  OnGenerateRoutingMap(map_dir_name, extract_path);
	  CreateDefaultEndWayPoint(map_dir_name);
      map_path = map_dir_name;
    }

	std::stringstream msg;
    msg << "map [" << map_path << "] make successfully!";
    buffer.INFO(msg.str());
	break;
  }

  map_name_.clear();
  trajectory_.clear();

  if (retval == Status::OK())
    return map_path;
  else
    return "";
}

void MapGenerator::CreateDefaultEndWayPoint(std::string& map_dir_name) {
  apollo::routing::POI poi;
  std::string default_end_way_point = map_dir_name + "/default_end_way_point.txt";

  if (trajectory_.size() > 0) {
    apollo::routing::Landmark* new_landmark = poi.add_landmark();
    new_landmark->set_name("1_-1");
    auto new_end_point = new_landmark->add_waypoint();
    auto &point_enu = trajectory_[trajectory_.size() - 1];
    // Update default end way point.

    new_end_point->set_id("1_-1");
    new_end_point->set_s(100.0);
    auto* pose = new_end_point->mutable_pose();
    pose->set_x(point_enu.x());
    pose->set_y(point_enu.y());
  }
  CHECK(apollo::common::util::SetProtoToASCIIFile(poi, default_end_way_point));
}

void MapGenerator::UpdateNavigationDefaultYaml(std::string& smooth_name) {
  //modules/map/relative_map/conf/navigation_path.yaml
  std::string base_yaml_path = "/apollo/modules/map/relative_map/conf/";
  std::string yaml_path = base_yaml_path + \
      apollo::common::util::GetFileName(smooth_name) + ".yaml";
  std::ofstream yaml_stream;
  
  yaml_stream.open(yaml_path);
  if (yaml_stream.is_open()) {
    YAML::Node path_root;

    path_root["navigation_path"]["name"] = smooth_name;
    path_root["path_left_width"]["left_width"] = 1.5;
    path_root["path_right_width"]["right_width"] = 1.5;
AERROR << path_root;
    yaml_stream << path_root;
    yaml_stream.close();

    if (apollo::common::util::PathExists(
          "/apollo/modules/map/relative_map/conf/relative_map.conf")) {
	  std::ofstream relative_map_conf_stream;
	  relative_map_conf_stream.open(
          "/apollo/modules/map/relative_map/conf/relative_map.conf",
          std::ios::app);
      relative_map_conf_stream << "\n--relative_map_navigation_path_filename="
                               << yaml_path << "\n";
	  relative_map_conf_stream.close();
    }
  }
}

}  // namespace dreamview
}  // namespace apollo
