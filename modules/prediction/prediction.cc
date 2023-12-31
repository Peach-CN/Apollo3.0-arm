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

#include "modules/prediction/prediction.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "boost/filesystem.hpp"
#include "boost/range/iterator_range.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace prediction {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::DirectoryExists;
using apollo::common::util::Glob;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

std::string Prediction::Name() const { return FLAGS_prediction_module_name; }

void GetBagFiles(const boost::filesystem::path& p,
                 std::vector<std::string>* bag_files) {
  CHECK(bag_files);
  if (!boost::filesystem::exists(p)) {
    return;
  }
  if (boost::filesystem::is_regular_file(p)) {
    const auto ext = p.extension();
    if (ext == ".bag" || ext == ".BAG") {
      AINFO << "Found bag file: " << p.c_str();
      bag_files->push_back(p.c_str());
    }
    return;
  }
  if (boost::filesystem::is_directory(p)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(p), {})) {
      GetBagFiles(entry.path(), bag_files);
    }
  }
}

void Prediction::ProcessRosbag(const std::string& filename) {
  const std::vector<std::string> topics{FLAGS_perception_obstacle_topic,
                                        FLAGS_localization_topic};
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    AERROR << "BagIOException when open bag: " << filename
           << " Exception: " << e.what();
    bag.close();
    return;
  } catch (...) {
    AERROR << "Failed to open bag: " << filename;
    bag.close();
    return;
  }
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (auto it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == FLAGS_localization_topic) {
      OnLocalization(*(it->instantiate<LocalizationEstimate>()));
    } else if (it->getTopic() == FLAGS_perception_obstacle_topic) {
      RunOnce(*(it->instantiate<PerceptionObstacles>()));
    }
  }
  bag.close();
}

Status Prediction::Init() {
  start_time_ = Clock::NowInSeconds();

  // Load prediction conf
  prediction_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_prediction_conf_file,
                                      &prediction_conf_)) {
    return OnError("Unable to load prediction conf file: " +
                   FLAGS_prediction_conf_file);
  } else {
    ADEBUG << "Prediction config file is loaded into: "
           << prediction_conf_.ShortDebugString();
  }

  adapter_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                      &adapter_conf_)) {
    return OnError("Unable to load adapter conf file: " +
                   FLAGS_prediction_adapter_config_filename);
  } else {
    ADEBUG << "Adapter config file is loaded into: "
           << adapter_conf_.ShortDebugString();
  }

  if (FLAGS_enable_use_prediction_module) {
    // Initialization of all managers
    AdapterManager::Init(adapter_conf_);
    ContainerManager::instance()->Init(adapter_conf_);
    EvaluatorManager::instance()->Init(prediction_conf_);
    PredictorManager::instance()->Init(prediction_conf_);

    CHECK(AdapterManager::GetLocalization()) << "Localization is not registered.";
    CHECK(AdapterManager::GetPerceptionObstacles())
        << "Perception is not registered.";

    // Set localization callback function
    AdapterManager::AddLocalizationCallback(&Prediction::OnLocalization, this);
    // Set planning callback function
    AdapterManager::AddPlanningCallback(&Prediction::OnPlanning, this);
    // Set perception obstacle callback function
    AdapterManager::AddPerceptionObstaclesCallback(&Prediction::RunOnce, this);
  } else {
      // Initialization of all managers
    AdapterManager::Init(adapter_conf_);
    ContainerManager::instance()->Init(adapter_conf_);
    CHECK(AdapterManager::GetPerceptionObstacles())
        << "Perception is not registered.";
    AdapterManager::AddPerceptionObstaclesCallback(&Prediction::SetObstacleToPrediction, this);
  }

  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    return OnError("Map cannot be loaded.");
  }

  if (FLAGS_prediction_offline_mode) {
    if (!FeatureOutput::Ready()) {
      return OnError("Feature output is not ready.");
    }
    if (FLAGS_prediction_offline_bags.empty()) {
      return Status::OK();  // use listen to ROS topic mode
    }
    std::vector<std::string> inputs;
    apollo::common::util::split(FLAGS_prediction_offline_bags, ':', &inputs);
    for (const auto& input : inputs) {
      std::vector<std::string> offline_bags;
      GetBagFiles(boost::filesystem::path(input), &offline_bags);
      std::sort(offline_bags.begin(), offline_bags.end());
      AINFO << "For input " << input << ", found " << offline_bags.size()
            << "  rosbags to process";
      for (std::size_t i = 0; i < offline_bags.size(); ++i) {
        AINFO << "\tProcessing: [ " << i << " / " << offline_bags.size()
              << " ]: " << offline_bags[i];
        ProcessRosbag(offline_bags[i]);
      }
    }
    Stop();
    ros::shutdown();
  }
  return Status::OK();
}

Status Prediction::Start() { return Status::OK(); }

void Prediction::Stop() {
  if (FLAGS_prediction_offline_mode) {
    FeatureOutput::Close();
  }
}

void Prediction::OnLocalization(const LocalizationEstimate& localization) {
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  CHECK_NOTNULL(pose_container);
  pose_container->Insert(localization);

  ADEBUG << "Received a localization message ["
         << localization.ShortDebugString() << "].";
}

void Prediction::OnPlanning(const planning::ADCTrajectory& adc_trajectory) {
  ADCTrajectoryContainer* adc_trajectory_container =
      dynamic_cast<ADCTrajectoryContainer*>(
          ContainerManager::instance()->GetContainer(
              AdapterConfig::PLANNING_TRAJECTORY));
  CHECK_NOTNULL(adc_trajectory_container);
  adc_trajectory_container->Insert(adc_trajectory);

  ADEBUG << "Received a planning message [" << adc_trajectory.ShortDebugString()
         << "].";
}

void Prediction::RunOnce(const PerceptionObstacles& perception_obstacles) {
  if (FLAGS_prediction_test_mode && FLAGS_prediction_test_duration > 0.0 &&
      (Clock::NowInSeconds() - start_time_ > FLAGS_prediction_test_duration)) {
    AINFO << "Prediction finished running in test mode";
    ros::shutdown();
  }

  // Update relative map if needed
  AdapterManager::Observe();
  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return;
  }

  double start_timestamp = Clock::NowInSeconds();

  // Insert obstacle
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(obstacles_container);
  obstacles_container->Insert(perception_obstacles);

  ADEBUG << "Received a perception message ["
         << perception_obstacles.ShortDebugString() << "].";

  // Update ADC status
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  ADCTrajectoryContainer* adc_container = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PLANNING_TRAJECTORY));
  CHECK_NOTNULL(pose_container);
  CHECK_NOTNULL(adc_container);

  PerceptionObstacle* adc = pose_container->ToPerceptionObstacle();
  if (adc != nullptr) {
    obstacles_container->InsertPerceptionObstacle(*adc, adc->timestamp());
    double x = adc->position().x();
    double y = adc->position().y();
    ADEBUG << "Get ADC position [" << std::fixed << std::setprecision(6) << x
           << ", " << std::fixed << std::setprecision(6) << y << "].";
    Vec2d adc_position(x, y);
    adc_container->SetPosition(adc_position);
  }

  // Make evaluations
  EvaluatorManager::instance()->Run(perception_obstacles);

  // No prediction for offline mode
  if (FLAGS_prediction_offline_mode) {
    return;
  }

  // Make predictions
  PredictorManager::instance()->Run(perception_obstacles);

  auto prediction_obstacles =
      PredictorManager::instance()->prediction_obstacles();
  prediction_obstacles.set_start_timestamp(start_timestamp);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_obstacles.header().lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_obstacles.header().camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_obstacles.header().radar_timestamp());

  if (FLAGS_prediction_test_mode) {
    for (auto const& prediction_obstacle :
         prediction_obstacles.prediction_obstacle()) {
      for (auto const& trajectory : prediction_obstacle.trajectory()) {
        for (auto const& trajectory_point : trajectory.trajectory_point()) {
          if (!ValidationChecker::ValidTrajectoryPoint(trajectory_point)) {
            AERROR << "Invalid trajectory point ["
                   << trajectory_point.ShortDebugString() << "]";
            return;
          }
        }
      }
    }
  }

  Publish(&prediction_obstacles);
}

Status Prediction::OnError(const std::string& error_msg) {
  return Status(ErrorCode::PREDICTION_ERROR, error_msg);
}

// add for do not use prediction module mode
void Prediction::SetObstacleToPrediction(const PerceptionObstacles& perception_obstacles) {

  double start_timestamp = Clock::NowInSeconds();

  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  ADCTrajectoryContainer* adc_trajectory_container =
      dynamic_cast<ADCTrajectoryContainer*>(
          ContainerManager::instance()->GetContainer(
              AdapterConfig::PLANNING_TRAJECTORY));

  CHECK_NOTNULL(obstacles_container);
  
  auto prediction_obstacles =
    PredictorManager::instance()->prediction_obstacles();

  PredictionObstacle prediction_obstacle;

  for (const auto& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    if (!perception_obstacle.has_id()) {
      AERROR << "A perception obstacle has no id.";
      continue;
    }

    int id = perception_obstacle.id();
    if (id < 0) {
      AERROR << "A perception obstacle has invalid id [" << id << "].";
      continue;
    }
    prediction_obstacle.set_timestamp(perception_obstacle.timestamp());
    prediction_obstacle.set_predicted_period(FLAGS_prediction_duration);
    prediction_obstacle.mutable_perception_obstacle()->CopyFrom(
        perception_obstacle);
    prediction_obstacles.add_prediction_obstacle()->CopyFrom(
        prediction_obstacle);
    prediction_obstacles.set_start_timestamp(start_timestamp);
    prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
    prediction_obstacles.mutable_header()->set_lidar_timestamp(
        perception_obstacles.header().lidar_timestamp());
    prediction_obstacles.mutable_header()->set_camera_timestamp(
        perception_obstacles.header().camera_timestamp());
    prediction_obstacles.mutable_header()->set_radar_timestamp(
        perception_obstacles.header().radar_timestamp());
    
  }
  prediction_obstacles.set_perception_error_code(
      perception_obstacles.error_code());

  Publish(&prediction_obstacles);
}
}  // namespace prediction
}  // namespace apollo
