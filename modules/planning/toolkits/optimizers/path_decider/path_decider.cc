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
 **/

#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

PathDecider::PathDecider() : Task("PathDecider") {}

apollo::common::Status PathDecider::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(const PathData &path_data,
                            PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);
  if (!MakeObjectDecision(path_data, path_decision)) {
    AERROR << "Failed to make decision based on tunnel";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph decision ");
  }
  return Status::OK();
}

bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  const auto &frenet_points = frenet_path.points();
  if (frenet_points.empty()) {
    AERROR << "Path is empty.";
    return false;
  }

  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;

  const double lateral_stop_radius =
      half_width + FLAGS_static_decision_nudge_l_buffer;

  std::list<ObsInfo> obs_list;

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto &obstacle = *path_obstacle->obstacle();
    bool is_bycycle_or_pedestrain =
        (obstacle.Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         obstacle.Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);

    if (!is_bycycle_or_pedestrain && !obstacle.IsStatic()) {
      continue;
    }

    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_ignore() &&
        path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    if (path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_sidepass()) {
      // SIDE_PASS decision
      continue;
    }

    if (path_obstacle->reference_line_st_boundary().boundary_type() ==
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();
#if 0
    ObsInfo obs_info;
    obs_info.id = obstacle.Id();
    obs_info.end_l = sl_boundary.end_l();
    obs_info.start_l = sl_boundary.start_l();
    obs_info.start_s = sl_boundary.start_s();
    obs_info.end_s = sl_boundary.end_s();
    obs_info.obstacle = *path_obstacle;
    InsertObsInfo(obs_list, obs_info);
#endif
    if (sl_boundary.end_s() < frenet_points.front().s() ||
        sl_boundary.start_s() > frenet_points.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle.Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle.Id(),
                                        object_decision);
      continue;
    }

    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l();
#if 1
    const double curr_s = frenet_point.s();
    ObsInfo obs_info;
    obs_info.id = obstacle.Id();
    obs_info.end_l = sl_boundary.end_l();
    obs_info.start_l = sl_boundary.start_l();
    obs_info.start_s = sl_boundary.start_s();
    obs_info.end_s = sl_boundary.end_s();
    obs_info.obstacle = *path_obstacle;
    //InsertObsInfo(obs_list, obs_info, curr_s);
#endif
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // ignore
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle.Id(),
                                        object_decision);
    } else if (curr_l - lateral_stop_radius < sl_boundary.end_l() &&
               curr_l + lateral_stop_radius > sl_boundary.start_l()) {
      // stop
      *object_decision.mutable_stop() =
          GenerateObjectStopDecision(*path_obstacle);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle.Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle.Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle.Id(), object_decision);
      }
    } else if (FLAGS_enable_nudge_decision) {
      // nudge
      if (curr_l - lateral_stop_radius > sl_boundary.end_l()) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle.Id(), object_decision);
      } else {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle.Id(), object_decision);
      }
    }
  }
  PathObstacle obstacle; 
  double max_width = GetMaxWidthBwteenObs(obs_list, &obstacle);
  if (max_width <= (half_width * 2 + FLAGS_static_decision_nudge_l_buffer)) {
    ObjectDecisionType object_decision;
    *object_decision.mutable_stop() =
          GenerateObjectStopDecision(obstacle);
    path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle.Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary());
    path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle.Id(), object_decision);
  }

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const PathObstacle &path_obstacle) const {
  ObjectStop object_stop;

  double stop_distance = path_obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param());
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance);

  const double stop_ref_s =
      path_obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

constexpr double kIgnoreObstacleDistance = 5.0;
/*void PathDecider::InsertObsInfo(std::list<ObsInfo>& obs_list, const ObsInfo& obs_info, double curr_s) {
  std::list<ObsInfo>::iterator it = obs_list.begin();
  if (obs_info.start_s - curr_s > kIgnoreObstacleDistance) return ;
  for (; it !=  obs_list.end(); it++ ) {
    if (obs_info.start_l <= it->start_l) {
      obs_list.insert(it, obs_info);
      return;
    }
  }
  obs_list.emplace_back(obs_info);
} */

double PathDecider::GetMaxWidthBwteenObs(const std::list<ObsInfo> &obs_list, PathObstacle *obstacle) {
  double max_space =0.0;
  if (obs_list.size() <= 1) {
    max_space = std::numeric_limits<double>::max();
    return max_space;
  }
  std::list<ObsInfo>::const_iterator it = obs_list.begin();
  double pre_end_l = it->end_l;
  for (; it !=  obs_list.end(); ++it) {
    if (it->start_l - pre_end_l > max_space) {
      max_space = it->start_l - pre_end_l;
      *obstacle = (it->obstacle);
    } 
    pre_end_l = it->end_l;
  }
  return max_space;
}

}  // namespace planning
}  // namespace apollo
