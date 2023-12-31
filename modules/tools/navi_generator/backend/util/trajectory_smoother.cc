/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @brief This file provides the implementation of the class
 * "TrajectorySmoother".
 */
#include "modules/tools/navi_generator/backend/util/trajectory_smoother.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"
#include "modules/tools/navi_generator/backend/util/file_operator.h"

namespace apollo {
namespace navi_generator {
namespace util {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;
using apollo::hdmap::MapPathPoint;
using apollo::planning::AnchorPoint;
using apollo::planning::QpSplineReferenceLineSmoother;
using apollo::planning::ReferenceLine;
using apollo::planning::ReferenceLineSmoother;
using apollo::planning::ReferencePoint;
using apollo::planning::SpiralReferenceLineSmoother;

namespace {
constexpr double kMinDist = 1.0;
constexpr double kMaximumPointKappa = 0.2;
constexpr std::size_t kMinSmoothLength = 150;
constexpr std::size_t kStepLength = 10;
}  // namespace

TrajectorySmoother::TrajectorySmoother() {
  TrajectoryUtilConfig util_config;
  if (!common::util::GetProtoFromFile(FLAGS_trajectory_util_config_filename,
                                      &util_config)) {
    AERROR << "Failed to read the trajectory smoother config file: "
           << FLAGS_trajectory_util_config_filename;
  }

  traj_smoother_config_ = util_config.smoother_config();

  if (!common::util::GetProtoFromFile(
          traj_smoother_config_.smoother_config_filename(),
          &smoother_config_)) {
    AERROR << "Failed to read the smoothing algorithm's config file: "
           << traj_smoother_config_.smoother_config_filename();
  }
}

bool TrajectorySmoother::Import(const std::string& filename) {
  filename_ = filename;
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    AERROR << "Can't open the raw trajecotry's file: " << filename;
    return false;
  }

  std::string point_str;
  while (std::getline(ifs, point_str)) {
    std::size_t idx = point_str.find(',');
    if (idx == std::string::npos) {
      continue;
    }
    auto x_str = point_str.substr(0, idx);
    auto y_str = point_str.substr(idx + 1);
    auto filter_point_func = [this](double x, double y) {
      if (raw_points_.size() > 0) {
        auto dx = x - raw_points_.back().x();
        auto dy = y - raw_points_.back().y();
        if (dx * dx + dy * dy > std::pow(kMinDist, 2)) {
          raw_points_.emplace_back(x, y);
        }
      } else {
        raw_points_.emplace_back(x, y);
      }
    };
    filter_point_func(std::stod(x_str), std::stod(y_str));
  }
  return true;
}

bool TrajectorySmoother::StepSmooth(const std::size_t smooth_length) {
  if (raw_points_.size() <= 2) {
    AERROR << "The original point size is " << raw_points_.size();
    return false;
  }
  std::size_t i = 1;
  {
    std::vector<ReferencePoint> ref_points;
    double s = 0.0;
    for (; s < traj_smoother_config_.smooth_length() && i < raw_points_.size();
         ++i) {
      LineSegment2d segment(raw_points_[i - 1], raw_points_[i]);
      ref_points.emplace_back(MapPathPoint(raw_points_[i], segment.heading()),
                              0.0, 0.0);
      s += segment.length();
    }
    if (!SmoothPoints(ref_points, &ref_points_)) {
      return false;
    }
  }
  for (; i < raw_points_.size(); ++i) {
    double s = 0.0;
    std::size_t j = ref_points_.size() - 1;
    while (j > 0 && s < traj_smoother_config_.smooth_length() / 2.0) {
      s += DistanceXY(ref_points_[j - 1], ref_points_[j]);
      --j;
    }
    ReferenceLine prev_half_ref(ref_points_.begin() + j, ref_points_.end());
    ref_points_.erase(ref_points_.begin() + j, ref_points_.end());
    common::SLPoint sl;
    prev_half_ref.XYToSL(raw_points_[i], &sl);
    while (sl.s() <= prev_half_ref.Length() && i + 1 < raw_points_.size()) {
      prev_half_ref.XYToSL(raw_points_[i + 1], &sl);
      ++i;
    }
    s = 0.0;
    j = i;
    auto ref_points = prev_half_ref.reference_points();
    while (j + 1 < raw_points_.size() &&
           s < traj_smoother_config_.smooth_length() / 2.0) {
      Vec2d vec = raw_points_[j + 1] - raw_points_[j];
      s += vec.Length();
      ref_points.emplace_back(MapPathPoint(raw_points_[j], vec.Angle()), 0.0,
                              0.0);
      ++j;
    }
    i = j;
    if (!SmoothPoints(ref_points, &ref_points_)) {
      return false;
    }
  }
  return true;
}

bool TrajectorySmoother::Smooth() {
  std::size_t smooth_length = traj_smoother_config_.smooth_length();
  while (smooth_length > kMinSmoothLength) {
    if (!StepSmooth(smooth_length)) {
      AWARN << "Smooth failed with smooth length : " << smooth_length;
      smooth_length -= kStepLength;
    } else {
      return true;
    }
  }
  AERROR << "Smooth failed.";
  return false;
}

bool TrajectorySmoother::Export(const std::string& filename) {
  FileOperator file_operator;
  return file_operator.Export(filename, ref_points_);
}

bool TrajectorySmoother::SmoothPoints(
    const std::vector<apollo::planning::ReferencePoint>& raw_points,
    std::vector<apollo::planning::ReferencePoint>* const smoothed_points) {
  CHECK_NOTNULL(smoothed_points);
  ReferenceLine init_ref(raw_points);
  std::unique_ptr<ReferenceLineSmoother> ptr_smoother =
      std::make_unique<QpSplineReferenceLineSmoother>(smoother_config_);
  if (IsSpiral(raw_points)) {
    ptr_smoother.reset();
    ptr_smoother =
        std::make_unique<SpiralReferenceLineSmoother>(smoother_config_);
    AINFO << "Default is QpSpline , but now we choose Spiral.";
  }
  // Prefer "std::make_unique" to direct use of "new".
  // Reference "https://herbsutter.com/gotw/_102/" for details.
  std::vector<AnchorPoint> anchors;
  if (!CreateAnchorPoints(init_ref.reference_points().front(), init_ref,
                          &anchors)) {
    AERROR << "Can't create anchor points.";
    return false;
  }
  ptr_smoother->SetAnchorPoints(anchors);
  ReferenceLine smoothed_init_ref;
  if (!ptr_smoother->Smooth(init_ref, &smoothed_init_ref)) {
    AERROR << "Smooth initial reference line failed";
    return false;
  }
  smoothed_points->insert(smoothed_points->end(),
                          smoothed_init_ref.reference_points().begin(),
                          smoothed_init_ref.reference_points().end());
  return true;
}

bool TrajectorySmoother::IsSpiral(
    const std::vector<apollo::planning::ReferencePoint>& points) {
  const auto& itr = std::minmax_element(
      points.begin(), points.end(),
      [](const ReferencePoint& item1, const ReferencePoint& item2) {
        return item1.heading() < item2.heading();
      });
  double delta_theta = itr.second->heading() - itr.first->heading();
  return delta_theta > kMaximumPointKappa ? true : false;
}

bool TrajectorySmoother::CreateAnchorPoints(
    const planning::ReferencePoint& init_point,
    const planning::ReferenceLine& ref_line,
    std::vector<planning::AnchorPoint>* anchor_points) {
  CHECK_NOTNULL(anchor_points);

  int num_of_anchors = std::max(
      2, static_cast<int>(ref_line.Length() /
                              smoother_config_.max_constraint_interval() +
                          0.5));
  std::vector<double> anchor_s;
  common::util::uniform_slice(0.0, ref_line.Length(), num_of_anchors - 1,
                              &anchor_s);
  common::SLPoint sl;
  if (!ref_line.XYToSL(Vec2d(init_point.x(), init_point.y()), &sl)) {
    AERROR << "Failed to project init point to reference line";
    return false;
  }
  bool set_init_point = false;
  for (const double s : anchor_s) {
    if (s + smoother_config_.max_constraint_interval() / 2.0 < sl.s()) {
      continue;
    }
    ReferencePoint ref_point;
    if (!set_init_point) {
      set_init_point = true;
      ref_point = init_point;
    } else {
      ref_point = ref_line.GetReferencePoint(s);
    }
    AnchorPoint anchor;
    anchor.path_point.set_x(ref_point.x());
    anchor.path_point.set_y(ref_point.y());
    anchor.path_point.set_z(0.0);
    anchor.path_point.set_s(s);
    anchor.path_point.set_theta(ref_point.heading());
    anchor.path_point.set_kappa(ref_point.kappa());
    anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
    anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
    anchor_points->emplace_back(anchor);
  }
  anchor_points->front().longitudinal_bound = 0;
  anchor_points->front().lateral_bound = 0;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 0;
  anchor_points->back().lateral_bound = 0;
  anchor_points->back().enforced = true;
  return true;
}
}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
