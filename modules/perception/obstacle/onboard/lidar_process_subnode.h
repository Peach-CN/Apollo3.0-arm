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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/sequence_type_fuser/base_type_fuser.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_object_builder.h"
#include "modules/perception/obstacle/lidar/interface/base_object_filter.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/interface/base_tracker.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/obstacle/onboard/scene_shared_data.h"
#include "modules/perception/onboard/subnode.h"

#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

namespace apollo {
namespace perception {

class LidarUtil {
 public:
  template <typename PointT>
  static bool IsXyPointIn2dXyPolygon(const PointT &point,
                                     const PolygonDType &polygon) {
    bool in_poly = false;
    double x1, x2, y1, y2;
    int nr_poly_points = static_cast<int>(polygon.points.size());
    // start with the last point to make the check last point<->first point the
    // first one
    double xold = polygon.points[nr_poly_points - 1].x;
    double yold = polygon.points[nr_poly_points - 1].y;
    for (int i = 0; i < nr_poly_points; i++) {
      double xnew = polygon.points[i].x;
      double ynew = polygon.points[i].y;
      if (xnew > xold) {
        x1 = xold;
        x2 = xnew;
        y1 = yold;
        y2 = ynew;
      } else {
        x1 = xnew;
        x2 = xold;
        y1 = ynew;
        y2 = yold;
      }
      if ((x1 < point.x) == (point.x <= x2) &&
          (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
        in_poly = !in_poly;
      }
      xold = xnew;
      yold = ynew;
    }
    return in_poly;
  }

  template <typename PointT>
  static bool IsXyPointInHdmap(const PointT &p,
                               const std::vector<PolygonDType> &polygons) {
    bool in_flag = false;
    for (std::size_t j = 0; j < polygons.size(); j++) {
      if (IsXyPointIn2dXyPolygon<PointT>(p, polygons[j])) {
        in_flag = true;
        break;
      }
    }
    return in_flag;
  }
};


class LidarProcessSubnode : public Subnode {
 public:
  LidarProcessSubnode() = default;
  ~LidarProcessSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

  float ground_estimate(pcl_util::PointCloudPtr in_cloud, int fore_or_back);
  
  // @brief check  point is vaild;
  // @brief params[IN] p: point to be checked
  // @return true if p is valid, otherwise return false
  bool isvalid_point(pcl_util::Point& p);

  // @brief filter invalid points;
  // @brief params[IN] in_cloud: original cloud to be filtered;
  // @brief params[OUT] out_cloud_indices: valid point indices;
  // @return nothing
  void filter_invalid_points(
      pcl_util::PointCloudPtr in_cloud,
      pcl_util::PointIndicesPtr out_cloud_indices);

  void filter_high_points(pcl_util::PointCloudPtr& in_cloud,
      pcl_util::PointIndicesPtr& out_cloud_indices);

  //void add_plane(pcl_util::PointCloudPtr& cloud);

  //bool cloud_preprocess(VLPCloud& vlp_cloud);

  int pt_pos_in_map(pcl_util::Point& p);

  bool is_near_point(pcl_util::Point& p);


  void OnPointCloud(const sensor_msgs::PointCloud2& message);

 protected:
  virtual SensorType GetSensorType() const = 0;
  virtual void AddMessageCallback() = 0;

 private:
  bool InitInternal() override;

  pcl_util::PointIndicesPtr GetROIIndices() { return roi_indices_; }

  void RegistAllAlgorithm();
  bool InitFrameDependence();
  bool InitAlgorithmPlugin();

  void TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                            pcl_util::PointCloudPtr* out_cloud);

  void PublishDataAndEvent(double timestamp,
                           const SharedDataPtr<SensorObjects>& data,
                           pcl_util::PointCloudPtr* cloud = nullptr);

  bool inited_ = false;
  double timestamp_ = 0.0;
  SeqId seq_num_ = 0;
  common::ErrorCode error_code_ = common::OK;
  LidarObjectData* processing_data_ = nullptr;
  SceneSharedData* scene_data_ = nullptr;
  std::string device_id_;

  HDMapInput* hdmap_input_ = nullptr;
  std::unique_ptr<BaseROIFilter> roi_filter_;
  std::unique_ptr<HdmapROIFilter> hdmap_roi_filter_;

  std::unique_ptr<BaseSegmentation> segmentor_;
  std::unique_ptr<BaseObjectFilter> object_filter_;
  std::unique_ptr<BaseObjectBuilder> object_builder_;
  std::unique_ptr<BaseTracker> tracker_;
  std::unique_ptr<BaseTypeFuser> type_fuser_;
  pcl_util::PointIndicesPtr roi_indices_;

  // @brief invalid zone;
  float _vehicle_rec_min_x;
  float _vehicle_rec_max_x;
  float _vehicle_rec_min_y;
  float _vehicle_rec_max_y;
  float _vehicle_rec_max_z;
  float _real_vehicle_min_x;
  float _real_vehicle_max_x;
  float _real_vehicle_min_y;
  float _real_vehicle_max_y;

  float _max_z_above_ground;

  std::vector<int> _near_cloud_map;
  std::vector<pcl_util::Point> _near_pts;
  const float _near_grid_size = 0.1;
  int _near_map_w = 0;
  int _near_map_h = 0;

  // @brief front ground height;
  float _fore_ground_h = 0.0;

  // @brief back ground height
  float _back_ground_h = 0.0;
};

class Lidar64ProcessSubnode : public LidarProcessSubnode {
 protected:
  SensorType GetSensorType() const override;
  void AddMessageCallback() override;
};

class Lidar16ProcessSubnode : public LidarProcessSubnode {
 protected:
  SensorType GetSensorType() const override;
  void AddMessageCallback() override;
};

REGISTER_SUBNODE(Lidar64ProcessSubnode);

// To use 16-beam Lidar, you need to
// 1. Point to proper model by setting --cnn_segmentation_config, which is
//    defined in modules/perception/common/perception_gflags.cc. The model is
//    generally located at modules/perception/model.
// 2. Define subnode config in modules/perception/conf/dag_streaming.config:
//    subnodes {
//      id: 1
//      name: "Lidar16ProcessSubnode"
//      reserve: "device_id:velodyne16;"
//      type: SUBNODE_IN
//    }
REGISTER_SUBNODE(Lidar16ProcessSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
