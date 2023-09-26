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

#include "modules/perception/obstacle/onboard/lidar_process_subnode.h"

#include <unordered_map>

#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sequence_type_fuser/sequence_type_fuser.h"
#include "modules/perception/obstacle/lidar/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"
#include "modules/perception/obstacle/lidar/object_filter/low_object_filter/low_object_filter.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/fast_cnn_segmentation.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/onboard/transform_input.h"

DEFINE_bool(use_new_perception_config, false, "use new perception config");
DEFINE_bool(xsq_perception_config, false, "xsq perception config");
DEFINE_bool(perception_printf, false, "perception printf");
DEFINE_bool(remove_lane_out_obstacle, false, "remove lane out obstacle");
DEFINE_bool(use_velodyne_perception, true, "use veldyne perception");
DEFINE_bool(use_lslidar_perception, false, "use lslidar perception");

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;
using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointD;
using pcl_util::PointIndices;
using pcl_util::PointIndicesPtr;

#define VLP_LIDAR_LINES 16

bool LidarProcessSubnode::InitInternal() {
  if (inited_) {
    return true;
  }

  RegistAllAlgorithm();

  if (!InitFrameDependence()) {
    AERROR << "failed to Init frame dependence.";
    return false;
  }

  if (!InitAlgorithmPlugin()) {
    AERROR << "failed to Init algorithm plugin.";
    return false;
  }
  // parse reserve fileds
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve filed: " << reserve_;
    return false;
  }

  if (reserve_field_map.find("device_id") == reserve_field_map.end()) {
    AERROR << "Failed to find field device_id, reserve: " << reserve_;
    return false;
  }
  device_id_ = reserve_field_map["device_id"];
  AddMessageCallback();

  inited_ = true;

  _vehicle_rec_min_x = -1.8;
  _vehicle_rec_max_x = 1.8;
  _vehicle_rec_min_y = -2.0;
  _vehicle_rec_max_y = 4.5;
  _vehicle_rec_max_z = 2.0;
  _real_vehicle_min_x = -1.15;
  _real_vehicle_max_x = 1.15;
  _real_vehicle_min_y = -1.0;
  _real_vehicle_max_y = 2.65;

  _max_z_above_ground = 3.0;
  
  _near_map_w = (_vehicle_rec_max_x - _vehicle_rec_min_x)/_near_grid_size;
  _near_map_h = (_vehicle_rec_max_y - _vehicle_rec_min_y)/_near_grid_size;
  _near_cloud_map.resize(_near_map_w*_near_map_h, 0);
  _near_pts.reserve(5000);

  return true;
}

float LidarProcessSubnode::ground_estimate(pcl_util::PointCloudPtr in_cloud, int fore_or_back) {
    if (fore_or_back > 0) {
        ADEBUG << "getting foreground height...";
    }
    if (fore_or_back < 0) {
        ADEBUG << "getting background height...";
    }
    float step = 0.1;
    float lowest_h = -5.0;
    int hist_bin_size = fabs(lowest_h * 2) / step;
    int lidar_line_num = 16;
    std::vector<float> _height_histogram(hist_bin_size, 0);

    int _one_lidar_cloud_size = 0;
    bool _use_back_estimate_ground = false;

    for (size_t i = 0; i < in_cloud->size(); i ++) {
        pcl_util::Point& p = in_cloud->points[i];
        if (!_use_back_estimate_ground && static_cast<int>(p.h) < _one_lidar_cloud_size) {
            continue;
        }
        int line_index = (static_cast<int>(p.h)) % VLP_LIDAR_LINES;
        if (line_index == 0 && p.y * fore_or_back > 0.0) {
            int hist_idx = (p.z - lowest_h) / step;
            if (0 <= hist_idx && hist_idx < hist_bin_size) {
                _height_histogram[hist_idx] += 2.0;
            }
        }
        if (line_index == 1 && p.y * fore_or_back > 0.0) {
            int hist_idx = (p.z - lowest_h) / step;
            if (0 <= hist_idx && hist_idx < hist_bin_size) {
                _height_histogram[hist_idx] += 2.0;
            }
        }
        if (line_index == 2 && p.y * fore_or_back > 0.0) {
            int hist_idx = (p.z - lowest_h) / step;
            if (0 <= hist_idx && hist_idx < hist_bin_size) {
                _height_histogram[hist_idx] += 1.0;
            }
        }
        if (line_index == 3 && p.y * fore_or_back > 0.0) {
            int hist_idx = (p.z - lowest_h) / step;
            if (0 <= hist_idx && hist_idx < hist_bin_size) {
                _height_histogram[hist_idx] += 0.5;
            }
        }
    }
    std::vector<float> _sorted_histogram = _height_histogram;
    std::sort(_sorted_histogram.begin(), _sorted_histogram.end(), std::greater<float>());
    for (int i = 0; i < _height_histogram.size(); ++i) {
        ADEBUG << _height_histogram[i] << "   " << _sorted_histogram[i];
    }
    float sum = accumulate(_height_histogram.begin(), _height_histogram.end(), 0.0);
    std::vector<float>::iterator iter0 = std::find(_height_histogram.begin(), _height_histogram.end(), _sorted_histogram[0]);
    std::vector<float>::iterator iter1 = std::find(_height_histogram.begin(), _height_histogram.end(), _sorted_histogram[1]);
    std::vector<float>::iterator iter2 = std::find(_height_histogram.begin(), _height_histogram.end(), _sorted_histogram[2]);

    if (iter0 != _height_histogram.end() && *iter0 > 0 &&
            (iter0 - _height_histogram.begin()) < hist_bin_size / 1.5) {
        ADEBUG << "ground to vector head dis: " << iter0 - _height_histogram.begin();
        float ground_height = (iter0 - _height_histogram.begin()) * step + lowest_h;
        ADEBUG << "select mode: first max histogram bin, ground height: " << ground_height << " ground bin size: " << *iter0;
        return ground_height;
    }

    if (iter1 != _height_histogram.end() && *iter1 > 0 &&
            (iter1 - _height_histogram.begin()) < hist_bin_size / 1.5 ) {
        float ground_height = (iter1 - _height_histogram.begin()) * step + lowest_h;
        ADEBUG << "select mode: second max histogram bin, ground height: " << ground_height << " ground bin size: " << *iter1;
        return ground_height;
    }

    if (iter2 != _height_histogram.end() && *iter2 > 0 &&
            (iter2 - _height_histogram.begin()) < hist_bin_size / 1.5 ) {
        float ground_height = (iter2 - _height_histogram.begin()) * step + lowest_h;
        ADEBUG << "select mode: third max histogram bin, ground height: " << ground_height << " ground bin size: " << *iter2;
        return ground_height;
    }
    float ground_height = 0.0;
    float ground_sum = 0.00001;
    for (int i = 0; i < hist_bin_size / 1.5; ++i) {
        ground_sum += _height_histogram[i];
        ground_height += (i * step + lowest_h) * _height_histogram[i];
    }
    ADEBUG << "ground sum: " << ground_sum;
    ground_height = ground_height / ground_sum;
    ADEBUG << "select mode: aver mode, ground height: " << ground_height;
    return ground_height;
}

bool LidarProcessSubnode::isvalid_point(pcl_util::Point& p) {
    if (std::isnan(p.x)) {
        return false;
    }
    if (p.z > _vehicle_rec_max_z) {
        return false;
    }
    if (_real_vehicle_min_x <= p.x && p.x <= _real_vehicle_max_x &&
        _real_vehicle_min_y <= p.y && p.y <= _real_vehicle_max_y) {
        return false;
    }
    return true;
}

inline int LidarProcessSubnode::pt_pos_in_map(pcl_util::Point& p) {
    int pos_x = (p.x - _vehicle_rec_min_x)/_near_grid_size;
    int pos_y = (p.y - _vehicle_rec_min_y)/_near_grid_size;
    return pos_y*_near_map_w + pos_x;
}

bool LidarProcessSubnode::is_near_point(pcl_util::Point& p) {
    // zoom in for a little for not judgement after
    if (_vehicle_rec_min_x + _near_grid_size/10.0 <= p.x && p.x <= _vehicle_rec_max_x - _near_grid_size/10.0 &&
        _vehicle_rec_min_y + _near_grid_size/10.0 <= p.y && p.y <= _vehicle_rec_max_y - _near_grid_size/10.0) {
        _near_cloud_map[pt_pos_in_map(p)]++;
        return true;
    }
    return false;
}

void LidarProcessSubnode::filter_invalid_points(pcl_util::PointCloudPtr in_cloud,
        pcl_util::PointIndicesPtr out_cloud_indices) {
    _near_pts.clear();
    _near_cloud_map.clear();
    _near_cloud_map.resize(_near_map_w*_near_map_h, 0);
    out_cloud_indices->indices.clear();
    out_cloud_indices->indices.reserve(in_cloud->size());
    for (size_t i = 0; i < in_cloud->size(); ++i) {
        pcl_util::Point& p = in_cloud->points[i];
        if (isvalid_point(p)) {
            p.h = i;
            if (is_near_point(p)) {
                _near_pts.push_back(p);
            } else {
                out_cloud_indices->indices.push_back(i);
            }
        }
    }
    for (size_t i = 0; i < _near_pts.size(); ++i) {
        if (_near_cloud_map[pt_pos_in_map(_near_pts[i])] > 5) {
            out_cloud_indices->indices.push_back(_near_pts[i].h);
        }
    }
}

void LidarProcessSubnode::filter_high_points(pcl_util::PointCloudPtr& in_cloud,
        pcl_util::PointIndicesPtr& out_cloud_indices) {
    _fore_ground_h = ground_estimate(in_cloud, 1);
    _back_ground_h = ground_estimate(in_cloud, -1);
    out_cloud_indices->indices.clear();
    out_cloud_indices->indices.reserve(in_cloud->size());
    for (size_t i = 0; i < in_cloud->size(); ++i) {
        pcl_util::Point& p = in_cloud->points[i];
        if (0 <= p.y &&
            _fore_ground_h + 0.2 < p.z && p.z < _fore_ground_h + _max_z_above_ground) {
            out_cloud_indices->indices.push_back(i);
        }
        if (p.y < 0 &&
            _back_ground_h + 0.2 < p.z && p.z < _back_ground_h + _max_z_above_ground) {
            out_cloud_indices->indices.push_back(i);
        }
    }
}

void LidarProcessSubnode::OnPointCloud(
    const sensor_msgs::PointCloud2& message) {
  AINFO << "process OnPointCloud.";
  PERF_FUNCTION("LidarProcessSubnode");
  if (!inited_) {
    AERROR << "the LidarProcessSubnode has not been Init";
    return;
  }

  if(FLAGS_use_new_perception_config)
  {
    const double kTimeStamp = message.header.stamp.toSec();
    timestamp_ = kTimeStamp;
    ++seq_num_;

    std::shared_ptr<SensorObjects> out_sensor_objects(new SensorObjects);
    out_sensor_objects->timestamp = timestamp_;
    out_sensor_objects->sensor_type = GetSensorType();
    out_sensor_objects->sensor_id = device_id_;
    out_sensor_objects->seq_num = seq_num_;

    PERF_BLOCK_START();
    std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
    if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
      AERROR << "failed to get trans at timestamp: "
            << GLOG_TIMESTAMP(kTimeStamp);
      return;
    } 

    out_sensor_objects->sensor2world_pose = *velodyne_trans;
    AINFO << "get lidar trans pose succ. pose: \n" << *velodyne_trans;
    PERF_BLOCK_END("lidar_get_velodyne2world_transfrom");

    /// call hdmap to get ROI
    if (FLAGS_use_navigation_mode) {
      AdapterManager::Observe();
    }
    HdmapStructPtr hdmap = nullptr;
    if (hdmap_input_) {
      PointD velodyne_pose = {0.0, 0.0, 0.0, 0};  // (0,0,0)
      Affine3d temp_trans(*velodyne_trans);
      PointD velodyne_pose_world = pcl::transformPoint(velodyne_pose, temp_trans);
      hdmap.reset(new HdmapStruct);
      //hdmap_input_->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap);
      hdmap_input_->GetROI(velodyne_pose_world, 120.0, &hdmap);
      PERF_BLOCK_END("lidar_get_roi_from_hdmap");
    }

    PointCloudPtr point_cloud(new PointCloud);
    TransPointCloudToPCL(message, &point_cloud);
    ADEBUG << "transform pointcloud success. points num is: "
          << point_cloud->points.size();
    PERF_BLOCK_END("lidar_transform_poindcloud");

    /// call segmentor
    std::vector<std::shared_ptr<Object>> objects;

    if(FLAGS_xsq_perception_config)
    {
      //1. filter nan & on vehicle points
      pcl_util::PointIndicesPtr valid_cloud_indices (new pcl_util::PointIndices);
      pcl_util::PointCloudPtr valid_cloud (new pcl_util::PointCloud);
      filter_invalid_points(point_cloud, valid_cloud_indices);

      pcl::copyPointCloud(*(point_cloud), *valid_cloud_indices, *valid_cloud);
      filter_high_points(valid_cloud, valid_cloud_indices);
      pcl_util::PointCloudPtr no_h_valid_cloud (new pcl_util::PointCloud);
      pcl::copyPointCloud(*valid_cloud, *valid_cloud_indices, *no_h_valid_cloud);
      valid_cloud = no_h_valid_cloud;
      ADEBUG << "VLP detect valid cloud size: " << valid_cloud->size();

      /// call segmentor
      //std::vector<std::shared_ptr<Object>> objects;
      if (segmentor_ != nullptr) {
        SegmentationOptions segmentation_options;
        segmentation_options.origin_cloud = valid_cloud;
        PointIndices non_ground_indices;
        non_ground_indices.indices.resize(valid_cloud->points.size());
        //non_ground_indices.indices.resize(point_cloud->points.size());

        std::iota(non_ground_indices.indices.begin(),
                  non_ground_indices.indices.end(), 0);

        if (!segmentor_->Segment(valid_cloud, non_ground_indices,
                                segmentation_options, &objects)) {
          AERROR << "failed to call segmention.";
          return;
        }
      }
      ADEBUG << "call segmentation succ. The num of objects is: " << objects.size();
      PERF_BLOCK_END("lidar_segmentation");

      if(FLAGS_perception_printf)
      {
        AINFO << "------------------------- objects.size(): " << objects.size();
      }
    }
    else
    {
      /// call segmentor
      //std::vector<std::shared_ptr<Object>> objects;
      if (segmentor_ != nullptr) {
        SegmentationOptions segmentation_options;
        segmentation_options.origin_cloud = point_cloud;
        PointIndices non_ground_indices;
        non_ground_indices.indices.resize(point_cloud->points.size());
        // non_ground_indices.indices.resize(point_cloud->points.size());

        std::iota(non_ground_indices.indices.begin(),
                  non_ground_indices.indices.end(), 0);
        if (!segmentor_->Segment(point_cloud, non_ground_indices,
                                segmentation_options, &objects)) {
          AERROR << "failed to call segmention.";
          return;
        }
      }
      ADEBUG << "call segmentation succ. The num of objects is: " << objects.size();
      PERF_BLOCK_END("lidar_segmentation");

      if(FLAGS_perception_printf)
      {
        AINFO << "------------------------- point_cloud->points.size(): " << point_cloud->points.size();
        AINFO << "------------------------- point_cloud->width: " << point_cloud->width;
        AINFO << "------------------------- point_cloud->height: " << point_cloud->height;
        AINFO << "------------------------- objects.size(): " << objects.size();
      }     
    }

    /// call object filter
    if (object_filter_ != nullptr) {
      ObjectFilterOptions object_filter_options;
      object_filter_options.velodyne_trans.reset(new Eigen::Matrix4d);
      object_filter_options.velodyne_trans = velodyne_trans;
      // object_filter_options.hdmap_struct_ptr = hdmap;

      if (!object_filter_->Filter(object_filter_options, &objects)) {
        AERROR << "failed to call object filter.";
        return;
      }
    }
    ADEBUG << "call object filter succ. The num of objects is: "
          << objects.size();
    PERF_BLOCK_END("lidar_object_filter");

    /// call object builder
    if (object_builder_ != nullptr) {
      ObjectBuilderOptions object_builder_options;
      if (!object_builder_->Build(object_builder_options, &objects)) {
        AERROR << "failed to call object builder.";
        return;
      }
    }
    ADEBUG << "call object_builder succ.";
    PERF_BLOCK_END("lidar_object_builder");

    /// call tracker
    if (tracker_ != nullptr) {
      TrackerOptions tracker_options;
      tracker_options.velodyne_trans = velodyne_trans;
      tracker_options.hdmap = hdmap;
      tracker_options.hdmap_input = hdmap_input_;
      if (!tracker_->Track(objects, timestamp_, tracker_options,
                          &(out_sensor_objects->objects))) {
        AERROR << "failed to call tracker.";
        return;
      }
    }
    ADEBUG << "call tracker succ, there are "
          << out_sensor_objects->objects.size() << " tracked objects.";
    PERF_BLOCK_END("lidar_tracker");

    AINFO << "Segment object num: " << objects.size()
      << " Tracker object num: " << out_sensor_objects->objects.size();

    /// call type fuser
    if (type_fuser_ != nullptr) {
      TypeFuserOptions type_fuser_options;
      type_fuser_options.timestamp = timestamp_;
      if (!type_fuser_->FuseType(type_fuser_options,
                                &(out_sensor_objects->objects))) {
        return;
      }
    }
    ADEBUG << "lidar process succ.";
    PERF_BLOCK_END("lidar_type_fuser");

    if(FLAGS_remove_lane_out_obstacle)
    {
      std::vector<PolygonDType> map_polygons;
      if (hdmap_roi_filter_ != nullptr) 
      {
        hdmap_roi_filter_->MergeHdmapStructToPolygons(hdmap, &map_polygons);
      }

      int obs_number = 0;
      for (size_t i = 0; i < out_sensor_objects->objects.size(); i++) {
        pcl_util::PointD obs_position;
        obs_position.x = out_sensor_objects->objects.at(i)->center(0);
        obs_position.y = out_sensor_objects->objects.at(i)->center(1);
        obs_position.z = out_sensor_objects->objects.at(i)->center(2);
        if (LidarUtil::IsXyPointInHdmap<pcl_util::PointD>(obs_position,
                                                          map_polygons)) {                                                
          out_sensor_objects->objects.at(obs_number) = out_sensor_objects->objects.at(i);
          obs_number++;
        }
      }
      out_sensor_objects->objects.resize(obs_number);
      if(FLAGS_perception_printf)
      {
        AINFO << "------------------------- obs_number: " << obs_number;
        ADEBUG << "query hdmap sucessfully!";      
      }
    }

    // if visualization mode, add the point cloud outside
    PublishDataAndEvent(timestamp_, out_sensor_objects, &point_cloud);
  }
  else
  {
    const double kTimeStamp = message.header.stamp.toSec();
    timestamp_ = kTimeStamp;
    ++seq_num_;

    std::shared_ptr<SensorObjects> out_sensor_objects(new SensorObjects);
    out_sensor_objects->timestamp = timestamp_;
    out_sensor_objects->sensor_type = GetSensorType();
    out_sensor_objects->sensor_id = device_id_;
    out_sensor_objects->seq_num = seq_num_;

    PERF_BLOCK_START();
    /// get velodyne2world transfrom
    std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
    if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
      AERROR << "failed to get trans at timestamp: "
            << GLOG_TIMESTAMP(kTimeStamp);
      return;
    }
    out_sensor_objects->sensor2world_pose = *velodyne_trans;
    AINFO << "get lidar trans pose succ. pose: \n" << *velodyne_trans;
    PERF_BLOCK_END("lidar_get_velodyne2world_transfrom");

    PointCloudPtr point_cloud(new PointCloud);
    TransPointCloudToPCL(message, &point_cloud);
    ADEBUG << "transform pointcloud success. points num is: "
          << point_cloud->points.size();
    PERF_BLOCK_END("lidar_transform_poindcloud");

    // error_code_ = common::OK;

    /// call hdmap to get ROI
    if (FLAGS_use_navigation_mode) {
      AdapterManager::Observe();
    }
    HdmapStructPtr hdmap = nullptr;
    if (hdmap_input_) {
      PointD velodyne_pose = {0.0, 0.0, 0.0, 0};  // (0,0,0)
      Affine3d temp_trans(*velodyne_trans);
      PointD velodyne_pose_world = pcl::transformPoint(velodyne_pose, temp_trans);
      hdmap.reset(new HdmapStruct);
      hdmap_input_->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap);
      PERF_BLOCK_END("lidar_get_roi_from_hdmap");
    }

    /// call roi_filter
    PointCloudPtr roi_cloud(new PointCloud);
    if (roi_filter_ != nullptr) {
      PointIndicesPtr roi_indices(new PointIndices);
      ROIFilterOptions roi_filter_options;
      roi_filter_options.velodyne_trans = velodyne_trans;
      roi_filter_options.hdmap = hdmap;
      if (roi_filter_->Filter(point_cloud, roi_filter_options,
                              roi_indices.get())) {
        pcl::copyPointCloud(*point_cloud, *roi_indices, *roi_cloud);
        roi_indices_ = roi_indices;
      } else {
        AERROR << "failed to call roi filter.";
        return;
      }
    }
    ADEBUG << "call roi_filter succ. The num of roi_cloud is: "
          << roi_cloud->points.size();
    PERF_BLOCK_END("lidar_roi_filter");

    /// call segmentor
    std::vector<std::shared_ptr<Object>> objects;
    if (segmentor_ != nullptr) {
      SegmentationOptions segmentation_options;
      segmentation_options.origin_cloud = point_cloud;
      PointIndices non_ground_indices;
      non_ground_indices.indices.resize(roi_cloud->points.size());
      // non_ground_indices.indices.resize(point_cloud->points.size());

      std::iota(non_ground_indices.indices.begin(),
                non_ground_indices.indices.end(), 0);
      if (!segmentor_->Segment(roi_cloud, non_ground_indices,
                              segmentation_options, &objects)) {
        AERROR << "failed to call segmention.";
        return;
      }
    }
    ADEBUG << "call segmentation succ. The num of objects is: " << objects.size();
    PERF_BLOCK_END("lidar_segmentation");

    if(FLAGS_perception_printf)
    {
      AINFO << "------------------------- objects.size(): " << objects.size();
    }

    /// call object filter
    if (object_filter_ != nullptr) {
      ObjectFilterOptions object_filter_options;
      object_filter_options.velodyne_trans.reset(new Eigen::Matrix4d);
      object_filter_options.velodyne_trans = velodyne_trans;
      // object_filter_options.hdmap_struct_ptr = hdmap;

      if (!object_filter_->Filter(object_filter_options, &objects)) {
        AERROR << "failed to call object filter.";
        return;
      }
    }
    ADEBUG << "call object filter succ. The num of objects is: "
          << objects.size();
    PERF_BLOCK_END("lidar_object_filter");

    /// call object builder
    if (object_builder_ != nullptr) {
      ObjectBuilderOptions object_builder_options;
      if (!object_builder_->Build(object_builder_options, &objects)) {
        AERROR << "failed to call object builder.";
        return;
      }
    }
    ADEBUG << "call object_builder succ.";
    PERF_BLOCK_END("lidar_object_builder");

    /// call tracker
    if (tracker_ != nullptr) {
      TrackerOptions tracker_options;
      tracker_options.velodyne_trans = velodyne_trans;
      tracker_options.hdmap = hdmap;
      tracker_options.hdmap_input = hdmap_input_;
      if (!tracker_->Track(objects, timestamp_, tracker_options,
                          &(out_sensor_objects->objects))) {
        AERROR << "failed to call tracker.";
        return;
      }
    }
    ADEBUG << "call tracker succ, there are "
          << out_sensor_objects->objects.size() << " tracked objects.";
    PERF_BLOCK_END("lidar_tracker");

    /// call type fuser
    if (type_fuser_ != nullptr) {
      TypeFuserOptions type_fuser_options;
      type_fuser_options.timestamp = timestamp_;
      if (!type_fuser_->FuseType(type_fuser_options,
                                &(out_sensor_objects->objects))) {
        return;
      }
    }
    ADEBUG << "lidar process succ.";
    PERF_BLOCK_END("lidar_type_fuser");

    // if visualization mode, add the point cloud outside
    PublishDataAndEvent(timestamp_, out_sensor_objects, &point_cloud);
  }

  return;
}

void LidarProcessSubnode::RegistAllAlgorithm() {
  RegisterFactoryDummyROIFilter();
  RegisterFactoryDummySegmentation();
  RegisterFactoryDummyObjectBuilder();
  RegisterFactoryDummyTracker();
  RegisterFactoryDummyTypeFuser();

  RegisterFactoryHdmapROIFilter();
  RegisterFactoryLowObjectFilter();
  RegisterFactoryCNNSegmentation();
  RegisterFactoryFastCNNSegmentation();
  RegisterFactoryMinBoxObjectBuilder();
  RegisterFactoryHmObjectTracker();
  RegisterFactorySequenceTypeFuser();
}

bool LidarProcessSubnode::InitFrameDependence() {
  /// init share data
  CHECK(shared_data_manager_ != nullptr);
  // init preprocess_data
  const std::string lidar_processing_data_name("LidarObjectData");
  processing_data_ = dynamic_cast<LidarObjectData*>(
      shared_data_manager_->GetSharedData(lidar_processing_data_name));
  if (processing_data_ == nullptr) {
    AERROR << "Failed to get shared data instance "
           << lidar_processing_data_name;
    return false;
  }

  const std::string scene_processing_data_name("SceneSharedData");
  scene_data_ = dynamic_cast<SceneSharedData*>(
      shared_data_manager_->GetSharedData(scene_processing_data_name));
  if (scene_data_ == nullptr) {
    AERROR << "Failed to get shared data instance "
           << scene_processing_data_name;
    return false;
  }

  AINFO << "Init shared data successfully, data: " << processing_data_->name();

  /// init hdmap
  if (FLAGS_enable_hdmap_input) {
    hdmap_input_ = HDMapInput::instance();
    if (!hdmap_input_) {
      AERROR << "failed to get HDMapInput instance.";
      return false;
    }
    AINFO << "get and Init hdmap_input succ.";
  }

  return true;
}

bool LidarProcessSubnode::InitAlgorithmPlugin() {
  /// init roi filter
  if(FLAGS_remove_lane_out_obstacle && FLAGS_use_new_perception_config)
  {
    hdmap_roi_filter_.reset(new HdmapROIFilter());
    if (!hdmap_roi_filter_) {
      AERROR << "Failed to get instance: " << FLAGS_onboard_roi_filter;
      return false;
    }
    if (!hdmap_roi_filter_->Init()) {
      AERROR << "Failed to Init roi filter: " << hdmap_roi_filter_->name();
      return false;
    }
    AINFO << "Init algorithm plugin successfully, hdmap_roi_filter_: "
          << hdmap_roi_filter_->name();
  }
  else
  {
    roi_filter_.reset(
        BaseROIFilterRegisterer::GetInstanceByName(FLAGS_onboard_roi_filter));
    if (!roi_filter_) {
      AERROR << "Failed to get instance: " << FLAGS_onboard_roi_filter;
      return false;
    }
    if (!roi_filter_->Init()) {
      AERROR << "Failed to Init roi filter: " << roi_filter_->name();
      return false;
    }
    AINFO << "Init algorithm plugin successfully, roi_filter_: "
          << roi_filter_->name();
  }

  /// init segmentation
  segmentor_.reset(
      BaseSegmentationRegisterer::GetInstanceByName(FLAGS_onboard_segmentor));
  if (!segmentor_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_segmentor;
    return false;
  }
  if (!segmentor_->Init()) {
    AERROR << "Failed to Init segmentor: " << segmentor_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, segmentor: "
        << segmentor_->name();

  /// init object build
  object_builder_.reset(BaseObjectBuilderRegisterer::GetInstanceByName(
      FLAGS_onboard_object_builder));
  if (!object_builder_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_object_builder;
    return false;
  }
  if (!object_builder_->Init()) {
    AERROR << "Failed to Init object builder: " << object_builder_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, object builder: "
        << object_builder_->name();

  /// init pre object filter
  object_filter_.reset(BaseObjectFilterRegisterer::GetInstanceByName(
      FLAGS_onboard_object_filter));
  if (!object_filter_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_object_filter;
    return false;
  }
  if (!object_filter_->Init()) {
    AERROR << "Failed to Init object filter: " << object_filter_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, object filter: "
        << object_filter_->name();

  /// init tracker
  tracker_.reset(
      BaseTrackerRegisterer::GetInstanceByName(FLAGS_onboard_tracker));
  if (!tracker_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_tracker;
    return false;
  }
  if (!tracker_->Init()) {
    AERROR << "Failed to Init tracker: " << tracker_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, tracker: " << tracker_->name();

  /// init type fuser
  type_fuser_.reset(
      BaseTypeFuserRegisterer::GetInstanceByName(FLAGS_onboard_type_fuser));
  if (!type_fuser_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_type_fuser;
    return false;
  }
  if (!type_fuser_->Init()) {
    AERROR << "Failed to Init type_fuser: " << type_fuser_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, type_fuser: "
        << type_fuser_->name();

  return true;
}

void LidarProcessSubnode::TransPointCloudToPCL(
    const sensor_msgs::PointCloud2& in_msg, PointCloudPtr* out_cloud) {
  // transform from ros to pcl
  pcl::PointCloud<pcl_util::PointXYZIT> in_cloud;
  pcl::fromROSMsg(in_msg, in_cloud);
  // transform from xyzit to xyzi
  PointCloudPtr& cloud = *out_cloud;
  cloud->header = in_cloud.header;
  cloud->width = in_cloud.width;
  cloud->height = in_cloud.height;
  cloud->is_dense = in_cloud.is_dense;
  cloud->sensor_origin_ = in_cloud.sensor_origin_;
  cloud->sensor_orientation_ = in_cloud.sensor_orientation_;
  cloud->points.resize(in_cloud.points.size());
  size_t points_num = 0;
  for (size_t idx = 0; idx < in_cloud.size(); ++idx) {
    pcl_util::PointXYZIT& pt = in_cloud.points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
        !std::isnan(pt.intensity)) {
      cloud->points[points_num].x = pt.x;
      cloud->points[points_num].y = pt.y;
      cloud->points[points_num].z = pt.z;
      cloud->points[points_num].intensity = pt.intensity;
      ++points_num;
    }
  }
  cloud->points.resize(points_num);
}

void LidarProcessSubnode::PublishDataAndEvent(
    double timestamp, const SharedDataPtr<SensorObjects>& data,
    PointCloudPtr* cloud) {
  // set shared data
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "Failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return;
  }

  AINFO << "lidar object size is " << data->objects.size();

  processing_data_->Add(key, data);

  // adding point cloud to scene
  if (cloud != nullptr) {
    SharedDataPtr<SceneItem> sdata(new SceneItem());
    sdata->timestamp = timestamp;
    sdata->cloud = *cloud;
    sdata->pose = data->sensor2world_pose;
    scene_data_->Add(key, sdata);
  }

  // pub events
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta& event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}
SensorType Lidar64ProcessSubnode::GetSensorType() const {
  return SensorType::VELODYNE_64;
}

void Lidar64ProcessSubnode::AddMessageCallback() {
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback<LidarProcessSubnode>(
      &LidarProcessSubnode::OnPointCloud, this);
}

SensorType Lidar16ProcessSubnode::GetSensorType() const {
  return SensorType::VELODYNE_16;
}

void Lidar16ProcessSubnode::AddMessageCallback() {
  if(FLAGS_use_velodyne_perception)
  {
    CHECK(AdapterManager::GetVLP16PointCloud())
        << "VLP16 PointCloud is not initialized.";
    AdapterManager::AddVLP16PointCloudCallback<LidarProcessSubnode>(
        &LidarProcessSubnode::OnPointCloud, this);
  }

  if(FLAGS_use_lslidar_perception)
  {
    CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
    AdapterManager::AddPointCloudCallback<LidarProcessSubnode>(
        &LidarProcessSubnode::OnPointCloud, this);
  }
}

}  // namespace perception
}  // namespace apollo
