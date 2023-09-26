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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_FAST_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_FAST_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT

#include <memory>
#include <string>
#include <list>
#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <condition_variable>

#include "caffe/caffe.hpp"

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/proto/fast_cnnseg.pb.h"
#include "modules/perception/proto/cnn_segmentation_config.pb.h"

#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/fast_cluster2d.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/fast_feature_generator.h"

#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/obstacle/camera/common/cnn_adapter.h"

namespace apollo {
namespace perception {

class FastCNNSegmentation : public BaseSegmentation {

public:
  FastCNNSegmentation() : BaseSegmentation() {}
  ~FastCNNSegmentation() {}

  bool Init() override;

  bool Segment(pcl_util::PointCloudPtr pc_ptr,
               const pcl_util::PointIndices& non_ground_indices,
               const SegmentationOptions& options,
               std::vector<std::shared_ptr<Object>>* objects) override;

  std::string name() const override { return "FastCNNSegmentation"; }

  float range() const { return _range; }
  int width() const { return _width; }
  int height() const { return _height; }

  bool get_configs(std::string& config_file, std::string& proto_file,
                  std::string& weight_file, std::string& feature_config,
                  std::string& post_config);

protected:
  std::shared_ptr<CNNAdapter> _cnnadapter;
  cnnseg::FastCNNSegParam _fast_cnnseg_param;
  std::shared_ptr<cnnseg::FastFeatureGenerator<float>> _fast_feature_generator;

  int _range;
  int _width;
  int _height;
  int _channels;
  int _num_classes;

  boost::shared_ptr<caffe::Blob<float>> _instance_pt_blob;
  boost::shared_ptr<caffe::Blob<float>> _category_pt_blob;
  boost::shared_ptr<caffe::Blob<float>> _confidence_pt_blob;
  boost::shared_ptr<caffe::Blob<float>> _classify_pt_blob;
  boost::shared_ptr<caffe::Blob<float>> _heading_pt_blob;
  boost::shared_ptr<caffe::Blob<float>> _height_pt_blob;
  boost::shared_ptr<caffe::Blob<float>> _feature_blob;

  std::vector<InternalObjectType> _type_map;

  cnnseg::FastCluster2D _fast_cluster2d;

private:

  DISALLOW_COPY_AND_ASSIGN(FastCNNSegmentation);
};

REGISTER_SEGMENTATION(FastCNNSegmentation);

}  // namespace perception
}  // namespace apollo

#endif
