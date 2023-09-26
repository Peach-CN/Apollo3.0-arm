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

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/fast_feature_generator.h"

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/util.h"

#define USE_MICROCAR_CAFFE_MODEL

using std::vector;

namespace apollo {
namespace perception {
namespace cnnseg {

template <typename Dtype>
bool FastFeatureGenerator<Dtype>::Init(const FastFeatureParam& fast_feature_param,
                                   caffe::Blob<Dtype>* out_blob) {
  out_blob_ = out_blob;

  // birdview parameters
  const auto& fast_bird_view_param = fast_feature_param.fast_bird_view_param();
  range_ = fast_bird_view_param.point_cloud_range();
  width_ = fast_bird_view_param.width();
  height_ = fast_bird_view_param.height();
  min_height_ = fast_bird_view_param.min_height();
  max_height_ = fast_bird_view_param.max_height();
  CHECK_EQ(width_, height_) << "Current implementation version requires that "
          "input_width == input_height.";

  AINFO << "--    range_: " << range_;
  AINFO << "--    width_: " << width_;
  AINFO << "--    height_: " << height_;
  AINFO << "--    min_height_: " << min_height_;
  AINFO << "--    max_height_: " << max_height_;

  CHECK(fast_bird_view_param.use_max_height());
  CHECK(fast_bird_view_param.use_mean_height());
  CHECK(fast_bird_view_param.use_log_count());
  CHECK(fast_bird_view_param.use_direction());
  CHECK(fast_bird_view_param.use_top_intensity());
  CHECK(fast_bird_view_param.use_mean_intensity());
  CHECK(fast_bird_view_param.use_distance());
  CHECK(fast_bird_view_param.use_nonempty());
  _data_channel = fast_bird_view_param.channels();
  CHECK(fast_bird_view_param.use_height_filter());

  _use_intensity_feature = fast_bird_view_param.use_intensity_feature();
  if (!_use_intensity_feature) {
      CHECK(_data_channel == 6);
  }

  // set output blob and log lookup table
#ifndef USE_MICROCAR_CAFFE_MODEL
  out_blob_->Reshape(1, 8, height_, width_);
#else
  out_blob_->Reshape(1, 6, height_, width_);
#endif

  log_table_.resize(256);
  for (size_t i = 0; i < log_table_.size(); ++i) {
    //log_table_[i] = std::log1p(static_cast<Dtype>(i));
    log_table_[i] = log(1 + i);
  }

  _log_blob.reset(new caffe::Blob<Dtype>(1, 1, 1, log_table_.size()));
  Dtype *log_table = _log_blob->mutable_gpu_data();
  caffe::caffe_copy(log_table_.size(), log_table_.data(), log_table);

  Dtype* out_blob_data = nullptr;
  out_blob_data = out_blob_->mutable_cpu_data();

  int channel_index = 0;
  max_height_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  mean_height_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  count_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  direction_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
#ifndef USE_MICROCAR_CAFFE_MODEL
  if (_use_intensity_feature) {
  top_intensity_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  mean_intensity_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  }
#endif
  distance_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  nonempty_data_ = out_blob_data + out_blob_->offset(0, channel_index++);
  CHECK_EQ(out_blob_->offset(0, channel_index), out_blob_->count());

  CHECK(fast_bird_view_param.use_dense_feat());

  // compute direction and distance features
  int siz = height_ * width_;
  #if 0
  vector<Dtype> direction_data(siz);
  vector<Dtype> distance_data(siz);

  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      int idx = row * width_ + col;
      // * row <-> x, column <-> y
      float center_x = Pixel2Pc(row, height_, range_);
      float center_y = Pixel2Pc(col, width_, range_);
      constexpr double K_CV_PI = 3.1415926535897932384626433832795;
      direction_data[idx] =
          static_cast<Dtype>(std::atan2(center_y, center_x) / (2.0 * K_CV_PI));
      distance_data[idx] =
          static_cast<Dtype>(std::hypot(center_x, center_y) / 60.0 - 0.5);
    }
  }
  caffe::caffe_copy(siz, direction_data.data(), direction_data_);
  caffe::caffe_copy(siz, distance_data.data(), distance_data_);
  #else
  Dtype direction_data[siz];
  Dtype distance_data[siz];

  for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
          int idx = row * width_ + col;
          float center_x = Pixel2Pc(row, height_, range_);
          float center_y = Pixel2Pc(col, width_, range_);
          constexpr double K_CV_PI = 3.1415926535897932384626433832795;
          direction_data[idx] = std::atan2(center_y, center_x) / (2 * K_CV_PI);
          distance_data[idx] = std::hypot(center_x, center_y) / 60.0 - 0.5;
      }
  }
  caffe::caffe_copy(siz, direction_data, direction_data_);
  caffe::caffe_copy(siz, distance_data, distance_data_);
  #endif

  return true;
}

template <typename Dtype>
void FastFeatureGenerator<Dtype>::Generate(
    apollo::perception::pcl_util::PointCloudConstPtr pc_ptr) {

  const auto& points = pc_ptr->points;

  // DO NOT remove this line!!!
  // Otherwise, the gpu_data will not be updated for the later frames.
  // It marks the head at cpu for blob.
  out_blob_->mutable_cpu_data();

  int siz = height_ * width_;
  caffe::caffe_set(siz, Dtype(-5), max_height_data_);
  caffe::caffe_set(siz, Dtype(0), mean_height_data_);
  caffe::caffe_set(siz, Dtype(0), count_data_);
#ifndef USE_MICROCAR_CAFFE_MODEL
  caffe::caffe_set(siz, Dtype(0), top_intensity_data_);
  caffe::caffe_set(siz, Dtype(0), mean_intensity_data_);
#endif
  caffe::caffe_set(siz, Dtype(0), nonempty_data_);

  map_idx_.resize(points.size());
  float inv_res_x =
      0.5 * static_cast<float>(width_) / static_cast<float>(range_);
  float inv_res_y =
      0.5 * static_cast<float>(height_) / static_cast<float>(range_);

  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i].z <= min_height_ || points[i].z >= max_height_) {
      map_idx_[i] = -1;
      continue;
    }
    // * the coordinates of x and y are exchanged here
    // (row <-> x, column <-> y)
    int pos_x = F2I(points[i].y, range_, inv_res_x);  // col
    int pos_y = F2I(points[i].x, range_, inv_res_y);  // row
    if (pos_x >= width_ || pos_x < 0 || pos_y >= height_ || pos_y < 0) {
      map_idx_[i] = -1;
      continue;
    }
    map_idx_[i] = pos_y * width_ + pos_x;

    int idx = map_idx_[i];
    float pz = points[i].z;
    float pi = points[i].intensity / 255.0;
    if (max_height_data_[idx] < pz) {
      max_height_data_[idx] = pz;
#ifndef USE_MICROCAR_CAFFE_MODEL
      top_intensity_data_[idx] = pi;
#endif
    }
    mean_height_data_[idx] += static_cast<Dtype>(pz);
#ifndef USE_MICROCAR_CAFFE_MODEL
    mean_intensity_data_[idx] += static_cast<Dtype>(pi);
#endif
    count_data_[idx] += Dtype(1);
  }

  for (int i = 0; i < siz; ++i) {
    constexpr double EPS = 1e-6;
    if (count_data_[i] < EPS) {
      max_height_data_[i] = Dtype(0);
    } else {
      mean_height_data_[i] /= count_data_[i];
#ifndef USE_MICROCAR_CAFFE_MODEL
      mean_intensity_data_[i] /= count_data_[i];
#endif
      nonempty_data_[i] = Dtype(1);
    }
    count_data_[i] = LogCount(static_cast<int>(count_data_[i]));
  }
}

template bool FastFeatureGenerator<float>::Init(const FastFeatureParam& feature_param,
                                            caffe::Blob<float>* blob);

template void FastFeatureGenerator<float>::Generate(
    apollo::perception::pcl_util::PointCloudConstPtr pc_ptr);

template bool FastFeatureGenerator<double>::Init(const FastFeatureParam& feature_param,
                                             caffe::Blob<double>* blob);

template void FastFeatureGenerator<double>::Generate(
    apollo::perception::pcl_util::PointCloudConstPtr pc_ptr);

}  // namespace cnnseg
}  // namespace perception
}  // namespace apollo
