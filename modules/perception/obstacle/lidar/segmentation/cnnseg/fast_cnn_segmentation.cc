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

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/fast_cnn_segmentation.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"

#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/common/util.h"

namespace apollo {
namespace perception {

DEFINE_int32(cnnseg_gpu, 0, "device id for cnnseg");

bool FastCNNSegmentation::Init() {
    std::string config_file;
    std::string proto_file;
    std::string weight_file;
    std::string feature_config;
    std::string post_config;
    if (!get_configs(config_file, proto_file, weight_file, feature_config, post_config)) {
        return false;
    }
    AINFO << "--    config_file: " << config_file;
    AINFO << "--     proto_file: " << proto_file;
    AINFO << "--    weight_file: " << weight_file;
    AINFO << "-- feature_config: " << feature_config;
    AINFO << "--    post_config: " << post_config;

    cnnseg::FastFeatureParam fast_feature_param;
    load_text_proto_message_file(feature_config, fast_feature_param);
    load_text_proto_message_file(config_file, _fast_cnnseg_param);
    const auto &model_type = _fast_cnnseg_param.model_type();
    AINFO << "--     model_type: " << model_type;

#ifdef USE_GPU
    if (FLAGS_cnnseg_gpu >= 0) {
        caffe::Caffe::SetDevice(FLAGS_cnnseg_gpu);
        caffe::Caffe::set_mode(caffe::Caffe::GPU);
        caffe::Caffe::DeviceQuery();
    } else {
#endif
        caffe::Caffe::set_mode(caffe::Caffe::CPU);
#ifdef USE_GPU
    }
#endif

    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    // init Net
    auto const &fast_net_param = _fast_cnnseg_param.fast_net_param();
    input_names.push_back(fast_net_param.feature_blob());
    output_names.push_back(fast_net_param.instance_pt_blob());
    output_names.push_back(fast_net_param.category_pt_blob());
    output_names.push_back(fast_net_param.confidence_pt_blob());
    output_names.push_back(fast_net_param.classify_pt_blob());
    output_names.push_back(fast_net_param.heading_pt_blob());
    output_names.push_back(fast_net_param.height_pt_blob());

    _range = fast_feature_param.fast_bird_view_param().point_cloud_range();
    _width = fast_feature_param.fast_bird_view_param().width();
    _height = fast_feature_param.fast_bird_view_param().height();
    _channels = fast_feature_param.fast_bird_view_param().channels();

    AINFO << "--    fast_feature_param.fast_bird_view_param()._range: " << _range;
    AINFO << "--    fast_feature_param.fast_bird_view_param()._width: " << _width;
    AINFO << "--    fast_feature_param.fast_bird_view_param()._height: " << _height;
    AINFO << "--    fast_feature_param.fast_bird_view_param()._channels: " << _channels;

    _cnnadapter.reset(new CNNCaffe);
    if (!_cnnadapter->init(input_names, output_names, proto_file, weight_file, 0, "")) {
        return false;
    }
    if(!_cnnadapter->reshape_input(fast_net_param.feature_blob(), {1, _channels, _height, _width}))
    {
        return false;
    }

    _instance_pt_blob = _cnnadapter->get_blob_by_name(fast_net_param.instance_pt_blob());
    _category_pt_blob = _cnnadapter->get_blob_by_name(fast_net_param.category_pt_blob());
    _confidence_pt_blob = _cnnadapter->get_blob_by_name(fast_net_param.confidence_pt_blob());
    _classify_pt_blob = _cnnadapter->get_blob_by_name(fast_net_param.classify_pt_blob());
    _heading_pt_blob = _cnnadapter->get_blob_by_name(fast_net_param.heading_pt_blob());
    _height_pt_blob = _cnnadapter->get_blob_by_name(fast_net_param.height_pt_blob());
    _feature_blob = _cnnadapter->get_blob_by_name(fast_net_param.feature_blob());

    CHECK(nullptr != _instance_pt_blob) << "`" << fast_net_param.instance_pt_blob() << "` not exists!";
    CHECK(nullptr != _category_pt_blob) << "`" << fast_net_param.category_pt_blob() << "` not exists!";
    if (_fast_cnnseg_param.do_classification()) {
        CHECK(nullptr != _classify_pt_blob) << "`" << fast_net_param.classify_pt_blob() << "` not exists!";
    }
    AINFO << "--    _fast_cnnseg_param.do_classification(): " << _fast_cnnseg_param.do_classification();
    if (_fast_cnnseg_param.do_heading()) {
        CHECK(nullptr != _heading_pt_blob) << "`" << fast_net_param.heading_pt_blob() << "` not exists!";
    }
    AINFO << "--    _fast_cnnseg_param.do_heading(): " << _fast_cnnseg_param.do_heading();
    CHECK(nullptr != _feature_blob) << "`" << fast_net_param.feature_blob() << "` not exists!";

    _fast_cluster2d.init(_height, _width, _range);

    _fast_feature_generator.reset(new cnnseg::FastFeatureGenerator<float>());
    if (!_fast_feature_generator->Init(fast_feature_param, _feature_blob.get())) {
        return false;
    }
    AINFO << "Using " << _fast_feature_generator->name();

    _num_classes = 5;
    _type_map.resize(_num_classes);
    _type_map[0] = INT_UNKNOWN;
    _type_map[1] = INT_SMALLMOT;
    _type_map[2] = INT_BIGMOT;
    _type_map[3] = INT_NONMOT;
    _type_map[4] = INT_PEDESTRIAN;

    return true;
}

bool FastCNNSegmentation::Segment(pcl_util::PointCloudPtr pc_ptr,
                              const pcl_util::PointIndices &non_ground_indices,
                              const SegmentationOptions &options,
                              std::vector<std::shared_ptr<Object>>* objects) {
    PERF_FUNCTION();
    objects->clear();
    int num_pts = pc_ptr->points.size();
    if (num_pts == 0) {// || non_ground_indices.indices.size() == 0) {
        return true;
    }

#ifdef USE_GPU
    if (FLAGS_cnnseg_gpu >= 0) {
        caffe::Caffe::SetDevice(FLAGS_cnnseg_gpu);
    }
#endif

    PERF_BLOCK_START();
    _fast_feature_generator->Generate(options.origin_cloud);

    _cnnadapter->forward();

    PERF_BLOCK_END("vlp_cnn_seg");

    _fast_cluster2d.cluster(_category_pt_blob->cpu_data(), _instance_pt_blob->cpu_data());
    _fast_cluster2d.classify(_classify_pt_blob->cpu_data(), _confidence_pt_blob->cpu_data(),
                        _heading_pt_blob->cpu_data(), _height_pt_blob->cpu_data(),
                        _num_classes);
    _fast_cluster2d.get_objects(pc_ptr, _fast_cnnseg_param.confidence_thresh(),
                        _fast_cnnseg_param.height_thresh(), _type_map,
                        objects);
    //AINFO << "************************ objects->size(): " << objects->size();
    //AINFO << "************************ _fast_cnnseg_param.confidence_thresh(): " << _fast_cnnseg_param.confidence_thresh();
    //AINFO << "************************ _fast_cnnseg_param.height_thresh(): " << _fast_cnnseg_param.height_thresh();

    return true;
}

bool FastCNNSegmentation::get_configs(std::string& config_file, std::string& proto_file,
                                  std::string& weight_file, std::string& feature_config,
                                  std::string& post_config) {
    config_manager::ConfigManager *config_manager =
        Singleton<config_manager::ConfigManager>::get();
    CHECK_NOTNULL(config_manager);

    const config_manager::ModelConfig *model_config = nullptr;
    if (!config_manager->get_model_config("CNNSegmentation", &model_config)) {
        AERROR << "Failed to get model config: cnn_segmentation";
        return false;
    }
    const std::string &work_root = config_manager->work_root();

    if (!model_config->get_value("config_file", &config_file)) {
        AERROR << "Failed to get value of config_file.";
        return false;
    }
    config_file = FileUtil::get_absolute_path(work_root, config_file);

    if (!model_config->get_value("proto_file", &proto_file)) {
        AERROR << "Failed to get value of proto_file.";
        return false;
    }
    proto_file = FileUtil::get_absolute_path(work_root, proto_file);

    if (!model_config->get_value("weight_file", &weight_file)) {
        AERROR << "Failed to get value of weight_file.";
        return false;
    }
    weight_file = FileUtil::get_absolute_path(work_root, weight_file);

    if (!model_config->get_value("feature_config", &feature_config)) {
        AERROR << "Failed to get value of feature_config.";
        return false;
    }
    feature_config = FileUtil::get_absolute_path(work_root, feature_config);

    if (!model_config->get_value("post_config", &post_config)) {
        AERROR << "Failed to get value of post_config.";
        return false;
    }
    post_config = FileUtil::get_absolute_path(work_root, post_config);

    return true;
}

}  // namespace perception
}  // namespace apollo
