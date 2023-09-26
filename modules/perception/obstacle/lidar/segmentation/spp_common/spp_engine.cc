#include "modules/common/log.h"

#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_engine.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/timer.h"

#if 0
namespace {
    void print_cloud(const adu::perception::pcl_util::PointCloudPtr& cloud,
                     std::string info) {
        if (cloud->points.size() == 0) {
            AINFO << "Cloud size is 0.";
            return;
        }
        for (int i = 0; i < cloud->points.size(); ++i) {
            perception::pcl_util::Point p = cloud->points[i];
            AINFO << info << " [" << i << "] x y z: "
                     << p.x << " "
                     << p.y << " "
                     << p.z; 
        }
    }
}
#endif

namespace apollo {
namespace perception {

void SppData::make_reference(unsigned int width, unsigned int height, float range) {
    if (obs_prob_data != nullptr) {
        if (obs_prob_data_ref == nullptr) {
            obs_prob_data_ref = new float*[height];
        }
        for (unsigned int i = 0; i < height; ++i) {
            obs_prob_data_ref[i] = obs_prob_data + i * width;
        }
    }
    data_width = width;
    data_height = height;
    data_range = range;
    data_size = width * height;
    if (transformed_offset_data_ref == nullptr) {
        transformed_offset_data_ref = new float[data_size * 2];
    }
}

void SppData::transform_offset_data() {
    Timer timer;
    timer.start();
    float scale = static_cast<float>(data_height) / (2.f * data_range);
    float* ptr1 = offset_data;
    float* ptr2 = offset_data + data_size;
    for (unsigned int i = 0, j = 0; i < data_size; ++i, j += 2) {
        transformed_offset_data_ref[j] = ptr1[i] * scale;
        transformed_offset_data_ref[j+1] = ptr2[i] * scale;
    }
}

SppData::~SppData() {
    if (obs_prob_data_ref) {
        delete[] obs_prob_data_ref;
    }
    if (transformed_offset_data_ref) {
        delete[] transformed_offset_data_ref;
    }
}

void SppEngine::init(unsigned int width, unsigned int height, float range) {
    //if (!_detector_2d_kmeans) {
    //    _detector_2d_kmeans = new KMeansDetector();
    //}
    if (!_detector_2d_cc) {
        _detector_2d_cc = new CCDetector();
    }
    //_detector_2d_kmeans->init(width, height, 1);
    _detector_2d_cc->init(width, height);
    _labels_2d.init(width, height);

    _width = width;
    _height = height;
    _range = range;

    _mst.init(width, height, 75.f, _configure._mst_c);
    _sequence.init(10);
    _roughness_sequence.init(10);

    /*
    _mst_seq.init(_configure._mst_width,
            _configure._mst_height,
            _configure._mst_resolution,
            _configure._mst_c);
    */
    _mst_seq.init(_configure._mst_width,
            _configure._mst_height,
            _configure._mst_resolution,
            _configure._mst_c,
            _configure._mst_max_dist_threshold);

    _roughness_mst_seq.init(_configure._mst_width,
            _configure._mst_height,
            _configure._mst_resolution,
            _configure._mst_c,
            _configure._mst_max_dist_threshold);
}

void SppEngine::init(unsigned int width, unsigned int height, float range,
        const std::string& config_file) {
    _configure.load_from_file(config_file);
    init(width, height, range);
}

unsigned int SppEngine::process(const SppData* data,
        pcl_util::PointCloudConstPtr point_cloud,
        const SppCloudMask& mask) {
    // 1. process 2d
    // Note: label start from 1, 0 means background
    if (_configure._c2d_method == "kmeans") {
        AINFO << "Process2D method: kmeans";
        //_detector_2d_kmeans->detect(data->obs_prob_data_ref,
        //        data->transformed_offset_data_ref,
        //        _labels_2d.get_label_map()[0], nullptr,
        //        _configure._c2d_prob_cutoff_contribute,
        //        _configure._c2d_prob_cutoff_seed,
        //        _configure._c2d_prob_cutoff_obj,
        //        _configure._c2d_prob_cutoff_diff,
        //        _configure._c2d_cc_to_center_dist,
        //        _configure._c2d_center_to_center_dist,
        //        _configure._c2d_occ_neighbor,
        //        true);
        //_labels_2d.from_label_map();
    }
    else {
        AINFO << "Process2D method: connected component";
        _detector_2d_cc->detect(data->obs_prob_data_ref,
                data->transformed_offset_data_ref, &_labels_2d);
    }
    _labels_2d.filter_label_map(data->confidence_data, data->confidence_threshold);
    if (data->class_prob_data != nullptr) {
        _labels_2d.cal_label_class(data->class_prob_data, data->class_num);
    }
    if (data->heading_data != nullptr) {
        _labels_2d.cal_label_heading(data->heading_data);
    }

    //dilate operate do not affect heading and classify but top_z
    _labels_2d.dilate_erode_occs(1);

    if (data->z_data != nullptr) {
        _labels_2d.cal_label_top_z(data->z_data);
    }
    // 2. process 2d to 3d
    unsigned int size = _width * _height;
    unsigned short max_label = static_cast<unsigned short>(_labels_2d.get_obstacle_num());
    _clusters.init(max_label + 1); // the first cluster will be empty
    float half_width = static_cast<float>(_width) / 2;
    float half_height = static_cast<float>(_height) / 2;
    int x = 0;
    int y = 0;
    float ratio_x = half_width / _range;
    float ratio_y = half_height / _range;
    int width = static_cast<int>(_width);
    int height = static_cast<int>(_height);

    for (unsigned int i = 0; i < point_cloud->points.size(); ++i) {
        if (mask.size() && mask[i] == 0) {
            continue;
        }
        const pcl_util::Point& point = point_cloud->points[i];
        y = static_cast<int>((_range - point.x) * ratio_x + 1.f) - 1;
        x = static_cast<int>((_range - point.y) * ratio_y + 1.f) - 1;

        if (x < 0 || x >= width || y < 0 || y >= height) {
            continue;
        }
        unsigned short& label = _labels_2d[y][x];
        if (!label) {
            continue;
        }
        if (point.z <= _labels_2d.get_obstacle(label-1)->get_top_z() + data->top_z_threshold) {
            _clusters.add_sample(label, point, i);
        }
    }
    for (unsigned int i = 1; i < _clusters.size(); ++i) {
        _clusters[i].yaw = _labels_2d.get_obstacle(i-1)->get_yaw();
        _clusters[i].confidence = _labels_2d.get_obstacle(i-1)->get_confidence();
        _clusters[i].class_prob = _labels_2d.get_obstacle(i-1)->get_probs();
    }
    // 4. split cluster along z axis
    //unsigned int nr_split = _clusters.analyze_clusters(_configure._c3d_height_gap, 1, true);
    // 5. remove empty clusters
    _clusters.remove_empty_clusters();

    return _clusters.size();
}

unsigned int SppEngine::process_mst(
        pcl_util::PointCloudConstPtr point_cloud,
        const SppCloudMask& mask) {
    _mst.reset();
    _mst_clusters.reset();
    _mst.add_point_cloud(point_cloud, mask);
    _mst.segmentation(point_cloud, mask, _mst_clusters,
            _configure._mst_neighbor, false);
    _mst_clusters.remove_empty_clusters();
    _mst_clusters.analyze_clusters(_configure._c3d_height_gap, 0, false);
    return _mst_clusters.size();
}

unsigned int SppEngine::process_sequence_mst(pcl_util::PointCloudConstPtr point_cloud,
        SppCloudMask* mask,
        const Eigen::Matrix4d& pose_v2w) {
    Timer timer;
    timer.start();
    _mst_clusters.reset();

    // 1. add frame to sequence
    SppFramePtr frame(new SppFrame);
    frame->pose_v2w = pose_v2w;
    _sequence.add_frame(frame);

    // 2. query map and move fp points
    SppDynamicMap& map = _mst_seq.get_dynamic_map();
    unsigned int n = 0;
    unsigned int added = 0;
    unsigned int valid = mask->valid_indices_count();
    std::set<int> indices;
    while (n < _clusters.size()) {
        auto& cluster = _clusters.get_clusters()[n];
        _sequence.transform_cloud_to_world(&cluster);
        float ratio = map.query_points_in_map_ratio(cluster, &indices);
        if (ratio > _configure._mst_move_threshold) {
            mask->add_indices(*cluster.points, 2); // distinguish from existing value
            added += cluster.points->size();
            _clusters.erase_cluster(n);
        } else { // remove movable map points
            map.clean_map(indices);
            ++n;
        }
    }

    // 3. get world frame and update map (without fp points)
    frame->cloud = mask->get_valid_cloud(point_cloud);
    pcl_util::PointCloudPtr world_cloud = _sequence.get_latest_world_cloud();
    SppCloudMask world_mask;
    mask->get_valid_mask(&world_mask);
    world_mask.reset_value(2, 0); // mask this value

    _mst_seq.add_point_cloud(world_cloud, world_mask, frame->pose_v2w);
    timer.end("Update map ");

    // 4. segment map
    _mst_seq.segmentation(point_cloud, *mask, _mst_clusters,
            _configure._mst_neighbor);

    // 5. split cluster along z axis
    _mst_clusters.remove_empty_clusters();
    _mst_clusters.analyze_clusters(_configure._c3d_height_gap, 0, false);
    timer.end("Core ");

    return _mst_clusters.size();
}

unsigned int SppEngine::process(const SppData* data,
        pcl_util::PointCloudConstPtr point_cloud) {
    _mask.clear();
    process(data, point_cloud, _mask);
    AINFO << "Foreground: " << _clusters.size() << " clusters";
    return _clusters.size();
}

unsigned int SppEngine::process_roughness_sequence_mst(pcl_util::PointCloudConstPtr point_cloud,
        SppCloudMask* mask,
        const Eigen::Matrix4d& pose_v2w) {
    Timer timer;
    timer.start();
    _roughness_mst_clusters.reset();

    // 1. add frame to sequence
    SppFramePtr frame(new SppFrame);
    frame->pose_v2w = pose_v2w;
    _roughness_sequence.add_frame(frame);

    // 3. get world frame and update map (without fp points)
    frame->cloud = mask->get_valid_cloud(point_cloud);
    
    //pcl_util::PointCloudPtr clouds(new pcl_util::PointCloud);
    _his_full_point_cloud.reset(new pcl_util::PointCloud);
    pcl_util::PointCloudPtr his_clouds = _roughness_sequence.get_fused_cloud(_history_roughness_size);
    SppCloudMask tmp_mask;
    tmp_mask.set_size(point_cloud->points.size() + his_clouds->points.size() - 
                      frame->cloud->points.size(), 0);
    for (int i = 0; i < point_cloud->points.size(); ++i) {
        _his_full_point_cloud->points.push_back(point_cloud->points[i]);
        tmp_mask[i] = (*mask)[i];
    }
    for (int i = frame->cloud->points.size(); i < his_clouds->points.size(); ++i) {
        _his_full_point_cloud->points.push_back(his_clouds->points[i]);
        tmp_mask[i - frame->cloud->points.size() + point_cloud->points.size()] = 1;
    }

    //print_cloud(frame->cloud, "original_roughness_cloud");

    //pcl_util::PointCloudPtr world_cloud = _roughness_sequence.get_latest_world_cloud();
    pcl_util::PointCloudPtr world_cloud = _roughness_sequence.get_world_fused_cloud(_history_roughness_size);
    //print_cloud(world_cloud, "world_roughness_cloud");
    SppCloudMask world_mask;
    tmp_mask.get_valid_mask(&world_mask);
    //world_mask.reset_value(2, 0); // mask this value

    _roughness_mst_seq.add_point_cloud(world_cloud, world_mask, frame->pose_v2w);
    timer.end("Update map ");

    // 4. segment map
    //_roughness_mst_seq.segmentation(point_cloud, *mask, _roughness_mst_clusters,
    //        _configure._mst_neighbor);
    _roughness_mst_seq.segmentation(_his_full_point_cloud, tmp_mask, _roughness_mst_clusters,
            _configure._mst_neighbor);

    // 5. split cluster along z axis
    _roughness_mst_clusters.remove_empty_clusters();
    _roughness_mst_clusters.analyze_clusters(_configure._c3d_height_gap, 0, false);
    timer.end("Core ");

    return _roughness_mst_clusters.size();
}

//std::pair<unsigned int, unsigned int> SppEngine::process_joint_cloud(const SppData* data,
std::vector<unsigned int> SppEngine::process_joint_cloud(const SppData* data,
        pcl_util::PointCloudConstPtr full_point_cloud,
        pcl_util::PointCloudPtr& his_full_point_cloud,
        pcl_util::PointIndicesPtr roi_indices,
        pcl_util::PointIndicesPtr roi_non_ground_indices,
        pcl_util::PointIndicesPtr roughness_indices,
        const int history_roughness_size,
        const Eigen::Matrix4d* pose_v2w) {

    _mask.clear();
    Timer timer;
    timer.start();
    _mask.set_size(full_point_cloud->points.size(), 0);
    _mask.add_indices(roi_indices);
    _mask.minus_indices_2nd(roi_indices, roi_non_ground_indices);
    _mask.flip();
    process(data, full_point_cloud, _mask);

   // MST for roughness cloud.
    _history_roughness_size = history_roughness_size;
    _roughness_mask.clear();
    _roughness_mask.set_size(full_point_cloud->points.size(), 0);
    _roughness_mask.add_indices_2nd(roi_indices, roughness_indices);
    _roughness_mask.minus_indices_2nd(roi_indices, roi_non_ground_indices);
    for (auto& cluster : _clusters.get_clusters()) {
        _roughness_mask.minus_indices(*cluster.points);
    }
    process_roughness_sequence_mst(full_point_cloud, &_roughness_mask, *pose_v2w);
    pcl::copyPointCloud(*_his_full_point_cloud, *his_full_point_cloud);

    _mask.clear();
    timer.end("2D post processing time ");

    _mask.set_size(full_point_cloud->points.size(), 0);
    _mask.add_indices_2nd(roi_indices, roi_non_ground_indices);

    for (auto& cluster : _clusters.get_clusters()) {
        _mask.minus_indices(*cluster.points);
    }
    timer.end("Compute mask time ");
    //process_mst(full_point_cloud, _mask);
    if (pose_v2w == nullptr) {
        AINFO << "Process frame mst";
        process_mst(full_point_cloud, _mask);
    } else {
        AINFO << "Process sequence mst";
        process_sequence_mst(full_point_cloud, &_mask, *pose_v2w);
    }
    timer.end("Mst time ");
    //std::pair<unsigned int, unsigned int> num;
    //num.first = _clusters.size();
    //num.second = _mst_clusters.size();
    std::vector<unsigned int> num(3);
    num[0] = _clusters.size();
    num[1] = _mst_clusters.size();
    num[2] = _roughness_mst_clusters.size();
    _clusters.merge(_mst_clusters);
    _clusters.merge(_roughness_mst_clusters);
    AINFO << "clusters size: " << _clusters.size();
    AINFO << "Foreground: " << num[0] << " clusters";
    AINFO << "Background: " << num[1] << " clusters";
    AINFO << "Roughness Background: " << num[2] << " clusters";


    return num;
}

}  // namespace perception
}  // namespace apollo
