#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CONFIGURE_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CONFIGURE_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace apollo {
namespace perception {

class SppConfigure {
public:
    SppConfigure() {
        _c2d_method = "kmeans"; // "cc"
        _c2d_prob_cutoff_contribute = 0.33f;
        _c2d_prob_cutoff_seed = 0.3f;
        _c2d_prob_cutoff_obj = 0.5f;
        _c2d_prob_cutoff_diff = 0.24f;
        _c2d_cc_to_center_dist = 12.f;
        _c2d_center_to_center_dist = 3.f;
        _c2d_occ_neighbor = 8.f;

        _c3d_height_gap = 0.5f;

        _mst_width = 480;
        _mst_height = 480;
        _mst_resolution = 0.3f;

        _mst_neighbor = 2;
        _mst_c = 8.f;

        _mst_move_threshold = 0.9;

        _mst_max_dist_threshold = 5.0f;
    }

    bool load_from_file(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["Clustering2D"]) {
            _c2d_method = config["Clustering2D"]["method"].as<std::string>();
            _c2d_prob_cutoff_contribute = config["Clustering2D"]["contribute"].as<float>();
            _c2d_prob_cutoff_seed = config["Clustering2D"]["seed"].as<float>();
            _c2d_prob_cutoff_obj = config["Clustering2D"]["obj"].as<float>();
            _c2d_prob_cutoff_diff = config["Clustering2D"]["diff"].as<float>();
            _c2d_cc_to_center_dist = config["Clustering2D"]["cc2center"].as<double>();
            _c2d_center_to_center_dist = config["Clustering2D"]["center2center"].as<float>();
            _c2d_occ_neighbor = config["Clustering2D"]["neighbor"].as<unsigned int>();
        }
        else {
            return false;
        }
        if (config["Clustering3D"]) {
            _c3d_height_gap = config["Clustering3D"]["gap"].as<float>();
        }
        else {
            return false;
        }
        if (config["Mst"]) {
            _mst_width = config["Mst"]["width"].as<unsigned int>();
            _mst_height = config["Mst"]["height"].as<unsigned int>();
            _mst_resolution = config["Mst"]["resolution"].as<float>();
            _mst_neighbor = config["Mst"]["neighbor"].as<unsigned int>();
            _mst_c = config["Mst"]["c"].as<float>();
            _mst_move_threshold = config["Mst"]["move"].as<float>();
            _mst_max_dist_threshold = config["Mst"]["max_dist_threshold"].as<float>();
        }
        else {
            return false;
        }
        return true;
    }

    void write_to_file(const std::string& filename) const {
        YAML::Node config;
        config["Clustering2D"]["method"] = _c2d_method;
        config["Clustering2D"]["contribute"] = _c2d_prob_cutoff_contribute;
        config["Clustering2D"]["seed"] = _c2d_prob_cutoff_seed;
        config["Clustering2D"]["obj"] = _c2d_prob_cutoff_obj;
        config["Clustering2D"]["diff"] = _c2d_prob_cutoff_diff;
        config["Clustering2D"]["cc2center"] = _c2d_cc_to_center_dist;
        config["Clustering2D"]["center2center"] = _c2d_center_to_center_dist;
        config["Clustering2D"]["neighbor"] = _c2d_occ_neighbor;

        config["Clustering3D"]["gap"] = _c3d_height_gap;

        config["Mst"]["width"] = _mst_width;
        config["Mst"]["height"] = _mst_height;
        config["Mst"]["resolution"] = _mst_resolution;
        config["Mst"]["neighbor"] = _mst_neighbor;
        config["Mst"]["c"] = _mst_c;
        config["Mst"]["move"] = _mst_move_threshold;
        config["Mst"]["max_dist_threshold"] = _mst_max_dist_threshold;
        std::ofstream fout(filename);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }

    }
public:
    //Clustering 2D
    std::string _c2d_method;
    float _c2d_prob_cutoff_contribute;
    float _c2d_prob_cutoff_seed;
    float _c2d_prob_cutoff_obj;
    float _c2d_prob_cutoff_diff;
    double _c2d_cc_to_center_dist;
    float _c2d_center_to_center_dist;
    unsigned int _c2d_occ_neighbor;

    //Clustering 3D
    float _c3d_height_gap;

    //Mst segmentation
    unsigned int _mst_width;
    unsigned int _mst_height;
    float _mst_resolution;

    unsigned int _mst_neighbor;
    float _mst_c;
    float _mst_move_threshold;

    float _mst_max_dist_threshold;
};

}  // namespace perception
}  // namespace apollo

#endif
