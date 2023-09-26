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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FAST_CLUSTER2D_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FAST_CLUSTER2D_H_

#include <algorithm>
#include <memory>
#include <vector>

#include "caffe/caffe.hpp"

#include "modules/common/log.h"
#include "modules/common/util/disjoint_set.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/util.h"

#include "modules/perception/obstacle/lidar/segmentation/spp_common/i_alloc.h"

namespace apollo {
namespace perception {
namespace cnnseg {

using apollo::common::util::DisjointSetFind;
using apollo::common::util::DisjointSetMakeSet;
using apollo::common::util::DisjointSetUnion;

enum class FastMetaType {
  META_UNKNOWN,
  META_SMALLMOT,
  META_BIGMOT,
  META_NONMOT,
  META_PEDESTRIAN,
  MAX_META_TYPE
};

struct FastObstacle {
    std::vector<int> grids;
    std::vector<float> class_score;
    pcl_util::PointCloudPtr cloud;
    int class_id;
    float score;
    float angle;
    float height;

    FastMetaType meta_type;
    std::vector<float> meta_type_probs;

    FastObstacle() : score(0.0), height(-5.0), meta_type(FastMetaType::META_UNKNOWN) {
        cloud.reset(new apollo::perception::pcl_util::PointCloud);
        meta_type_probs.assign(static_cast<int>(FastMetaType::MAX_META_TYPE), 0.0);
    }
};

class FastCluster2D {
public:
    FastCluster2D() : _id_img(nullptr) {
    }
    ~FastCluster2D() {
        if (_id_img) {
            idl::i_free2(_id_img);
        }
    }
    void init(const int rows, const int cols, const float range) {
        _rows = rows;
        _cols = cols;
        _mapsize = _rows * _cols;
        _range = range;
        _scale = 0.5 * _rows / _range;
        _inv_res_x = 0.5 * _cols / _range;
        _inv_res_y = 0.5 * _rows / _range;
        if (_id_img) {
            idl::i_free2(_id_img);
        }
        _id_img = idl::i_alloc2<unsigned short>(rows, cols);
    }
    
    void cluster(const float* category_pt_data,
                const float* instance_pt_data) {
        const float* instance_pt_x_data = instance_pt_data;
        const float* instance_pt_y_data = instance_pt_data + _mapsize;
        std::vector<std::vector<Node> > _nodes;
        _nodes.assign(_rows, std::vector<Node>(_cols, Node()));
        for (int row = 0; row < _rows; row++) {
            for (int col = 0; col < _cols; col++) {
                int offset = row * _cols + col;
                Node* node = &_nodes[row][col];
                DisjointSetMakeSet(node);
                node->is_object = *(category_pt_data + offset) >= 0.5;
                int center_row = round(row + instance_pt_x_data[offset] * _scale);
                int center_col = round(col + instance_pt_y_data[offset] * _scale);
                center_row = std::min(std::max(center_row, 0), _rows - 1);
                center_col = std::min(std::max(center_col, 0), _cols - 1);
                node->center_node = &_nodes[center_row][center_col];
            }
        }
        for (int row = 0; row < _rows; row++) {
            for (int col = 0; col < _cols; col++) {
                Node* node = &_nodes[row][col];
                if (node->is_object && node->traversed == 0) {
                    traverse(node);
                }
            }
        }
        for (int row = 0; row < _rows; row++) {
            for (int col = 0; col < _cols; col++) {
                Node* node = &_nodes[row][col];
                if (!node->is_center) {
                    continue;
                }
                for (int row2 = row - 1; row2 <= row + 1; row2++) {
                    for (int col2 = col - 1; col2 <= col + 1; col2++) {
                        if (row2 >= 0 && row2 < _rows && col2 >= 0 && col2 < _cols) {
                            Node* node2 = &_nodes[row2][col2];
                            if (node2->is_center) {
                                DisjointSetUnion(node, node2);
                            }
                        }
                    }
                }
            }
        }
        int id = 0;
        _obstacles.clear();
        memset(_id_img[0], 0, sizeof(unsigned short) * _rows * _cols);
        for (int row = 0; row < _rows; row++) {
            for (int col = 0; col < _cols; col++) {
                Node* node = &_nodes[row][col];
                if (!node->is_object) {
                    continue;
                }
                Node* root = DisjointSetFind(node);
                if (root->obstacle_id < 0) {
                    id++;
                    root->obstacle_id = id;
                    FastObstacle obs;
                    _obstacles.push_back(obs);
                }
                _id_img[row][col] = static_cast<unsigned short>(root->obstacle_id);
                _obstacles[root->obstacle_id - 1].grids.push_back(row * _cols + col);
            }
        }
    }
    
    void filter(const float* confidence_pt_data,
                const float* height_pt_data) {
        for (size_t obs_id = 0; obs_id < _obstacles.size(); obs_id++) {
            FastObstacle* obs = &_obstacles[obs_id];
            double score = 0.0;
            double height = 0.0;
            for (size_t grid_id = 0; grid_id < obs->grids.size(); grid_id++) {
                int grid = obs->grids[grid_id];
                score += confidence_pt_data[grid];
                height += height_pt_data[grid];
            }
            obs->score = score / obs->grids.size();
            obs->height = height / obs->grids.size();
            obs->cloud.reset(new pcl_util::PointCloud);
            obs->class_score.assign(INT_MAX_OBJECT_TYPE, 0.0);
            obs->class_id = 0;
            obs->angle = 0;
        }
    }

    void classify(const float* classify_pt_data,
                const float* confidence_pt_data,
                const float* heading_pt_data,
                const float* height_pt_data, const int num_classes) {
        for (size_t obs_id = 0; obs_id < _obstacles.size(); obs_id++) {
            FastObstacle* obs = &_obstacles[obs_id];
            double score = 0.0;
            double dir_x = 0.0;
            double dir_y = 0.0;
            double height = 0.0;
            obs->class_score.assign(num_classes, 0.0);
            for (size_t grid_id = 0; grid_id < obs->grids.size(); grid_id++) {
                int grid = obs->grids[grid_id];
                score += confidence_pt_data[grid];
                dir_x += heading_pt_data[grid];
                dir_y += heading_pt_data[_mapsize + grid];
                height += height_pt_data[grid];
                for (int k = 0; k < num_classes; k++) {
                    obs->class_score[k] += classify_pt_data[k * _mapsize + grid];
                }
            }
            int class_id = 0;
            obs->class_score[class_id] /= obs->grids.size();
            for (int k = 1; k < num_classes; k++) {
                obs->class_score[k] /= obs->grids.size();
                if (obs->class_score[k] > obs->class_score[class_id]) {
                    class_id = k;
                }
            }
            obs->class_id = class_id;
            obs->angle = atan2(dir_y, dir_x) / 2;
            obs->score = score / obs->grids.size();
            obs->height = height / obs->grids.size();
            obs->cloud.reset(new pcl_util::PointCloud);
        }
    }
    
    void get_objects(const pcl_util::PointCloudPtr &pc_ptr,
                    const float confidence_thresh,
                    const float height_thresh,
                    const std::vector<InternalObjectType> &type_map,
                    std::vector<std::shared_ptr<Object>>* objects) {
        for (const auto point : pc_ptr->points) {
            int pos_x = F2I(point.y, _range, _inv_res_x);
            int pos_y = F2I(point.x, _range, _inv_res_y);
            if (pos_x < _cols && pos_x >= 0 && pos_y < _rows && pos_y >= 0) {
                int obs_id = _id_img[pos_y][pos_x];
                if (obs_id > 0 && _obstacles[obs_id-1].score >= confidence_thresh) {
                    if (height_thresh < 0 || point.z <= _obstacles[obs_id-1].height + height_thresh) {
                        _obstacles[obs_id-1].cloud->push_back(point);
                    }
                }
            }
        }
        //AINFO << "------------------------------------------------------_obstacles.size():" << _obstacles.size();
        for (size_t obs_id = 0; obs_id < _obstacles.size(); obs_id++) {
            FastObstacle* obs = &_obstacles[obs_id];
            if (obs->cloud->size() <= 3) {
                continue;
            }
            //AINFO << "------------------------------------------------------obs->cloud->size():" << obs->cloud->size();
            std::shared_ptr<Object> obj(new apollo::perception::Object);
            #if 0
            obj->cloud = obs->cloud;
            obj->internal_type = type_map[obs->class_id];
            obj->internal_type_probs = obs->class_score;
            obj->direction[0] = cos(obs->angle);
            obj->direction[1] = sin(obs->angle);
            obj->direction[2] = 0;

            //obj->confidence = obs->score;

            if (obj->lidar_supplement == nullptr) {
                obj->lidar_supplement.reset(new LidarSupplement());
            }
            obj->lidar_supplement->raw_probs.push_back(
                    std::vector<float>(MAX_OBJECT_TYPE, 0.f));
            obj->lidar_supplement->raw_classification_methods.push_back("CNNSegmentation");
            obj->lidar_supplement->raw_probs.back()[UNKNOWN] 
                = obs->class_score[0];
            obj->lidar_supplement->raw_probs.back()[PEDESTRIAN] 
                = obs->class_score[4];
            obj->lidar_supplement->raw_probs.back()[BICYCLE] 
                = obs->class_score[3];
            obj->lidar_supplement->raw_probs.back()[VEHICLE] 
                = obs->class_score[1]
                + obs->class_score[2];

            // copy to type
            obj->type_probs.assign(
                    obj->lidar_supplement->raw_probs.back().begin(),
                    obj->lidar_supplement->raw_probs.back().end());
            obj->type = static_cast<ObjectType>(
                    std::distance(obj->type_probs.begin(),
                        std::max_element(obj->type_probs.begin(),
                            obj->type_probs.end())));
        #else
      	obj->cloud = obs->cloud;
      	obj->score = obs->score;
      	obj->score_type = PerceptionObstacle::CONFIDENCE_CNN;
      	obj->type = GetObjectType(obs->meta_type);
      	obj->type_probs = GetObjectTypeProbs(obs->meta_type_probs);
        #endif
        objects->push_back(obj);
        }
    }

  ObjectType GetObjectType(const FastMetaType meta_type_id) {
    switch (meta_type_id) {
      case FastMetaType::META_UNKNOWN:
        return ObjectType::UNKNOWN;
      case FastMetaType::META_SMALLMOT:
        return ObjectType::VEHICLE;
      case FastMetaType::META_BIGMOT:
        return ObjectType::VEHICLE;
      case FastMetaType::META_NONMOT:
        return ObjectType::BICYCLE;
      case FastMetaType::META_PEDESTRIAN:
        return ObjectType::PEDESTRIAN;
      default: {
        AERROR << "Undefined ObjectType output by CNNSeg model.";
        return ObjectType::UNKNOWN;
      }
    }
  }

  std::vector<float> GetObjectTypeProbs(
      const std::vector<float>& meta_type_probs) {
    std::vector<float> object_type_probs(
        static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0.0);
    object_type_probs[static_cast<int>(ObjectType::UNKNOWN)] =
        meta_type_probs[static_cast<int>(FastMetaType::META_UNKNOWN)];
    object_type_probs[static_cast<int>(ObjectType::VEHICLE)] =
        meta_type_probs[static_cast<int>(FastMetaType::META_SMALLMOT)] +
        meta_type_probs[static_cast<int>(FastMetaType::META_BIGMOT)];
    object_type_probs[static_cast<int>(ObjectType::BICYCLE)] =
        meta_type_probs[static_cast<int>(FastMetaType::META_NONMOT)];
    object_type_probs[static_cast<int>(ObjectType::PEDESTRIAN)] =
        meta_type_probs[static_cast<int>(FastMetaType::META_PEDESTRIAN)];
    return object_type_probs;
  }
    
private:
    int _rows = 0;
    int _cols = 0;
    int _mapsize = 0;
    float _range = 0.f;
    float _scale = 0.f;
    float _inv_res_x = 0.f;
    float _inv_res_y = 0.f;
    unsigned short** _id_img = nullptr;
    std::vector<FastObstacle> _obstacles;
    
    struct Node {
        Node(){
            center_node = nullptr;
            parent = nullptr;
            node_rank = 0;
            traversed = 0;
            is_center = false;
            is_object = false;
            obstacle_id = -1;
        }
        Node* center_node;
        Node* parent;
        char node_rank;
        char traversed;
        bool is_center;
        bool is_object;
        int obstacle_id;
    };

    void traverse(Node* x) {
        std::vector<Node*> p;
        p.clear();
        while (x->traversed == 0) {
            p.push_back(x);
            x->traversed = 2;
            x = x->center_node;
        }
        if (x->traversed == 2) {
            for (int i = (int)p.size() - 1; i >= 0 && p[i] != x; i--) {
                p[i]->is_center = true;
            }
            x->is_center = true;
        }
        for (size_t i = 0; i < p.size(); i++){
            Node* y = p[i];
            y->traversed = 1;
            y->parent = x->parent;
        }
    }
};

}  // namespace cnnseg
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_FAST_CLUSTER2D_H_
