#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_cc_2d.h"
#include <cmath>
#include <cstring>
#include <iostream>

namespace apollo {
namespace perception {

int SppCCDetector::detect(const float * const *prob_map,
        const float *offset_map,
        LabelMap* labels) {
    int offset = 0;
    for (int row = 0; row < _rows; row++) {
        for (int col = 0; col < _cols; col++) {
            Node* node = &_nodes[row][col];
            disjoint_set_make_set(node);
            node->is_object = prob_map[row][col] >= 0.5;
            int center_row = static_cast<int>(offset_map[offset] + row + 0.5f);
            int center_col = static_cast<int>(offset_map[offset + 1] + col + 0.5f);
            center_row = std::min(std::max(center_row, 0), _rows - 1);
            center_col = std::min(std::max(center_col, 0), _cols - 1);
            node->center_node = &_nodes[center_row][center_col];
            node->is_center = false;
            node->traversed = 0;
            offset += 2;
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

    for (int row = 0; row < _rows; ++row) {
        for (int col = 0; col < _cols; ++col) {
            Node* node = &_nodes[row][col];
            if (!node->is_center) {
                continue;
            }
            Node* node_neighbor = nullptr;
            // right
            if (col < _cols - 1) {
                node_neighbor = &_nodes[row][col+1];
                if (node_neighbor->is_center) {
                    disjoint_set_union(node, node_neighbor);
                }
            }
            // down
            if (row < _rows - 1) {
                node_neighbor = &_nodes[row+1][col];
                if (node_neighbor->is_center) {
                    disjoint_set_union(node, node_neighbor);
                }
            }
            // right down
            if (row < _rows - 1 && col < _cols - 1) {
                node_neighbor = &_nodes[row+1][col+1];
                if (node_neighbor->is_center) {
                    disjoint_set_union(node, node_neighbor);
                }
            }
            // left down
            if (row < _rows - 1 && col > 0) {
                node_neighbor = &_nodes[row+1][col-1];
                if (node_neighbor->is_center) {
                    disjoint_set_union(node, node_neighbor);
                }
            }
        }
    }

    int id = 0;
    _ptr_map.clear();
    labels->clear_occs();
    for (int row = 0; row < _rows; row++) {
        for (int col = 0; col < _cols; col++) {
            Node* node = &_nodes[row][col];
            if (!node->is_object) {
                (*labels)[row][col] = 0;
                continue;
            }
            Node* root = disjoint_set_find(node);
            auto iter = _ptr_map.find(root);
            if (iter == _ptr_map.end()) {
                iter = _ptr_map.emplace(root, ++id).first;
            }
            (*labels)[row][col] = iter->second;
            labels->push_back(iter->second - 1, col, row);
        }
    }

    return id;
}

void SppCCDetector::traverse(SppCCDetector::Node* x) {
    std::vector<SppCCDetector::Node*> p;
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
    for (unsigned int i = 0; i < p.size(); i++){
        SppCCDetector::Node* y = p[i];
        y->traversed = 1;
        y->parent = x->parent;
    }
}

}  // namespace perception
}  // namespace apollo
