#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_CC_2D_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_CC_2D_H

#include <map>
#include <vector>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/i_alloc.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_label_map.h"

namespace apollo {
namespace perception {

class SppCCDetector {
public:
    SppCCDetector() : _nodes(nullptr) {
    }
    ~SppCCDetector() {
        if (_nodes != nullptr) {
            idl::i_free2(_nodes);
        }
    }
    bool init(int width, int height) {
        _rows = height;
        _cols = width;
        if (_nodes == nullptr) {
            _nodes = idl::i_alloc2<Node>(height, width);
        }
        return true;
    }

    int detect(const float * const *prob_map,
            const float *offset_map,
            LabelMap *labels);
private:
    struct Node {
        Node(){
            center_node = nullptr;
            parent = nullptr;
            node_rank = 0;
            traversed = 0;
            is_center = false;
            is_object = false;
        }
        Node* center_node;
        Node* parent;
        char node_rank;
        char traversed;
        bool is_center;
        bool is_object;
    };

    void traverse(Node* x);

    int _rows = 0;
    int _cols = 0;
    Node** _nodes = nullptr;

    std::map<Node*, int> _ptr_map;
};

template<class T>
void disjoint_set_make_set(T* x) {
    x->parent = x;
    x->node_rank = 0;
}

template<class T>
T* disjoint_set_find_loop(T* x) {
    T* node = x;
    while (node->parent != node) {
        node = node->parent;
    }
    return node;
}

template<class T>
T* disjoint_set_find(T* x) {
    T* y = x->parent;
    if (y == x || y->parent == y) {
        return y;
    }
    T* root = disjoint_set_find_loop(y->parent);
    x->parent = root;
    y->parent = root;
    return root;
}

template<class T>
void disjoint_set_union(T* x, T* y) {
    x = disjoint_set_find(x);
    y = disjoint_set_find(y);
    if (x == y) {
        return;
    }
    if (x->node_rank < y->node_rank) {
        x->parent = y;
    } else if (y->node_rank < x->node_rank) {
        y->parent = x;
    } else {
        y->parent = x;
        x->node_rank++;
    }
}

}  // namespace perception
}  // namespace apollo
#endif

