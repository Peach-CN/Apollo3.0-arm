// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: Dongming Chen (chendongming@baidu.com)
// @author: Yan He (yanhe@baidu.com)
// @file: segment_graph.h
// @brief:

#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_MST_SEGMENT_GRAPH_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_MST_SEGMENT_GRAPH_H

#include <algorithm>
#include <cmath>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <math.h>

#include "modules/common/util/disjoint_set.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/disjoint.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_dynamic_map.h"

namespace apollo {
namespace perception {
// threshold function
#define THRESHOLD(size, c) (c/size)

typedef struct {
    float w;
    int a;
    int b;
} Edge;

bool operator<(const Edge &a, const Edge &b);

/*
 * Segment a graph
 *
 * Returns a disjoint-set forest representing the segmentation.
 *
 * num_vertices: number of vertices in graph.
 * num_edges: number of edges in graph
 * Edges: array of Edges.
 * c: constant for treshold function.
 */
Universe *segment_graph(int num_vertices, int num_edges, Edge *edges, float c);

/*
 * Segment a graph with sorted edge
 *
 * Returns a disjoint-set forest representing the segmentation.
 *
 * num_vertices: number of vertices in graph.
 * num_edges: number of edges in graph
 * Edges: sorted array of Edges.
 * c: constant for treshold function.
 */
Universe *segment_graph_with_sorted_edges(int num_vertices, 
        int num_edges, const Edge *edges, float c);

/*
 * Wrapper of the above functions to manage memory
 */

class GraphSegmentor {
public:
    GraphSegmentor();

    void init(float c);

    void segment_graph(int num_vertices, int num_edges, Edge *edges, bool need_sort = true);

    void segment_graph_max_dist(int num_vertices, int num_edges, Edge *edges,
        std::unordered_map<int, std::vector<float>> id_map, float max_dist_threshold,
        bool need_sort = true);

    Universe* get_universe() {
        return &_universe;
    }

private:
    Universe _universe;
    std::vector<float> _threshold;
    std::vector<float> _threshold_table;
    float _c;
};

}  // namespace perception
}  // namespace apollo

#endif // ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_MST_SEGMENT_GRAPH_H
