// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: Dongming Chen (chendongming@baidu.com)
// @author: Yan He (yanhe@baidu.com)
// @file: segment_graph.cpp
// @brief:

#include "modules/perception/obstacle/lidar/segmentation/spp_common/segment_graph.h"
#include<float.h>
#include <iostream>

namespace apollo {
namespace perception {

bool operator<(const Edge &a, const Edge &b) {
    return a.w < b.w;
}

Universe *segment_graph(int num_vertices, int num_edges, Edge *edges, float c) { 
    // sort edges by weight
    std::sort(edges, edges + num_edges);

    // make a disjoint-set forest
    Universe* universe = new Universe(num_vertices);

    // init thresholds
    float *threshold = new float[num_vertices];
    for (int i = 0; i < num_vertices; ++i) {
        threshold[i] = THRESHOLD(1, c);
    }
    // for each edge, in non-decreasing weight order...
    for (int i = 0; i < num_edges; ++i) {
        Edge *pedge = &edges[i];

    // components conected by this edge
        int a = universe->find(pedge->a);
        int b = universe->find(pedge->b);
        if (a != b) {
            if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) {
                universe->join(a, b);
                a = universe->find(a);
                threshold[a] = pedge->w + THRESHOLD(universe->size(a), c);
            }
        }
    }

    // free up
    delete[] threshold;
    return universe;
}

Universe *segment_graph_with_sorted_edges(int num_vertices, int num_edges, 
        const Edge *edges, float c) { 

    // make a disjoint-set forest
    Universe* universe = new Universe(num_vertices);

    // init thresholds
    float *threshold = new float[num_vertices];
    for (int i = 0; i < num_vertices; ++i) {
        threshold[i] = THRESHOLD(1, c);
    }
    // for each edge, in non-decreasing weight order...
    for (int i = 0; i < num_edges; ++i) {
        const Edge *pedge = &edges[i];

    // components conected by this edge
        int a = universe->find(pedge->a);
        int b = universe->find(pedge->b);
        if (a != b) {
            if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) {
                universe->join(a, b);
                a = universe->find(a);
                threshold[a] = pedge->w + THRESHOLD(universe->size(a), c);
            }
        }
    }

    // free up
    delete[] threshold;
    return universe;
}

GraphSegmentor::GraphSegmentor() : _c(0.0) {
    _threshold.reserve(10000);
}

void GraphSegmentor::init(float c) {
    _threshold_table.resize(50000, FLT_MAX);
    for (std::size_t i = 1; i < _threshold_table.size(); ++i) {
        _threshold_table[i] = THRESHOLD(i, c);
    }
    _c = c;
}

void GraphSegmentor::segment_graph(int num_vertices, 
        int num_edges, Edge *edges, bool need_sort) {
    // sort edges by weight
    if (need_sort) {
        std::sort(edges, edges + num_edges);
    }

    // make a disjoint-set forest
    _universe.reset(num_vertices);

    // init thresholds
    float thred = THRESHOLD(1, _c);
    _threshold.assign(num_vertices, thred);
    int size = static_cast<int>(_threshold_table.size());
    // for each edge, in non-decreasing weight order...
    for (int i = 0; i < num_edges; ++i) {
        const Edge *pedge = &edges[i];

        // components conected by this edge
        int a = _universe.find(pedge->a);
        int b = _universe.find(pedge->b);
        if (a != b) {
            if ((pedge->w <= _threshold[a]) && (pedge->w <= _threshold[b])) {
                _universe.join(a, b);
                a = _universe.find(a);
                _threshold[a] = pedge->w + (_universe.size(a) >= size ? 
                        THRESHOLD(_universe.size(a), _c) : _threshold_table[_universe.size(a)]);
            }
        }
    }
}

void GraphSegmentor::segment_graph_max_dist(int num_vertices, 
        int num_edges, Edge *edges,
        std::unordered_map<int, std::vector<float>> id_map,
        float max_dist_threshold,
        bool need_sort) {
    // sort edges by weight
    if (need_sort) {
        std::sort(edges, edges + num_edges);
    }

    // make a disjoint-set forest
    _universe.reset(num_vertices);

    // init thresholds
    float thred = THRESHOLD(1, _c);
    _threshold.assign(num_vertices, thred);
    int size = static_cast<int>(_threshold_table.size());
    // for each edge, in non-decreasing weight order...
    for (int i = 0; i < num_edges; ++i) {
        const Edge *pedge = &edges[i];

        // components conected by this edge
        int a = _universe.find(pedge->a);
        int b = _universe.find(pedge->b);
        if (a != b) {
            if (id_map.find(a) != id_map.end() &&
                id_map.find(b) != id_map.end() &&
                id_map[a][0] < FLT_MAX &&
                id_map[b][0] < FLT_MAX) {
                if (fabs(id_map[a][0] - id_map[b][1]) > max_dist_threshold ||
                    fabs(id_map[a][1] - id_map[b][0]) > max_dist_threshold ||
                    fabs(id_map[a][2] - id_map[b][3]) > max_dist_threshold ||
                    fabs(id_map[a][3] - id_map[b][2]) > max_dist_threshold) {
                    continue;
                }
            }

            if ((pedge->w <= _threshold[a]) && (pedge->w <= _threshold[b])) {
                _universe.join(a, b);
                a = _universe.find(a);
                _threshold[a] = pedge->w + (_universe.size(a) >= size ? 
                        THRESHOLD(_universe.size(a), _c) : _threshold_table[_universe.size(a)]);
               id_map[a] = {std::min(id_map[a][0], id_map[b][0]),
                               std::max(id_map[a][1], id_map[b][1]),
                               std::min(id_map[a][2], id_map[b][2]),
                               std::max(id_map[a][3], id_map[b][3])};
            }
        }
    }
}

}  // namespace perception
}  // namespace apollo
