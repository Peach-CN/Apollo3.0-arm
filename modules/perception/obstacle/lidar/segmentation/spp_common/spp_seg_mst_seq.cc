#include <functional>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_mst_seq.h"

namespace apollo {
namespace perception {

void SppSegMstSeq::init(unsigned int width, unsigned int height, float resolution, float c) {
    _dynamic_map.init(height, width, resolution);
    _graph_segmentor.init(c);
}

void SppSegMstSeq::init(unsigned int width, unsigned int height, float resolution,
        float c, float max_dist_threshold) {
    _max_dist_threshold = max_dist_threshold;
    _dynamic_map.init(height, width, resolution);
    _graph_segmentor.init(c);
}

void SppSegMstSeq::add_point_cloud(pcl_util::PointCloudConstPtr world_cloud,
        const SppCloudMask& mask,
        const Eigen::Matrix4d& pose_v2w) {
    Eigen::Vector2f center;
    center(0) = static_cast<float>(pose_v2w(0, 3));
    center(1) = static_cast<float>(pose_v2w(1, 3));
    _dynamic_map.update_map(center, true);
    _dynamic_map.add_point_cloud(world_cloud, mask, &_indices);
} 

unsigned int SppSegMstSeq::segmentation(
        pcl_util::PointCloudConstPtr point_cloud,
        const SppCloudMask& mask,
        SppClusterList& clusters,
        int radius) {

    int width = _dynamic_map.cols();
    int height = _dynamic_map.rows();

    // build graph on dynamic map
    Edge edge_t;
    _mst_edges.clear();
    std::function<float(const SppSegMstSeq&, 
            const SppMapNode&, const int&, const int&,
            const SppMapNode&, const int&, const int&)> dist_func
        = _dynamic_map.is_first_update() ?
        &SppSegMstSeq::distance : &SppSegMstSeq::distance_seq;

    //// if max_dist
    _id_map.clear();
    // a) add short-term edges
    for (int y = radius; y < height - radius; ++y) {
        for (int x = radius; x < width - radius; ++x) {
            if (_dynamic_map[y][x].count == 0) {
                continue;
            }
            edge_t.a = _dynamic_map[y][x].id;
            //// if max_dist
            if (_id_map.find(_dynamic_map[y][x].id) == _id_map.end()) {
                _id_map[_dynamic_map[y][x].id] = {
                        _dynamic_map[y][x].min_x,
                        _dynamic_map[y][x].max_x,
                        _dynamic_map[y][x].min_y,
                        _dynamic_map[y][x].max_y};
            } else {
                _id_map[_dynamic_map[y][x].id][0] = std::min(
                            _id_map[_dynamic_map[y][x].id][0],
                            _dynamic_map[y][x].min_x);
                _id_map[_dynamic_map[y][x].id][1] = std::max(
                            _id_map[_dynamic_map[y][x].id][1],
                            _dynamic_map[y][x].max_x);
                _id_map[_dynamic_map[y][x].id][2] = std::min(
                            _id_map[_dynamic_map[y][x].id][2],
                            _dynamic_map[y][x].min_y);
                _id_map[_dynamic_map[y][x].id][3] = std::max(
                            _id_map[_dynamic_map[y][x].id][3],
                            _dynamic_map[y][x].max_y);
            }

            // right
            for (int nx = x + 1; nx <= x + radius; ++nx) {
                if (_dynamic_map[y][nx].count > 0) {
                    edge_t.b = _dynamic_map[y][nx].id;
                    //// if max_dist
                    if (_id_map.find(_dynamic_map[y][nx].id) == _id_map.end()) {
                        _id_map[_dynamic_map[y][nx].id] = {
                                _dynamic_map[y][nx].min_x,
                                _dynamic_map[y][nx].max_x,
                                _dynamic_map[y][nx].min_y,
                                _dynamic_map[y][nx].max_y};
                    } else {
                        _id_map[_dynamic_map[y][nx].id][0] = std::min(
                                    _id_map[_dynamic_map[y][nx].id][0],
                                    _dynamic_map[y][nx].min_x);
                        _id_map[_dynamic_map[y][nx].id][1] = std::max(
                                    _id_map[_dynamic_map[y][nx].id][1],
                                    _dynamic_map[y][nx].max_x);
                        _id_map[_dynamic_map[y][nx].id][2] = std::min(
                                    _id_map[_dynamic_map[y][nx].id][2],
                                    _dynamic_map[y][nx].min_y);
                        _id_map[_dynamic_map[y][nx].id][3] = std::max(
                                    _id_map[_dynamic_map[y][nx].id][3],
                                    _dynamic_map[y][nx].max_y);
                    }

                    edge_t.w = dist_func(*this, _dynamic_map[y][x], x, y,
                            _dynamic_map[y][nx], nx, y);
                    _mst_edges.push_back(edge_t);
                }
            }
            // down
            for (int ny = y + 1; ny <= y + radius; ++ny) {
                for (int nx = x - radius; nx <= x + radius; ++nx) {
                    if (_dynamic_map[ny][nx].count > 0) {
                        edge_t.b = _dynamic_map[ny][nx].id;
                        //// if max_dist
                        if (_id_map.find(_dynamic_map[ny][nx].id) == _id_map.end()) {
                            _id_map[_dynamic_map[ny][nx].id] = {
                                    _dynamic_map[ny][nx].min_x,
                                    _dynamic_map[ny][nx].max_x,
                                    _dynamic_map[ny][nx].min_y,
                                    _dynamic_map[ny][nx].max_y};
                        } else {
                            _id_map[_dynamic_map[ny][nx].id][0] = std::min(
                                        _id_map[_dynamic_map[ny][nx].id][0],
                                        _dynamic_map[ny][nx].min_x);
                            _id_map[_dynamic_map[ny][nx].id][1] = std::max(
                                        _id_map[_dynamic_map[ny][nx].id][1],
                                        _dynamic_map[ny][nx].max_x);
                            _id_map[_dynamic_map[ny][nx].id][2] = std::min(
                                        _id_map[_dynamic_map[ny][nx].id][2],
                                        _dynamic_map[ny][nx].min_y);
                            _id_map[_dynamic_map[ny][nx].id][3] = std::max(
                                        _id_map[_dynamic_map[ny][nx].id][3],
                                        _dynamic_map[ny][nx].max_y);
                        }

                        edge_t.w = dist_func(*this, _dynamic_map[y][x], x, y,
                                _dynamic_map[ny][nx], nx, ny);
                        _mst_edges.push_back(edge_t);
                    }
                }
            }
        }
    }

    // b) add sequential long-term edges
    const int radius_long = radius * 16;
    const int skip = 3;
    const float long_term_weight = _dynamic_map.resolution() * 1.5f;
    if (!_dynamic_map.is_first_update()) {
        for (int y = radius_long; y < height - radius_long; y += skip) {
            for (int x = radius_long; x < width - radius_long; x += skip) {
                if (_dynamic_map[y][x].count == 0 || _dynamic_map[y][x].label == 0) {
                    continue;
                }
                edge_t.a = _dynamic_map[y][x].id;
                //// if max_dist
                if (_id_map.find(_dynamic_map[y][x].id) == _id_map.end()) {
                    _id_map[_dynamic_map[y][x].id] = {
                            _dynamic_map[y][x].min_x,
                            _dynamic_map[y][x].max_x,
                            _dynamic_map[y][x].min_y,
                            _dynamic_map[y][x].max_y};
                } else {
                    _id_map[_dynamic_map[y][x].id][0] = std::min(
                                _id_map[_dynamic_map[y][x].id][0],
                                _dynamic_map[y][x].min_x);
                    _id_map[_dynamic_map[y][x].id][1] = std::max(
                                _id_map[_dynamic_map[y][x].id][1],
                                _dynamic_map[y][x].max_x);
                    _id_map[_dynamic_map[y][x].id][2] = std::min(
                                _id_map[_dynamic_map[y][x].id][2],
                                _dynamic_map[y][x].min_y);
                    _id_map[_dynamic_map[y][x].id][3] = std::max(
                                _id_map[_dynamic_map[y][x].id][3],
                                _dynamic_map[y][x].max_y);
                }

                // right
                for (int nx = x + radius + 1; nx <= x + radius_long; nx += skip) {
                    if (_dynamic_map[y][nx].count > 0 && 
                            _dynamic_map[y][x].label == _dynamic_map[y][nx].label) {
                        edge_t.b = _dynamic_map[y][nx].id;
                        //// if max_dist
                        if (_id_map.find(_dynamic_map[y][nx].id) == _id_map.end()) {
                            _id_map[_dynamic_map[y][nx].id] = {
                                    _dynamic_map[y][nx].min_x,
                                    _dynamic_map[y][nx].max_x,
                                    _dynamic_map[y][nx].min_y,
                                    _dynamic_map[y][nx].max_y};
                        } else {
                            _id_map[_dynamic_map[y][nx].id][0] = std::min(
                                        _id_map[_dynamic_map[y][nx].id][0],
                                        _dynamic_map[y][nx].min_x);
                            _id_map[_dynamic_map[y][nx].id][1] = std::max(
                                        _id_map[_dynamic_map[y][nx].id][1],
                                        _dynamic_map[y][nx].max_x);
                            _id_map[_dynamic_map[y][nx].id][2] = std::min(
                                        _id_map[_dynamic_map[y][nx].id][2],
                                        _dynamic_map[y][nx].min_y);
                            _id_map[_dynamic_map[y][nx].id][3] = std::max(
                                        _id_map[_dynamic_map[y][nx].id][3],
                                        _dynamic_map[y][nx].max_y);
                        }

                        edge_t.w = long_term_weight;
                        _mst_edges.push_back(edge_t);
                    }
                }
                // down
                for (int ny = y + radius + 1; ny <= y + radius_long; ny += skip) {
                    for (int nx = x - radius_long; nx <= x + radius_long; nx += skip) {
                        if (_dynamic_map[ny][nx].count > 0 &&
                                abs(nx - x) > radius &&
                                _dynamic_map[ny][nx].label == _dynamic_map[y][x].label) {
                            edge_t.b = _dynamic_map[ny][nx].id;
                            //// if max_dist
                            if (_id_map.find(_dynamic_map[ny][nx].id) == _id_map.end()) {
                                _id_map[_dynamic_map[ny][nx].id] = {
                                        _dynamic_map[ny][nx].min_x,
                                        _dynamic_map[ny][nx].max_x,
                                        _dynamic_map[ny][nx].min_y,
                                        _dynamic_map[ny][nx].max_y};
                            } else {
                                _id_map[_dynamic_map[ny][nx].id][0] = std::min(
                                            _id_map[_dynamic_map[ny][nx].id][0],
                                            _dynamic_map[ny][nx].min_x);
                                _id_map[_dynamic_map[ny][nx].id][1] = std::max(
                                            _id_map[_dynamic_map[ny][nx].id][1],
                                            _dynamic_map[ny][nx].max_x);
                                _id_map[_dynamic_map[ny][nx].id][2] = std::min(
                                            _id_map[_dynamic_map[ny][nx].id][2],
                                            _dynamic_map[ny][nx].min_y);
                                _id_map[_dynamic_map[ny][nx].id][3] = std::max(
                                            _id_map[_dynamic_map[ny][nx].id][3],
                                            _dynamic_map[ny][nx].max_y);
                            }

                            edge_t.w = long_term_weight;
                            _mst_edges.push_back(edge_t);
                        }
                    }
                }
            }
        }
    }

    // segmentation and smooth label
    
    //Universe* universe = segment_graph(_dynamic_map.id_count(),
    //        _mst_edges.size(), _mst_edges.data(), c);

    if (_max_dist_threshold < 0.01) {
        _graph_segmentor.segment_graph(_dynamic_map.id_count(),
                _mst_edges.size(), _mst_edges.data());
    } else {
        //// max_dist
        _graph_segmentor.segment_graph_max_dist(_dynamic_map.id_count(),
                _mst_edges.size(), _mst_edges.data(), _id_map, _max_dist_threshold);

    }

    Universe* universe = _graph_segmentor.get_universe();

    std::vector<unsigned int> mapping(_dynamic_map.id_count(), 0);
    std::map<unsigned int, unsigned int> unique;
    std::map<unsigned int, unsigned int>::iterator iter;
    unsigned int label = 0;
    unsigned int size = clusters.size();
    for (int i = 0; i < static_cast<int>(_dynamic_map.id_count()); ++i) {
        mapping[i] = universe->find(i);
        iter = unique.find(mapping[i]);
        if (iter == unique.end()) {
            unique.insert(std::make_pair(mapping[i], label));
            mapping[i] = label++;
        }
        else {
            mapping[i] = iter->second;
        }

        mapping[i] += size;
    } 
    //delete universe;

    // assgin smoothed label to dynamic map
    for (unsigned int i = 0; i < _dynamic_map.size(); ++i) {
        if (_dynamic_map[0][i].count > 0) {
            _dynamic_map[0][i].label = mapping[_dynamic_map[0][i].id];
        }
    }

    merge_overlapping_segments(label);

    // dump to clusters
    unsigned int size_plus = size + unique.size();
    clusters.resize(size_plus); 

    if (_indices.size() > 0) {
        for (unsigned int i = 0, j = 0; i < point_cloud->points.size(); ++i) {
            if (mask.size() && mask[i] == 0) {
                continue;
            }
            if (_indices[j] < 0) {
                ++j;
                continue;
            }
            clusters[mapping[_dynamic_map[0][_indices[j]].id]].add_sample(
                    point_cloud->points[i], i);
            ++j;
        }
    }

    return size_plus - size;
}

float SppSegMstSeq::distance(const SppMapNode& lhs, const int& xl, const int& yl,
        const SppMapNode& rhs, const int& xr, const int& yr) const {
    float x_diff = static_cast<float>(xl - xr) * _dynamic_map.resolution();
    float y_diff = static_cast<float>(yl - yr) * _dynamic_map.resolution();
    float a_diff = lhs.altitude_mean - rhs.altitude_mean
        + (lhs.altitude_std - rhs.altitude_std);
    return sqrt(x_diff * x_diff + y_diff * y_diff + a_diff * a_diff);
}

float SppSegMstSeq::distance_seq(const SppMapNode& lhs, const int& xl, const int& yl,
        const SppMapNode& rhs, const int& xr, const int& yr) const {
    if (lhs.label > 0 && lhs.label == rhs.label) {
        return _dynamic_map.resolution();
    } else {
        float x_diff = static_cast<float>(xl - xr) * _dynamic_map.resolution();
        float y_diff = static_cast<float>(yl - yr) * _dynamic_map.resolution();
        float a_diff = lhs.altitude_mean - rhs.altitude_mean
            + (lhs.altitude_std - rhs.altitude_std);
        return sqrt(x_diff * x_diff + y_diff * y_diff + a_diff * a_diff);
    }
}

void SppSegMstSeq::merge_overlapping_segments(unsigned int num_labels) {
    std::vector<std::map<unsigned int, unsigned int> > neighbor_counting_table(num_labels);
    std::vector<unsigned int> counting_table(num_labels, 0);
    std::vector<unsigned int> merged_label(num_labels, 0);
    std::iota(merged_label.begin(), merged_label.end(), 0);
    for (unsigned int y = 1; y < _dynamic_map.rows() - 1; ++y) {
        for (unsigned int x = 1; x < _dynamic_map.cols() - 1; ++x) {
            if (_dynamic_map[y][x].count == 0) {
                continue;
            }
            unsigned int& label = _dynamic_map[y][x].label;
            ++counting_table[label];
            std::set<unsigned int> nlabels;
            for (unsigned int ny = y - 1; ny <= y + 1; ++ny) {
                for (unsigned int nx = x - 1; nx <= x + 1; ++nx) {
                    if (_dynamic_map[ny][nx].count == 0) {
                        continue;
                    }
                    if (label != _dynamic_map[ny][nx].label) {
                        nlabels.insert(_dynamic_map[ny][nx].label);
                    }
                }
            }
            for (auto iter = nlabels.begin(); iter != nlabels.end(); ++iter) {
                ++neighbor_counting_table[label][*iter];
            }
        }
    }

    float ratio = 0.f;
    while (true) {
        bool need_iter = false;
        for (unsigned int i = 0; i < num_labels; ++i) {
            bool merge = false;
            auto iter = neighbor_counting_table[i].begin();
            for (; iter != neighbor_counting_table[i].end(); ++iter) {
                ratio = static_cast<float>(iter->second) /  counting_table[i];
                if (ratio >= 0.5f) {
                    merge = true;
                    break;
                }
            }
            if (merge) {
                const unsigned int label_src = i;
                const unsigned int label_dst = iter->first; 
                // merge src to dst
                for (iter = neighbor_counting_table[i].begin();
                        iter != neighbor_counting_table[i].end();
                        ++iter) {
                    if (iter->first != label_dst) {
                        neighbor_counting_table[label_dst][iter->first]
                            += iter->second;
                    }
                }
                iter = neighbor_counting_table[label_dst].find(label_src);
                if (iter != neighbor_counting_table[label_dst].end()) {
                    neighbor_counting_table[label_dst].erase(iter);
                }
                // assign label
                merged_label[label_src] = label_dst;
                // clear and break
                neighbor_counting_table[i].clear();
                need_iter = true;
            }
        }
        if (!need_iter) {
            break;
        }
    }

    for (unsigned int i = 0; i < _dynamic_map.size(); ++i) {
        if (_dynamic_map[0][i].count > 0) {
            _dynamic_map[0][i].label = merged_label[_dynamic_map[0][i].label];
        }
    }
}

}  // namespace perception
}  // namespace apollo

