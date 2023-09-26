#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_dynamic_map.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/timer.h"

namespace apollo {
namespace perception {

SppDynamicMap::~SppDynamicMap() {
    release();
}

void SppDynamicMap::init(unsigned int rows, unsigned int cols, float resolution) {
    _rows = rows;
    _cols = cols;
    _half_rows = static_cast<int>(_rows >> 1);
    _half_cols = static_cast<int>(_cols >> 1);
    _resolution = resolution;
    release();
    _buffer[0] = idl::i_alloc2<SppMapNode>(rows, cols);
    _buffer[1] = idl::i_alloc2<SppMapNode>(rows, cols);
    _range_mask = idl::i_alloc2<unsigned char>(rows, cols);
    memset(_buffer[0][0], 0, sizeof(SppMapNode) * _rows * _cols);
    memset(_buffer[1][0], 0, sizeof(SppMapNode) * _rows * _cols);
    memset(_range_mask[0], 0, sizeof(char) * _rows * _cols);
    _data = _buffer[0];
    _origin_set_flag = false;

    // generate mask according to the cumulated range parameter
    float distance_r_sqr_ul = _half_rows * _half_rows;
    float distance_c_sqr_ul = _half_cols * _half_cols;
    float distance_r_sqr = distance_r_sqr_ul;
    float distance_c_sqr = distance_c_sqr_ul;
    for (int r = 0; r < static_cast<int>(_rows); ++r) {
        distance_c_sqr = distance_c_sqr_ul;
        for (int c = 0; c < static_cast<int>(_cols); ++c) {
            float distance = sqrt(distance_r_sqr + distance_c_sqr) * _resolution;
            if (distance < _s_cumulated_range) {
                _range_mask[r][c] = 1;
            }
            if (distance < _s_ignore_range) {
                _range_mask[r][c] = 2;
            }
            distance_c_sqr += (c - _half_cols) * 2 + 1;
            _data[r][c].min_x = FLT_MAX;
            _data[r][c].max_x = -FLT_MAX;
            _data[r][c].min_y = FLT_MAX;
            _data[r][c].max_y = -FLT_MAX;
        }
        distance_r_sqr += (r - _half_rows) * 2 + 1;
    }
}

void SppDynamicMap::release() {
    if (_buffer[0]) {
        idl::i_free2(_buffer[0]);
        _buffer[0] = nullptr;
    }
    if (_buffer[1]) {
        idl::i_free2(_buffer[1]);
        _buffer[1] = nullptr;
    }
    _data = nullptr;
}

void SppDynamicMap::reset() {
    if (_data) {
        memset(_data[0], 0, sizeof(SppMapNode) * _rows * _cols);
    }
    _id_count = 0;
    _global_life_time = 0;

    for (int r = 0; r < static_cast<int>(_rows); ++r) {
        for (int c = 0; c < static_cast<int>(_cols); ++c) {
            if (!_data) {
                break;
            }
            _data[r][c].min_x = FLT_MAX;
            _data[r][c].max_x = -FLT_MAX;
            _data[r][c].min_y = FLT_MAX;
            _data[r][c].max_y = -FLT_MAX;
        }
    }
}

bool SppDynamicMap::get_coordinate(const Eigen::Vector2f& point,
        Eigen::Vector2i* coordinate) const {
    if (!_origin_set_flag || coordinate == nullptr) {
        return false;
    }
    (*coordinate)(0) = static_cast<int>((point(0) - _origin(0)) / _resolution)
        + _half_cols;
    (*coordinate)(1) = static_cast<int>((point(1) - _origin(1)) / _resolution)
        + _half_rows;
    *coordinate += _offset;
    return true;
}

void SppDynamicMap::update_map(const Eigen::Vector2f& center, bool shift) {
    if (!_origin_set_flag) {
        _origin = center;
        _offset << 0, 0;
        _origin_set_flag = true;
    } else {
        Eigen::Vector2i center_coordinate;
        center_coordinate(0) = static_cast<int>((center(0) - _origin(0)) / _resolution);
        center_coordinate(1) = static_cast<int>((center(1) - _origin(1)) / _resolution);
        Eigen::Vector2i offset_inc = center_coordinate + _offset;
        _offset = -center_coordinate;
        if (abs(offset_inc(0)) >= _half_cols || abs(offset_inc(1)) >= _half_rows) {
            reset();
            return;
        }
        if (shift) {
            shift_map(offset_inc);
            shift_indices();
            // vehicle is moving
            // if (abs(offset_inc(0)) + abs(offset_inc(1)) > 1) {
            clean_map();
            // } else {
            //     maintain_map();
            // }
        }
    }
}

void SppDynamicMap::shift_map(const Eigen::Vector2i& shift_dst_to_src) {
    SppMapNode** src = nullptr;
    SppMapNode** dst = nullptr;
    if (_buffer[0] == _data) {
        src = _buffer[0];
        dst = _buffer[1];
    } else {
        src = _buffer[1];
        dst = _buffer[0];
    }
    // switch data
    _data = dst;
    // copy
    memset(dst[0], 0, sizeof(SppMapNode) * _rows * _cols);
    Eigen::Vector2i start_src;
    Eigen::Vector2i end_src;
    start_src(0) = std::max(0, shift_dst_to_src(0));
    start_src(1) = std::max(0, shift_dst_to_src(1));
    end_src(0) = std::min(_cols - 1, _cols - 1 + shift_dst_to_src(0));
    end_src(1) = std::min(_rows - 1, _rows - 1 + shift_dst_to_src(1));
    if (start_src(0) > end_src(0) || start_src(1) > end_src(1)) {
        return;
    }
    Eigen::Vector2i start_dst = start_src - shift_dst_to_src;
    Eigen::Vector2i end_dst = end_src - shift_dst_to_src;

    int col_length = end_src(0) - start_src(0) + 1;

    for (int row = start_dst(1); row <= end_dst(1); ++row) {
        memcpy(dst[row] + start_dst(0),
                src[row + shift_dst_to_src(1)] + start_src(0),
                sizeof(SppMapNode) * col_length);
    }
}

void SppDynamicMap::shift_indices() {
    _id_count = 0;
    for (unsigned int i = 0; i < _rows * _cols; ++i) {
        SppMapNode& node = _data[0][i];
        if (node.count > 0) {
            node.id = _id_count++;
            // Note old label will be increased by one,
            // to distinguish with new added sample(label=0)
            ++node.label;
        }
    }
}

void SppDynamicMap::clean_map() {
    std::vector<int> indices;
    for (unsigned int i = 0; i < _rows * _cols; ++i) {
        SppMapNode& node = _data[0][i];
        node.min_x = FLT_MAX;
        node.max_x = -FLT_MAX;
        node.min_y = FLT_MAX;
        node.max_y = -FLT_MAX;
        if (node.count > 0 &&
                (_global_life_time - node.lift_time > _s_clean_disappear_time
                 || !_range_mask[0][i])) {
            memset(&node, 0, sizeof(SppMapNode));
            node.min_x = FLT_MAX;
            node.max_x = -FLT_MAX;
            node.min_y = FLT_MAX;
            node.max_y = -FLT_MAX;
        }
    }
    std::vector<unsigned int> xs;
    std::vector<unsigned int> ys;
    for (unsigned int y = 1; y < _rows - 1; ++y) {
        for (unsigned int x = 1; x < _cols - 1; ++x) {
            if (_data[y][x].count > 0 &&
                    _data[y-1][x-1].count == 0 &&
                    _data[y-1][x].count == 0 &&
                    _data[y-1][x+1].count == 0 &&
                    _data[y][x-1].count == 0 &&
                    _data[y][x+1].count == 0 &&
                    _data[y+1][x-1].count == 0 &&
                    _data[y+1][x].count == 0 &&
                    _data[y+1][x+1].count == 0) {
                xs.push_back(x);
                ys.push_back(y);
            }
        }
    }
    for (unsigned int i = 0; i < xs.size(); ++i) {
        memset(&_data[ys[i]][xs[i]], 0, sizeof(SppMapNode));
        _data[ys[i]][xs[i]].min_x = FLT_MAX;
        _data[ys[i]][xs[i]].max_x = -FLT_MAX;
        _data[ys[i]][xs[i]].min_y = FLT_MAX;
        _data[ys[i]][xs[i]].max_y = -FLT_MAX;
    }
}

void SppDynamicMap::maintain_map() {
    for (unsigned int i = 0; i < _rows * _cols; ++i) {
        SppMapNode& node = _data[0][i];
        if (node.count > 0) {
            if (!_range_mask[0][i]) {
                memset(&node, 0, sizeof(SppMapNode));
                node.min_x = FLT_MAX;
                node.max_x = -FLT_MAX;
                node.min_y = FLT_MAX;
                node.max_y = -FLT_MAX;
            } else {
                node.lift_time = _global_life_time;
            }
        }
    }
}

bool SppDynamicMap::add_point_cloud(
        const pcl_util::PointCloudConstPtr point_cloud,
        const SppCloudMask& mask,
        std::vector<int>* map_indices) {
    if (!_origin_set_flag) {
        return false;
    }
    if (map_indices != nullptr) {
        map_indices->assign(point_cloud->points.size(), -1);
    }
    int cols = static_cast<int>(_cols);
    int rows = static_cast<int>(_rows);
    Eigen::Vector2f center;
    center[0] = _origin[0] - _offset[0] * _resolution;
    center[1] = _origin[1] - _offset[1] * _resolution;
    for (unsigned int i = 0; i < point_cloud->points.size(); ++i) {
        const pcl_util::Point& point = point_cloud->points[i];
        Eigen::Vector2f p(point.x, point.y);
        //if (point.h < 10.f && (point.h < _s_min_height || point.h > _s_max_height)) {
        //    continue;
        //}
        Eigen::Vector2i coordinate;
        if (!get_coordinate(p, &coordinate)) {
            continue;
        }
        const int& x = coordinate(0);
        const int& y = coordinate(1);
        if (x < 0 || x >= cols || y < 0 || y >= rows || _range_mask[y][x] == 2) {
            continue;
        }
        
        if (mask.size() == 0 || mask[i] > 0) {
            // online update node
            if (_data[y][x].count == 0) {
                _data[y][x].id = _id_count++;
            }
            unsigned int countp1 = 1 + _data[y][x].count;
            float last_mean = _data[y][x].altitude_mean;
            float last_std = _data[y][x].altitude_std;
            _data[y][x].altitude_mean = (_data[y][x].altitude_mean * _data[y][x].count
                    + point.z) / countp1;
            if (countp1 == 1) {
                _data[y][x].altitude_std = 0.f;
            } else {
                _data[y][x].altitude_std =
                    (_data[y][x].count * last_std * last_std
                     + (point.z - last_mean)
                     * (point.z - _data[y][x].altitude_mean)) / countp1;
                _data[y][x].altitude_std = sqrt(_data[y][x].altitude_std);
            }
            _data[y][x].count = countp1;
            _data[y][x].lift_time = _global_life_time;
            if (point.x < _data[y][x].min_x) {
                _data[y][x].min_x = point.x;
            }
            if (point.x > _data[y][x].max_x) {
                _data[y][x].max_x = point.x;
            }
            if (point.y < _data[y][x].min_y) {
                _data[y][x].min_y = point.y;
            }
            if (point.y > _data[y][x].max_y) {
                _data[y][x].max_y = point.y;
            }
        }

        if (map_indices != nullptr) {
            if (_data[y][x].count > 0) { // exist in map
                map_indices->at(i) = y * _cols + x;
            } else {
                map_indices->at(i) = -1;
            }
        }
    }
    ++_global_life_time;
    return true;
}

float SppDynamicMap::query_points_in_map_ratio(
        const SppCluster& cluster,
        std::set<int>* node_indices) const {
    int cols = static_cast<int>(_cols);
    int rows = static_cast<int>(_rows);
    unsigned int count = 0;
    if (node_indices != nullptr) {
        node_indices->clear();
    }
    for (const auto& point : *cluster.points) {
        Eigen::Vector2f p(point.x, point.y);
        Eigen::Vector2i coordinate;
        if (!get_coordinate(p, &coordinate)) {
            continue;
        }
        const int& x = coordinate(0);
        const int& y = coordinate(1);
        if (x < 0 || x >= cols || y < 0 || y >= rows) {
            continue;
        }
        if (_data[y][x].count > 0) {
            ++count;
        }
        if (node_indices != nullptr) {
            node_indices->insert(y * cols + x);
        }
    }
    return static_cast<float>(count) / cluster.points->size();
}

bool SppDynamicMap::clean_map(const std::set<int>& node_indices) {
    for (auto& id : node_indices) {
        memset(&_data[0][id], 0, sizeof(SppMapNode));
        _data[0][id].min_x = FLT_MAX;
        _data[0][id].max_x = -FLT_MAX;
        _data[0][id].min_y = FLT_MAX;
        _data[0][id].max_y = -FLT_MAX;
    }
    return true;
}

}  // namespace perception
}  // namespace apollo
