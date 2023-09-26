#include <map>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_mst.h"

namespace apollo {
namespace perception {

unsigned int SppSegMst::segmentation(
        pcl_util::PointCloudConstPtr point_cloud,
        const SppCloudMask& mask,
        SppClusterList& clusters,
        int radius,
        bool vis_label) {

    int width = static_cast<int>(_image.get_cols());
    int height = static_cast<int>(_image.get_rows());

    Edge edge_t;
    _mst_edges.clear();
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (_image[y][x].count == 0) {
                continue;
            } 
            edge_t.a = _image[y][x].id;
            int nx_begin = std::max(x - radius, 0);
            int ny_begin = std::max(y - radius, 0);
            int nx_end = std::min(x + radius, width - 1);
            int ny_end = std::min(y + radius, height - 1);
            for (int ny = ny_begin; ny <= ny_end; ++ny) {
                for (int nx = nx_begin; nx <= nx_end; ++nx) {
                    if (_image[ny][nx].count > 0 && (nx > x || ny > y)) {
                        edge_t.b = _image[ny][nx].id;  
                        edge_t.w = _image[y][x].distance(_image[ny][nx]);
                        _mst_edges.push_back(edge_t);
                    }
                }
            }
        }
    }
    //Universe* universe = segment_graph(_image.get_label_count(), 
    //            _mst_edges.size(), _mst_edges.data(), c);
    _graph_segmentor.segment_graph(_image.get_label_count(), 
                _mst_edges.size(), _mst_edges.data());
    Universe* universe = _graph_segmentor.get_universe();

    std::vector<unsigned int> mapping(_image.get_label_count(), 0);
    std::map<unsigned int, unsigned int> unique;
    std::map<unsigned int, unsigned int>::iterator iter;
    unsigned int label = 0;
    unsigned int size = clusters.size();
    for (int i = 0; i < static_cast<int>(_image.get_label_count()); ++i) {
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
    unsigned int size_plus = size + unique.size();
    clusters.resize(size_plus); //additional one cluster for out-of-range points

    if (vis_label) {
        memset(_label_image[0], 0, sizeof(unsigned short) * width * height);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (_image[y][x].count == 0) {
                    continue;
                } 
                _label_image[y][x] = mapping[_image[y][x].id] - size + 1;
            }
        }
    }

    float half_width = static_cast<float>(_image.get_cols()) / 2; 
    float half_height = static_cast<float>(_image.get_rows()) / 2;
    float range = _image.get_range();
    float ratio_x = half_width / range;
    float ratio_y = half_height / range;
    std::vector<pcl_util::Point> single_points;
    std::vector<int> single_point_ids;
    for (unsigned int i = 0; i < point_cloud->points.size(); ++i) {
        if (mask.size() && mask[i] == 0) {
            continue;
        }
        const pcl_util::Point& point = point_cloud->points[i];
        if (point.x * point.x + point.y * point.y < 25.f) {
            continue;
        }
        int y = static_cast<int>(std::floor((range - point.x) * ratio_x));
        int x = static_cast<int>(std::floor((range - point.y) * ratio_y));
        if (x < 0 || x >= width || y < 0 || y >= height) {
            //single_points.push_back(point);
            //single_point_ids.push_back(static_cast<int>(i));
        }
        else {
            clusters[mapping[_image[y][x].id]].add_sample(point, i);
        }
    }
    if (single_points.size() > 0) {
        unsigned int cur_size = size_plus;
        size_plus += single_points.size();
        clusters.resize(size_plus);
        for (unsigned int i = 0; i < single_points.size(); ++i) {
            clusters[cur_size+i].add_sample(single_points[i], single_point_ids[i]);
        }
    }
    return size_plus - size;
}

}  // namespace perception
}  // namespace apollo
