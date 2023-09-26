#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cloud_mask.h"

namespace apollo {
namespace perception {

pcl_util::PointCloudPtr SppCloudMask::get_valid_cloud(
        pcl_util::PointCloudConstPtr src_cloud) const {
    if (_mask.size() != src_cloud->points.size()) {
        return nullptr;
    }
    pcl_util::PointCloudPtr dst_cloud(new pcl_util::PointCloud);
    dst_cloud->points.reserve(src_cloud->points.size() / 2);
    for (unsigned int i = 0; i < _mask.size(); ++i) {
        if (_mask[i]) {
            dst_cloud->points.push_back(src_cloud->points[i]);
        }
    }
    return dst_cloud;
}

void SppCloudMask::add_indices(pcl_util::PointIndicesPtr indices, int value) {
    for (auto& id : indices->indices) {
        _mask[id] = value;
    }
}
void SppCloudMask::add_indices(std::vector<SppPoint>& points, int value) {
    for (auto& point : points) {
        _mask[point.point_id] = value;
    }
}
void SppCloudMask::add_indices_2nd(pcl_util::PointIndicesPtr indices, 
        pcl_util::PointIndicesPtr indices_of_indices,
        int value) {
    for (auto& id : indices_of_indices->indices) {
        _mask[indices->indices[id]] = value;
    }
}
void SppCloudMask::minus_indices(pcl_util::PointIndicesPtr indices) {
    for (auto& id : indices->indices) {
        _mask[id] = 0;
    }
}
void SppCloudMask::minus_indices(std::vector<SppPoint>& points) {
    for (auto& point : points) {
        _mask[point.point_id] = 0;
    }
}
void SppCloudMask::minus_indices_2nd(pcl_util::PointIndicesPtr indices, 
        pcl_util::PointIndicesPtr indices_of_indices) {
    for (auto& id : indices_of_indices->indices) {
        _mask[indices->indices[id]] = 0;
    }
}

unsigned int SppCloudMask::valid_indices_count() {
    unsigned int count = 0;
    for (auto& i : _mask) {
        if (i > 0) {
            ++count;
        }
    }
    return count;
}

void SppCloudMask::flip() {
    for (auto& i : _mask) {
        i = i > 0 ? 0 : 1;
    }
}

void SppCloudMask::get_valid_mask(SppCloudMask* rhs) const {
    if (rhs == nullptr) {
        return;
    }
    rhs->clear();
    for (auto& i : _mask) {
        if (i > 0) {
            rhs->_mask.push_back(i);
        }
    }
}

void SppCloudMask::reset_value(int value, int target_value) {
    for (auto& i : _mask) {
        if (i == value) {
            i = target_value;
        }
    }
}

}  // namespace perception
}  // namespace apollo

