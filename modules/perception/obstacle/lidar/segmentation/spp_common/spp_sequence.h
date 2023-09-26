#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEQUENCE_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEQUENCE_H
#include <Eigen/Dense>
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cloud_mask.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"

namespace apollo {
namespace perception {

struct SppFrame {
    unsigned int id = 0;
    pcl_util::PointCloudPtr cloud;
    Eigen::Matrix4d pose_v2w;
};
typedef std::shared_ptr<SppFrame> SppFramePtr;
typedef std::shared_ptr<const SppFrame> SppFrameConstPtr;

template <class T>
class SppCircularVector {
public:
    SppCircularVector() = default;
    void init(unsigned int size) {
        _data.resize(size);
        _size = size;
        _idx = 0;
        _count = 0;
    }
    void add_data(const T& data) {
        _data[_idx] = data;
        _idx = (_idx + 1) % _size;
        ++_count;
        _count = std::min(_count, _size);
    }
    T* get_latest_data() {
        if (!_count) {
            return nullptr;
        }
        return &_data[(_idx + _size - 1) % _size];
    }
    void get_last_n_data(std::vector<T>* buffer, unsigned int n) {
        if (buffer == nullptr) {
            return;
        }
        buffer->clear();
        for (unsigned int i = 0, id = (_idx + _size - 1) % _size;
                i < std::min(n, _count);
                id = (id + _size - 1) % _size, ++i) {
            buffer->push_back(_data[id]);
        }
    }
    void get_all_data(std::vector<T>* buffer) {
        return get_last_n_data(buffer, _count);
    }
    unsigned int size() {
        return _count;
    }
private:
    std::vector<T> _data;
    unsigned int _size = 0;
    unsigned int _idx = 0;
    unsigned int _count = 0;
};

class SppSequence {
public:
    SppSequence() = default;
    void init(unsigned int window_size) {
        _buffer.init(window_size);
    }
    void add_frame(SppFramePtr frame_ptr) {
        if (_idx == 0) {
            _offset = frame_ptr->pose_v2w.block<3, 1>(0, 3);
        }
        frame_ptr->id = _idx++;
        frame_ptr->pose_v2w.block<3, 1>(0, 3) -= _offset;
        _buffer.add_data(frame_ptr);
    }
    pcl_util::PointCloudPtr get_fused_cloud();
    pcl_util::PointCloudPtr get_fused_cloud(unsigned int num);
    pcl_util::PointCloudPtr get_world_fused_cloud();
    pcl_util::PointCloudPtr get_world_fused_cloud(unsigned int num);
    pcl_util::PointCloudPtr get_latest_world_cloud();
    //pcl_util::PointCloudPtr get_latest_world_cloud(unsigned int num);
    void transform_cloud_to_world(SppCluster* cluster);
private:
    SppCircularVector<SppFramePtr> _buffer;
    unsigned int _idx = 0;
    Eigen::Vector3d _offset;
};

}  // namespace perception
}  // namespace apollo

#endif
