#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CLOUD_MASK_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CLOUD_MASK_H
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"

namespace apollo {
namespace perception {

class SppCloudMask {
public:
    SppCloudMask() = default;
    inline void set_size(unsigned int size, int init_value) {
        _mask.assign(size, init_value);
    }
    inline void fill_all(int value) {
        std::fill(_mask.begin(), _mask.end(), value);
    }
    inline const std::vector<int>& get_mask() {
        return _mask;
    }
    inline int& operator[](int id) {
        return _mask[id];
    }
    inline const int& operator[](int id) const {
        return _mask[id];
    }
    inline const unsigned int size() const {
        return _mask.size();
    }
    inline void clear() {
        _mask.clear();
    }
    unsigned int valid_indices_count();

    pcl_util::PointCloudPtr get_valid_cloud(
            pcl_util::PointCloudConstPtr src_cloud) const;

    void flip();

    void add_indices(pcl_util::PointIndicesPtr indices, int value = 1);
    void add_indices(std::vector<SppPoint>& points, int value = 1);
    void add_indices_2nd(pcl_util::PointIndicesPtr indices, 
            pcl_util::PointIndicesPtr indices_of_indices,
            int value = 1);

    void minus_indices(pcl_util::PointIndicesPtr indices);
    void minus_indices(std::vector<SppPoint>& points);
    void minus_indices_2nd(pcl_util::PointIndicesPtr indices, 
            pcl_util::PointIndicesPtr indices_of_indices);

    void get_valid_mask(SppCloudMask* rhs) const;
    void reset_value(int value, int target_value);
   private:
    std::vector<int> _mask;
};

}  // namespace perception
}  // namespace apollo
#endif
