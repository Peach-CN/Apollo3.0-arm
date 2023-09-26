#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_DYNAMIC_MAP_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_DYNAMIC_MAP_H
#include <Eigen/Dense>
#include <set>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/i_alloc.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cloud_mask.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"

namespace apollo {
namespace perception {

struct SppMapNode {
    // altitude of samples in this node
    float altitude_mean = 0.f;
    float altitude_std = 0.f;
    // node id in map
    unsigned int id = 0;
    // sample count
    unsigned int count = 0;
    // latest time appeared in map
    unsigned int lift_time = 0;
    // reserved for segmentation modules
    unsigned int label = 0;
    // four boundary points
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
};

class SppDynamicMap {
public:
    SppDynamicMap() = default;
    ~SppDynamicMap();
    void init(unsigned int rows, unsigned int cols, float resolution);
    void release();
    void reset();
    inline unsigned int rows() const {
        return _rows;
    }
    inline unsigned int cols() const {
        return _cols;
    }
    inline unsigned int size() const {
        return _rows * _cols;
    }
    inline float resolution() const {
        return _resolution;
    }
    inline SppMapNode* operator[] (int row) {
        return _data[row];
    }
    inline const SppMapNode* operator[] (int row) const {
        return _data[row];
    }
    inline bool is_first_update() const {
        return _global_life_time == 1;
    }
    inline unsigned int id_count() const {
        return _id_count;
    }
    // note the input point should be in the world frame
    bool get_coordinate(const Eigen::Vector2f& point, Eigen::Vector2i* coordinate) const;
    // update local map to make sure that center position is in the 
    // center of the map
    void update_map(const Eigen::Vector2f& center, bool shift = false);
    // shift local map with offset 
    void shift_map(const Eigen::Vector2i& shift_dst_to_src);
    // shift local id to make continuous
    void shift_indices();
    // clean map with lift time and #observations
    void clean_map();
    // increase map life time
    void maintain_map();
    // add point cloud in world frame
    bool add_point_cloud(
            const pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask,
            std::vector<int>* map_indices);
    // query the percentage of points of a cluster in the dynamic map
    float query_points_in_map_ratio(const SppCluster& cluster,
            std::set<int>* node_indices) const;
    // clean the map nodes hit by the cluster points
    bool clean_map(const std::set<int>& node_indices);
private:
	// origin of the world frame, will be set as the position of the first frame
    Eigen::Vector2f _origin;
    bool _origin_set_flag = false;
    // offset of the local xy coordinate
    Eigen::Vector2i _offset;
    // map parameter
    float _resolution = 0.f;
    unsigned int _rows = 0;
    unsigned int _cols = 0;
    int _half_rows = 0;
    int _half_cols = 0;
    // map data, reserve double buffer to make a simple memory pool
    SppMapNode** _data = nullptr; 
    SppMapNode** _buffer[2] = {nullptr, nullptr};
    unsigned char** _range_mask = nullptr;

    unsigned int _id_count = 0;
    unsigned int _global_life_time = 0;
private:
    static const unsigned int _s_clean_disappear_time = 5;
private:
    static constexpr float _s_min_height = 0.4;
    static constexpr float _s_max_height = 2.f;
    static constexpr float _s_cumulated_range = 15.f;
    static constexpr float _s_ignore_range = 0.f;
};

}  // namespace perception
}  // namespace apollo
#endif
