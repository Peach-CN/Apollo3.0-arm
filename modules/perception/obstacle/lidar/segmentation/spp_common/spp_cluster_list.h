#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CLUSTER_LIST_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_CLUSTER_LIST_H
#include <vector>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"

namespace apollo {
namespace perception {

class SppClusterList {
public:
    SppClusterList() {
        _clusters.reserve(1000);
    }
    inline void init(unsigned int size) {
        _clusters.clear();
        _clusters.resize(size);
    }
    inline void reset() {
        _clusters.clear();
    }
    inline void resize(unsigned int size) {
        _clusters.resize(size);
    }
    inline void add_sample(unsigned int cluster_id, const pcl_util::Point& point,
            int point_id) {
        _clusters[cluster_id].add_sample(point, point_id);
    }
    inline std::vector<SppCluster>& get_clusters() {
        return _clusters;
    }
    inline unsigned int size() const {
        return _clusters.size();
    }
    inline void set_output_score(bool output) {
        SppCluster::output_score = output;
    }
    inline SppCluster& operator[](int id) {
        return _clusters[id];
    }
    inline const SppCluster& operator[](int id) const {
        return _clusters[id];
    }
    unsigned int move_from_tail(unsigned int move_num, SppClusterList& rhs);
    void merge(SppClusterList& rhs);
    unsigned int analyze_clusters(float max_gap, unsigned int start_id = 0, 
            bool select_max = false); 
    bool compute_altitude_and_split_cluster(unsigned int id, float max_gap, bool select_max); 
    friend std::ostream& operator << (std::ostream& out, const SppClusterList& rhs);
    void remove_empty_clusters();
    void filter_clusters(float threshold, 
            unsigned int start_id, 
            const float* data);
    void filter_clusters(float threshold, 
            SppClusterList& rhs, unsigned int start_id,
            const float* data);
    void filter_clusters(float threshold, 
            SppCluster& rhs, unsigned int start_id, 
            const float* data);
    void split_into_another_cluster(SppClusterList& rhs, 
            unsigned int this_id);
    void erase_cluster(unsigned int id);
private:
    std::vector<SppCluster> _clusters;
};

}  // namespace perception
}  // namespace apollo

#endif
