#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_MST_SEQ_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_MST_SEQ_H
#include "modules/perception/obstacle/lidar/segmentation/spp_common/segment_graph.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_dynamic_map.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster_list.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/disjoint.h"

#include <vector>
#include <unordered_set>
#include <unordered_map>

namespace apollo {
namespace perception {

class SppSegMstSeq {
public:
    //typedef ConnectedComponent::Edge Edge;
    //typedef Universe Universe;
public:
    SppSegMstSeq() = default;
    ~SppSegMstSeq() = default;
    void init(unsigned int width, unsigned int height, float resolution, float c);
    void init(unsigned int width, unsigned int height, float resolution,
        float c, float max_dist_threshold);

    void add_point_cloud(pcl_util::PointCloudConstPtr world_cloud,
            const SppCloudMask& mask,
            const Eigen::Matrix4d& pose_v2w); 
    unsigned int segmentation(
            pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask,
            SppClusterList& clusters,
            int radius);
    inline const SppDynamicMap& get_dynamic_map() const {
        return _dynamic_map;
    }
    inline SppDynamicMap& get_dynamic_map() {
        return _dynamic_map;
    }

private:
    float distance(const SppMapNode& lhs, const int& xl, const int& yl,
            const SppMapNode& rhs, const int& xr, const int& yr) const;
    float distance_seq(const SppMapNode& lhs, const int& xl, const int& yl,
            const SppMapNode& rhs, const int& xr, const int& yr) const;
    void merge_overlapping_segments(unsigned int num_labels);
private:
    SppDynamicMap _dynamic_map;
    std::vector<Edge> _mst_edges;
    std::vector<int> _indices;
    GraphSegmentor _graph_segmentor;

    std::unordered_map<int, std::vector<float>> _id_map;
    float _max_dist_threshold;
};

}  // namespace perception
}  // namespace apollo

#endif
