#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_MST_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_SEG_MST_H
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster_list.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cell_matrix.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/segment_graph.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/disjoint.h"

namespace apollo {
namespace perception {

class SppSegMst {
public:
    //typedef ConnectedComponent::Edge Edge;
    //typedef Universe Universe;
public:
    SppSegMst() : _label_image(nullptr) {
    }
    ~SppSegMst() {
        if (_label_image) {
            idl::i_free2(_label_image);
        }
    }
    inline void init(unsigned int width, unsigned int height, float range, float c) {
        _image.init(height, width, range);
        if (_label_image) {
            idl::i_free2(_label_image);
        }
        _label_image = idl::i_alloc2<unsigned short>(height, width);
        memset(_label_image[0], 0, sizeof(unsigned short) * width * height);
        _graph_segmentor.init(c);
    }
    inline const unsigned short* get_label_image() const {
        return _label_image[0];
    }
    inline const SppCellMatrix& get_cell_matrix() const {
        return _image;
    }
    inline void reset() {
        _image.reset();
    }
    inline void add_point_cloud(pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask) {
        _image.add_point_cloud(point_cloud, mask);
    }
    unsigned int segmentation(
            pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask,
            SppClusterList& clusters,
            int radius,
            bool vis_label);
private:
    SppCellMatrix _image;
    unsigned short** _label_image;
    std::vector<Edge> _mst_edges;
    GraphSegmentor _graph_segmentor;
};

}  // namespace perception
}  // namespace apollo
#endif
