#ifndef ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_ENGINE_H
#define ADU_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_COMMON_POST_SPP_ENGINE_H
#include <Eigen/Dense>
//#include "obstacle/vlp_lidar/segmentation/common/post_ver_2_0/include/spp_seg_2d.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_cc_2d.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cluster.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_mst.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_configure.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_cloud_mask.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_sequence.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_seg_mst_seq.h"
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_label_map.h"

namespace apollo {
namespace perception {

struct SppData {
    float* obs_prob_data = nullptr;
    float* offset_data = nullptr;
    float* confidence_data = nullptr;
    float* z_data = nullptr;
    float* class_prob_data = nullptr;
    float* heading_data = nullptr;

    float** obs_prob_data_ref = nullptr;
    float* transformed_offset_data_ref = nullptr;

    float confidence_threshold = 0.f;
    float top_z_threshold = 0.f;

    unsigned int class_num = 0;
    unsigned int data_width = 0;
    unsigned int data_height = 0;
    unsigned int data_size = 0;
    float data_range = 0;

    void make_reference(unsigned int width, unsigned int height, float range);
    void transform_offset_data();
    ~SppData();
};

class SppEngine {
public:
    //typedef SppOccDetector<float> KMeansDetector;
    typedef SppCCDetector CCDetector;
public:
    SppEngine() : //_detector_2d_kmeans(nullptr), 
        _detector_2d_cc(nullptr), 
        _width(0), _height(0), _range(0) {
    }
    ~SppEngine() {
        //if (_detector_2d_kmeans) {
        //    delete _detector_2d_kmeans;
        //}
        if (_detector_2d_cc) {
            delete _detector_2d_cc;
        }
    }
    void init(unsigned int width, unsigned int height, float range);
    void init(unsigned int width, unsigned int height, float range, 
            const std::string& config_file);

    unsigned int process(const SppData* data,
            pcl_util::PointCloudConstPtr point_cloud);

    //std::pair<unsigned int, unsigned int> process_joint_cloud(const SppData* data,
    std::vector<unsigned int> process_joint_cloud(const SppData* data,
            pcl_util::PointCloudConstPtr full_point_cloud,
            pcl_util::PointCloudPtr& his_full_point_cloud,
            pcl_util::PointIndicesPtr roi_indices,
            pcl_util::PointIndicesPtr roi_non_ground_indices,
            pcl_util::PointIndicesPtr roughness_indices,
            const int history_roughness_size,
            const Eigen::Matrix4d* pose_v2w = nullptr);

    inline const SppClusterList& get_clusters() const {
        return _clusters;
    } 
    inline SppClusterList& get_clusters() { return _clusters;
    }
    inline std::vector<SppCluster>& get_raw_clusters() {
        return _clusters.get_clusters();
    }
    inline const unsigned short* get_labels_2d() const {
        return _labels_2d.get_label_map()[0];
    }
    inline const SppSegMst& get_mst() const {
        return _mst;
    }
private:
    unsigned int process(const SppData* data, 
            pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask);

    unsigned int process_mst(pcl_util::PointCloudConstPtr point_cloud,
            const SppCloudMask& mask);

    unsigned int process_sequence_mst(pcl_util::PointCloudConstPtr point_cloud,
            SppCloudMask* mask,
            const Eigen::Matrix4d& pose_v2w);

    unsigned int process_roughness_sequence_mst(pcl_util::PointCloudConstPtr point_cloud,
            SppCloudMask* mask,
            const Eigen::Matrix4d& pose_v2w);
private:
    //KMeansDetector* _detector_2d_kmeans;
    CCDetector* _detector_2d_cc;

    unsigned int _width;
    unsigned int _height;
    float _range;

    LabelMap _labels_2d;

    SppClusterList _clusters;
    SppClusterList _mst_clusters;
    SppSegMst _mst;
    SppCloudMask _mask;
    SppSequence _sequence;
    SppSegMstSeq _mst_seq;

    SppConfigure _configure;

    std::list<std::pair<pcl_util::PointCloudPtr, Eigen::Matrix4d>> _history_roughness_list;
    SppSequence _roughness_sequence;
    SppSegMstSeq _roughness_mst_seq;
    SppCloudMask _roughness_mask;
    SppClusterList _roughness_mst_clusters;
    int _history_roughness_size;
    pcl_util::PointCloudPtr _his_full_point_cloud;
};

}  // namespace perception
}  // namespace apollo
#endif
