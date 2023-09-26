#include <pcl/common/transforms.h>
#include "modules/perception/obstacle/lidar/segmentation/spp_common/spp_sequence.h"

namespace apollo {
namespace perception {

pcl_util::PointCloudPtr SppSequence::get_fused_cloud() {
    std::vector<SppFramePtr> frames;
    _buffer.get_all_data(&frames);
    if (_buffer.size() == 0) {
        return nullptr;
    }

    Eigen::Matrix4d cur_pose = frames[0]->pose_v2w;
    Eigen::Vector3d offset = cur_pose.block<3, 1>(0, 3);
    cur_pose.block<3, 1>(0, 3) -= offset;

    pcl_util::PointCloudPtr fused_cloud(new pcl_util::PointCloud);
    pcl::copyPointCloud(*frames[0]->cloud, *fused_cloud);
    for (unsigned int i = 1; i < frames.size(); ++i) {
        Eigen::Matrix4d pose = frames[i]->pose_v2w;
        pose.block<3, 1>(0, 3) -= offset;
        Eigen::Matrix4d relative_pose = cur_pose.inverse() * pose;
        pcl_util::PointCloud cloud;
        pcl::transformPointCloud(*frames[i]->cloud, cloud, relative_pose);
        *fused_cloud += cloud;
    }
    return fused_cloud;
}

pcl_util::PointCloudPtr SppSequence::get_fused_cloud(unsigned int num) {
    std::vector<SppFramePtr> frames;
    //_buffer.get_all_data(&frames);
    _buffer.get_last_n_data(&frames, num);
    if (_buffer.size() == 0) {
        return nullptr;
    }

    Eigen::Matrix4d cur_pose = frames[0]->pose_v2w;
    Eigen::Vector3d offset = cur_pose.block<3, 1>(0, 3);
    cur_pose.block<3, 1>(0, 3) -= offset;

    pcl_util::PointCloudPtr fused_cloud(new pcl_util::PointCloud);
    pcl::copyPointCloud(*frames[0]->cloud, *fused_cloud);
    for (unsigned int i = 1; i < frames.size(); ++i) {
        Eigen::Matrix4d pose = frames[i]->pose_v2w;
        pose.block<3, 1>(0, 3) -= offset;
        Eigen::Matrix4d relative_pose = cur_pose.inverse() * pose;
        pcl_util::PointCloud cloud;
        pcl::transformPointCloud(*frames[i]->cloud, cloud, relative_pose);
        *fused_cloud += cloud;
    }
    return fused_cloud;
}

pcl_util::PointCloudPtr SppSequence::get_world_fused_cloud() {
    std::vector<SppFramePtr> frames;
    _buffer.get_all_data(&frames);
    if (_buffer.size() == 0) {
        return nullptr;
    }

    pcl_util::PointCloudPtr fused_cloud(new pcl_util::PointCloud);
    pcl::transformPointCloud(*frames[0]->cloud, *fused_cloud, frames[0]->pose_v2w);
    for (unsigned int i = 1; i < frames.size(); ++i) {
        pcl_util::PointCloud cloud;
        pcl::transformPointCloud(*frames[i]->cloud, cloud, frames[i]->pose_v2w);
        *fused_cloud += cloud;
    }
    return fused_cloud;
}

pcl_util::PointCloudPtr SppSequence::get_world_fused_cloud(unsigned int num) {
    std::vector<SppFramePtr> frames;
    //_buffer.get_all_data(&frames);
    _buffer.get_last_n_data(&frames, num);
    if (_buffer.size() == 0) {
        return nullptr;
    }

    pcl_util::PointCloudPtr fused_cloud(new pcl_util::PointCloud);
    pcl::transformPointCloud(*frames[0]->cloud, *fused_cloud, frames[0]->pose_v2w);
    for (unsigned int i = 1; i < frames.size(); ++i) {
        pcl_util::PointCloud cloud;
        pcl::transformPointCloud(*frames[i]->cloud, cloud, frames[i]->pose_v2w);
        *fused_cloud += cloud;
    }
    return fused_cloud;
}

pcl_util::PointCloudPtr SppSequence::get_latest_world_cloud() {
    SppFramePtr frame = *_buffer.get_latest_data();
    Eigen::Matrix4d pose = frame->pose_v2w;
    pcl_util::PointCloudPtr world_cloud(new pcl_util::PointCloud);
    pcl::transformPointCloud(*frame->cloud, *world_cloud, pose);
    return world_cloud;
}

void SppSequence::transform_cloud_to_world(SppCluster* cluster) {
    if (cluster == nullptr) {
        return;
    }
    SppFramePtr frame = *_buffer.get_latest_data();
    Eigen::Matrix4d pose = frame->pose_v2w;
    Eigen::Vector3d xyz;
    for (auto& point : *cluster->points) {
        xyz(0) = point.x;
        xyz(1) = point.y;
        xyz(2) = point.z;
        xyz = pose.block<3, 3>(0, 0) * xyz + pose.block<3, 1>(0, 3);
        point.x = static_cast<float>(xyz(0));
        point.y = static_cast<float>(xyz(1));
        point.z = static_cast<float>(xyz(2));
    }
}

}  // namespace perception
}  // namespace apollo

