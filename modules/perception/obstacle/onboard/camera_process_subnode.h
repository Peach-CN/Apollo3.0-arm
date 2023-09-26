/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_SUBNODE_H_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "yaml-cpp/yaml.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/cuda_util/util.h"
#include "modules/perception/lib/base/singleton.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/converter/geometry_camera_converter.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"
#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/camera/filter/object_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_converter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"
#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker.h"
#include "modules/perception/obstacle/camera/transformer/flat_camera_transformer.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/traffic_light/util/color_space.h"

namespace apollo {
namespace perception {

namespace zkhy {
enum RecognitionType {
  INVALID = 0,
  VEHICLE,
  PEDESTRIAN,
  CHILD,
  BICYCLE,
  MOTO,
  TRUCK,
  BUS,
  OTHERS,
  ESTIMATED,
  CONTINUOUS
};

struct OutputObstacles {
  float currentSpeed;                 //(m/second) current frame self vehicle speed
  float frameRate;                    //(fps) frames per second

  unsigned char trackId;              //current obstacle corresponding tracking id in obstacle tracking buffer
  unsigned char trackFrameNum;        //the track frame numbers of the obstacle, increments in 1 each frame until to the max record number, the actual number is (trackFrameNum + 1)
  unsigned char stateLabel;           //classify obstacle for front collision, FC, 0: invalid or continuous obstacle, 1: nearest o  bstacle in warning area, 2: obstacle in waning area, 3: obstacle out of warning area
  unsigned char classLabel;           //the obstacle class label: 0-invalid; 1-warning obstacle; 2-obstacles to be warned; 3-non warning obstacle; 4-left continuous obstacle; 5-right continuous obstacle; 6-estimated vanish obstacle; 7-valid obstacle // original, the obstacle class label: 0-invalid; 1-car; 2-person; 3-continuous obstacle; 4-valid; 5-other
  unsigned char continuousLabel;      //the continuous obstacle class label: 0-invalid; 1-left continuous obstacle; 2-right continuous obstacle
  unsigned char fuzzyEstimationValid; //(0/1) 0: current fuzzy estimation is invalid; 1: current fuzzy estimation is valid
  RecognitionType obstacleType;       //the obstacle Type: INVALID=0,VEHICLE, PEDESTRIAN, ...

  float avgDisp;                      //(pixel) the average disparity of an obstacle with adding infDisp: avgDisp = BF_VALUE/avgDitance+infDis
  float avgDistanceZ;                 //(m) the average Z distance of single obstacle rectangle
  float nearDistanceZ;                //(m) the minimum Z distance of continuous obstacle
  float farDistanceZ;                 //(m) the longest Z distance of continuous obstacle

  float real3DLeftX;                  //(-+m) the left X for real 3D coordinate of the obstacle(the origin X is the center of car, right is positive)
  float real3DRightX;                 //(-+m) the right X for real 3D coordinate of the obstacle(the origin X is the center of car, right is positive)
  float real3DCenterX;                //(-+m) the center X for real 3D coordinate of the obstacle(the origin X is the center of car, right is positive)
  float real3DUpY;                    //(-+m) the up Y for real 3D coordinate of the obstacle(the origin Y is the camera position, down is positive)
  float real3DLowY;                   //(-+m) the Low y for real 3D coordinate of the obstacle(the origin Y is the camera position, down is positive)

  unsigned short firstPointX;         //(pixel) the X-axis of first point of rectangle, the first point :(x, y), left top point of single obstacle/near bottom point of continuous obstacle, full size pixel coordinate
  unsigned short firstPointY;         //(pixel) the Y-axis of first point of rectangle, the first point :(x, y), left top point of single obstacle/near bottom point of continuous obstacle, full size pixel coordinate
  unsigned short secondPointX;        //(pixel) the X-axis of second point of rectangle, the second point:(x+width, y), right top point of single obstacle/near top point of continuous obstacle, full size pixel coordinate
  unsigned short secondPointY;        //(pixel) the Y-axis of second point of rectangle, the second point:(x+width, y), right top point of single obstacle/near top point of continuous obstacle, full size pixel coordinate
  unsigned short thirdPointX;         //(pixel) the X-axis of third point of rectangle, the third point :(x+width, y+height), right bottom point of single obstacle/far top point of continuous obstacle, full size pixel coordinate
  unsigned short thirdPointY;         //(pixel) the Y-axis of third point of rectangle, the third point :(x+width, y+height), right bottom point of single obstacle/far top point of continuous obstacle, full size pixel coordinate
  unsigned short fourthPointX;        //(pixel) the X-axis of fourth point of rectangle, the fourth point:(x,y+height), left bottom point of single obstacle/far bottom point of continuous obstacle, full size pixel coordinate
  unsigned short fourthPointY;        //(pixel) the Y-axis of fourth point of rectangle, the fourth point:(x,y+height), left bottom point of single obstacle/far bottom point of continuous obstacle, full size pixel coordinate

  float fuzzyRelativeDistanceZ;       //(m) estimated relative distance in Z direction
  float fuzzyRelativeSpeedZ;          //(m/second) estimated speed in Z direction of current obstacle
  float fuzzyCollisionTimeZ;          //(second) estimated collision time in Z direction

  unsigned char fuzzyCollisionX;      //(0/1) estimated whether there is collision in X direction
  float fuzzy3DWidth;                 //(m) estimated real 3D width of current obstacle
  float fuzzy3DCenterX;               //(-+m) estimated real 3D position of obstacle center in X direction (the origin X is the center of car, right is positive)
  float fuzzy3DLeftX;                 //(-+m) estimated real 3D position of obstacle left in X direction (the origin X is the center of car, right is positive)
  float fuzzy3DRightX;                //(-+m) estimated real 3D position of obstacle right in X direction (the origin X is the center of car, right is positive)
  float fuzzy3DHeight;                //(m) estimated real 3D height of current obstacle
  float fuzzy3DUpY;                   //(-+m) estimated real 3D position of obstacle up in Y direction (the origin Y is the camera position, down is positive)
  float fuzzy3DLowY;                  //(-+m) estimated real 3D position of obstacle low in Y direction (the origin Y is the camera position, down is positive)

  float fuzzyRelativeSpeedCenterX;    //(m/second) estimated center speed in X direction of current obstacle
  float fuzzyRelativeSpeedLeftX;      //(m/second) estimated left speed in X direction of current obstacle
  float fuzzyRelativeSpeedRightX;     //(m/second) estimated right speed in X direction of current obstacle

#ifdef EXTEND_INFO_ENABLE
  unsigned char storeId;              //current obstacle store id in obstacle detection buffer
#endif //EXTEND_INFO_ENABLE
};

static_assert(sizeof(OutputObstacles) == 128, "unmatched structure size");
}

class CameraProcessSubnode : public Subnode {
 public:
  CameraProcessSubnode() = default;
  ~CameraProcessSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  bool InitInternal() override;
  bool InitCalibration();
  bool InitModules();

  void ImgCallback(const sensor_msgs::Image& message);
  void ChassisCallback(const apollo::canbus::Chassis& message);
  void ObstacleDetectionCallback(const std_msgs::String &raw_data);

  bool MessageToMat(const sensor_msgs::Image& msg, cv::Mat* img);
  bool MatToMessage(const cv::Mat& img, sensor_msgs::Image* msg);

  void VisualObjToSensorObj(
      const std::vector<std::shared_ptr<VisualObject>>& objects,
      SharedDataPtr<SensorObjects>* sensor_objects, FilterOptions options);
  void OutputObstacleToSensorObj(const zkhy::OutputObstacles *obstacle,
      const int count, SharedDataPtr<SensorObjects>* sensor_objects,
      FilterOptions options);

  void PublishDataAndEvent(const double timestamp,
                           const SharedDataPtr<SensorObjects>& sensor_objects,
                           const SharedDataPtr<CameraItem>& camera_item);

  void PublishPerceptionPbObj(
      const SharedDataPtr<SensorObjects>& sensor_objects);
  void PublishPerceptionPbLnMsk(const cv::Mat& mask,
                                const sensor_msgs::Image& message);

  void ObjectCopy(std::shared_ptr<VisualObject> from, std::shared_ptr<VisualObject> &to, int isnew);
  // General
  std::string device_id_ = "camera";
  SeqId seq_num_ = 0;
  double timestamp_ns_ = 0.0;

  // Shared Data
  CameraObjectData* cam_obj_data_;
  CameraSharedData* cam_shared_data_;

  // Calibration
  int32_t image_height_ = 1080;
  int32_t image_width_ = 1920;
  Eigen::Matrix4d camera_to_car_;
  Eigen::Matrix<double, 3, 4> intrinsics_;

  // Dynamic calibration based on objects
  // Always available, but retreat to static one if flag is false
  bool adjusted_extrinsics_ = false;
  Eigen::Matrix4d camera_to_car_adj_;
  Eigen::Matrix4d camera_to_world_;

  // Publish to Perception Protobuf and ROS topic
  bool pb_obj_ = false;  // Objects
  apollo::canbus::Chassis chassis_;
  bool pb_ln_msk_ = false;  // Lane marking mask
  float ln_msk_threshold_ = 0.95f;
  const int num_lines = 13;

  // Modules
  std::unique_ptr<BaseCameraDetector> detector_;
  std::unique_ptr<BaseCameraConverter> converter_;
  std::unique_ptr<BaseCameraTracker> tracker_;
  std::unique_ptr<BaseCameraTransformer> transformer_;
  std::unique_ptr<BaseCameraFilter> filter_;
  std::unique_ptr<sensor_msgs::PointCloud2> pointmsg_;
  std::vector<std::shared_ptr<VisualObject>> saveobjects_;

  // Thirdparty Obstacle Detection
  std::unique_ptr<ros::NodeHandle> obstacle_node_handle_;
  ros::Subscriber obstacle_detection_sub_;
};

REGISTER_SUBNODE(CameraProcessSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_SUBNODE_H_
