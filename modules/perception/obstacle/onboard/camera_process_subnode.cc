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

#include "modules/perception/obstacle/onboard/camera_process_subnode.h"
#include <opencv2/core/version.hpp>
#if (CV_MAJOR_VERSION == 2)
#include <opencv2/opencv.hpp>
#ifdef HAVE_OPENCV_GPU
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/gpumat.hpp>
#endif
#else
#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#endif
#include "eigen_conversions/eigen_msg.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include <chrono>
#include "modules/common/math/box2d.h"
#include "modules/common/time/time.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/file.h"
#include "modules/perception/proto/yolo_camera_detector_config.pb.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/proto/yolo.pb.h"

#define XPERF_CODE_BLOCK_BEGIN() std::chrono::system_clock::time_point _tmBegin, _tmEnd; \
                                 _tmBegin = apollo::common::time::Clock::Now();

#define XPERF_CODE_BLOCK_END(fnName) _tmEnd = apollo::common::time::Clock::Now(); \
                                 AERROR << "[" << fnName << "]" << std::chrono::duration_cast<std::chrono::milliseconds>(_tmEnd - _tmBegin).count() << " ms"; \
                                 _tmBegin = _tmEnd;

DEFINE_int32(save_framenum, 0, "");

namespace apollo {
namespace perception {

using apollo::common::util::GetAbsolutePath;
using apollo::common::util::GetProtoFromFile;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::util::Print;
using apollo::common::util::StrCat;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;

/*
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
}*/

bool CameraProcessSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

#if (CV_MAJOR_VERSION == 2)
#ifdef HAVE_OPENCV_GPU
  FLAGS_enable_opencv_gpu = cv::gpu::getCudaEnabledDeviceCount();
  if (FLAGS_enable_opencv_gpu) {
    cv::Mat cmat = cv::Mat::zeros(cv::Size(1920, 1080), CV_16UC1);
    cv::gpu::GpuMat gmat(cmat);
  }
#else
  FLAGS_enable_opencv_gpu = 0;
#endif
#else
  FLAGS_enable_opencv_gpu = cv::cuda::getCudaEnabledDeviceCount();
  if (FLAGS_enable_opencv_gpu) {
    cv::Mat cmat = cv::Mat::zeros(cv::Size(1920, 1080), CV_16UC1);
    cv::cuda::GpuMat gmat(cmat);
  }
#endif

  if (fields.count("device_id")) {
    device_id_ = fields["device_id"];
  }
  if (fields.count("pb_obj") && stoi(fields["pb_obj"])) {
    pb_obj_ = true;
  }
  if (fields.count("pb_ln_msk") && stoi(fields["pb_ln_msk"])) {
    pb_ln_msk_ = true;
  }

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData *>(
      shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  if (FLAGS_use_3rdpart_obstacle_detection) {
    obstacle_node_handle_.reset(new ros::NodeHandle("ObstacleRecvNode"));
    obstacle_detection_sub_ = obstacle_node_handle_->subscribe("/apollo/zkhy_obs", 10,
                        &CameraProcessSubnode::ObstacleDetectionCallback, static_cast<CameraProcessSubnode *>(this));
  } else {
    AdapterManager::AddImageFrontCallback(&CameraProcessSubnode::ImgCallback,
                                          this);
  }
  if (pb_obj_) {
    AdapterManager::AddChassisCallback(&CameraProcessSubnode::ChassisCallback,
                                       this);
  }
  return true;
}

void CameraProcessSubnode::ObstacleDetectionCallback(const std_msgs::String &raw_data) {
  int count = raw_data.data.size() / sizeof(zkhy::OutputObstacles);
  const zkhy::OutputObstacles *obstacle = reinterpret_cast<const zkhy::OutputObstacles *>(raw_data.data.c_str());

  PerceptionObstacles obstacles;

  if (!FLAGS_use_navigation_mode)
    AdapterManager::Observe();

  // Header
  AdapterManager::FillPerceptionObstaclesHeader("perception_obstacle",
                                                &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(TimeUtil::GetCurrentTime());
  header->set_radar_timestamp(0);
  obstacles.set_error_code(common::ErrorCode::OK);

  for (int index = 0; index < count; ++index, obstacle++) {
    if (std::fabs(obstacle->real3DLowY - obstacle->real3DUpY) < 0.2) continue;

    PerceptionObstacle *pb_obj = obstacles.add_perception_obstacle();

    pb_obj->set_id(static_cast<int>(obstacle->trackId));
    pb_obj->set_theta(0.0);

    Point* obj_center = pb_obj->mutable_position();
    obj_center->set_x(obstacle->avgDistanceZ);
    obj_center->set_y(1.0 * obstacle->real3DCenterX);
    obj_center->set_z(0.0);

    if ( !FLAGS_use_navigation_mode && !AdapterManager::GetLocalization()->Empty() ) {
      const auto& localization = AdapterManager::GetLocalization()->GetLatestObserved();
      double vx = localization.pose().position().x();
      double vy = localization.pose().position().y();
      double heading = localization.pose().heading();

      vx = vx + obj_center->x() * std::cos(heading) - obj_center->y() * std::sin(heading);
      vy = vy + obj_center->y() * std::cos(heading) + obj_center->x() * std::sin(heading);
      obj_center->set_x(vx);
      obj_center->set_y(vy);
    }
    Point* obj_velocity = pb_obj->mutable_velocity();
    obj_velocity->set_x(0.0);
    obj_velocity->set_y(0.0);
    obj_velocity->set_z(0.0);

    pb_obj->set_length(std::fabs(obstacle->real3DRightX - obstacle->real3DLeftX));
    if (obstacle->obstacleType == zkhy::RecognitionType::PEDESTRIAN)
      pb_obj->set_width(std::fabs(obstacle->real3DRightX - obstacle->real3DLeftX) * 1.2);
    else
      pb_obj->set_width(std::fabs(obstacle->real3DRightX - obstacle->real3DLeftX));
    pb_obj->set_height(std::fabs(obstacle->real3DLowY - obstacle->real3DUpY));

    Box2d object_bounding_box = {{obj_center->x(), 1.0 * obj_center->y()}, 0.0, 
       pb_obj->length(), pb_obj->width()};
    std::vector<Vec2d> corners;
    object_bounding_box.GetAllCorners(&corners);

    for (const auto& corner : corners) {
      Point* p = pb_obj->add_polygon_point();
      p->set_x(corner.x());
      p->set_y(corner.y());
      p->set_z(0.0);
    }

    pb_obj->set_confidence(1.0);
    pb_obj->set_confidence_type(PerceptionObstacle::CONFIDENCE_CNN);
    pb_obj->set_tracking_time(TimeUtil::GetCurrentTime());
    switch (obstacle->obstacleType) {
    //INVALID
    case zkhy::VEHICLE:
    case zkhy::TRUCK:
    case zkhy::BUS:
      pb_obj->set_type(static_cast<PerceptionObstacle::Type>(5));
      break;

    case zkhy::PEDESTRIAN:
    case zkhy::CHILD:
      pb_obj->set_type(static_cast<PerceptionObstacle::Type>(3));
      break;

    case zkhy::BICYCLE:
    case zkhy::MOTO:
      pb_obj->set_type(static_cast<PerceptionObstacle::Type>(4));
      break;

    case zkhy::OTHERS:
    case zkhy::ESTIMATED:
    case zkhy::CONTINUOUS:
      pb_obj->set_type(static_cast<PerceptionObstacle::Type>(2));
      break;
    }
    pb_obj->set_type(static_cast<PerceptionObstacle::Type>(3));
    pb_obj->set_timestamp(TimeUtil::GetCurrentTime());  // in seconds.
  }
  for (auto obstacle : obstacles.perception_obstacle()) {
    obstacle.mutable_velocity()->set_x(
        obstacle.velocity().x() + chassis_.speed_mps());
  }

  if (pb_obj_) AdapterManager::PublishPerceptionObstacles(obstacles);;
}

bool CameraProcessSubnode::InitCalibration() {
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  calibrator->get_image_height_width(&image_height_, &image_width_);
  camera_to_car_ = calibrator->get_camera_extrinsics();
  intrinsics_ = calibrator->get_camera_intrinsic();
  return true;
}

bool CameraProcessSubnode::InitModules() {
  RegisterFactoryYoloCameraDetector();
  RegisterFactoryRTNetYoloCameraDetector();
  RegisterFactoryGeometryCameraConverter();
  RegisterFactoryCascadedCameraTracker();
  RegisterFactoryFlatCameraTransformer();
  RegisterFactoryObjectCameraFilter();

  obstacle::yolo::YoloParam yolo_param;
  yolo_camera_detector_config::ModelConfigs config;

  CHECK(GetProtoFromFile(FLAGS_yolo_camera_detector_config, &config));
  const std::string &yolo_root = config.yolo_root();
  const std::string yolo_config = GetAbsolutePath(yolo_root,
                                             FLAGS_yolo_config_filename);
  CHECK(apollo::common::util::GetProtoFromASCIIFile(yolo_config, &yolo_param));

  const auto &model_param = yolo_param.model_param();
  const auto &model_type = model_param.model_type();
  if (!FLAGS_use_3rdpart_obstacle_detection) {
  switch (model_type) {
    case obstacle::yolo::ModelType::Caffe:
      detector_.reset(
          BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector"));
      break;

    case obstacle::yolo::ModelType::RTNet:
      detector_.reset(
          BaseCameraDetectorRegisterer::GetInstanceByName("RTNetYoloCameraDetector"));
      break;
  }
  detector_->Init();
  }
  converter_.reset(BaseCameraConverterRegisterer::GetInstanceByName(
      "GeometryCameraConverter"));
  converter_->Init();

  tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("CascadedCameraTracker"));
  tracker_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::GetInstanceByName(
      "FlatCameraTransformer"));
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();

  return true;
}

void CameraProcessSubnode::ImgCallback(const sensor_msgs::Image &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  AdapterManager::Observe();
  if ( !AdapterManager::GetLocalization()->Empty() &&
      !AdapterManager::GetChassis()->Empty() ) {
    const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
    const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();

    VehicleStateProvider::instance()->Update(localization, chassis);
  }

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ++seq_num_;
  ADEBUG << "CameraProcessSubnode Process:  frame: " << seq_num_;
  PERF_FUNCTION("CameraProcessSubnode");
  PERF_BLOCK_START();

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    MessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }
#if 0
  if ( FLAGS_enable_opencv_gpu < 1 ) {
    cv::resize(img, img, cv::Size(1920, 1080), 0, 0);
  } else {
#if (CV_MAJOR_VERSION == 2)
#ifdef HAVE_OPENCV_GPU
    cv::gpu::GpuMat matsrc, matdst;
    matsrc.upload(img);
    cv::gpu::resize(matsrc, matdst, cv::Size(1920, 1080));
    matdst.download(img);
#endif
#else
    cv::cuda::GpuMat matsrc, matdst;
    matsrc.upload(img);
    cv::cuda::resize(matsrc, matdst, cv::Size(1920, 1080));
    matdst.download(img);
#endif
  }
#endif
  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;

  PERF_BLOCK_END("CameraProcessSubnode_Image_Preprocess");
  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);

  cv::Mat mask_color(mask.rows, mask.cols, CV_32FC1);
  if (!FLAGS_enable_cnn_lane_detection) {
  if (FLAGS_use_whole_lane_line) {
    std::vector<cv::Mat> masks;
    detector_->Lanetask(img, &masks);
    mask_color.setTo(cv::Scalar(0));
    ln_msk_threshold_ = 0.9;
    for (int c = 0; c < num_lines; ++c) {
#if 0
      for (int h = 0; h < masks[c].rows; ++h) {
        for (int w = 0; w < masks[c].cols; ++w) {
          if (masks[c].at<float>(h, w) >= ln_msk_threshold_) {
            mask_color.at<float>(h, w) = static_cast<float>(c);
          }
      }
#else
      cv::threshold(masks[c],mask_color, ln_msk_threshold_, c, cv::THRESH_BINARY);
#endif
    }
  } else {
    mask.copyTo(mask_color);
    ln_msk_threshold_ = 0.5;
#if 0
    for (int h = 0; h < mask_color.rows; ++h) {
      for (int w = 0; w < mask_color.cols; ++w) {
        if (mask_color.at<float>(h, w) >= ln_msk_threshold_) {
          mask_color.at<float>(h, w) = static_cast<float>(5);
        }
      }
    }
#else
    cv::threshold(mask_color,mask_color, ln_msk_threshold_, 5, cv::THRESH_BINARY);
#endif
  }
  }
  PERF_BLOCK_END("CameraProcessSubnode_detector_");

  converter_->Convert(&objects);
  PERF_BLOCK_END("CameraProcessSubnode_converter_");

  if (FLAGS_use_navigation_mode) {
    transformer_->Transform(&objects);
    adjusted_extrinsics_ =
        transformer_->GetAdjustedExtrinsics(&camera_to_car_adj_);
    PERF_BLOCK_END("CameraProcessSubnode_transformer_");
  }

  tracker_->Associate(img, timestamp, &objects);
  PERF_BLOCK_END("CameraProcessSubnode_tracker_");

  FilterOptions options;

  if (FLAGS_use_navigation_mode) {
    options.camera_trans = std::make_shared<Eigen::Matrix4d>();
    options.camera_trans->setIdentity();
  } else {
    options.camera_trans = std::make_shared<Eigen::Matrix4d>();
    if (!GetCameraTrans(timestamp, options.camera_trans.get())) {
      AERROR << "failed to get trans at timestamp: " << timestamp;
      return;
    }
  }

  camera_to_world_ = *(options.camera_trans);
  // need to create camera options here for filter
  filter_->Filter(timestamp, &objects, options);
  PERF_BLOCK_END("CameraProcessSubnode_filter_");

  auto ccm = Singleton<CalibrationConfigManager>::get();
  auto calibrator = ccm->get_camera_calibration();
  calibrator->SetCar2CameraExtrinsicsAdj(camera_to_car_adj_,
                                         adjusted_extrinsics_);

  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  out_objs->timestamp = timestamp;
  VisualObjToSensorObj(objects, &out_objs, options);

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img;
  mask_color.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
  PERF_BLOCK_END("CameraProcessSubnode publish in DAG");

  if (pb_obj_) PublishPerceptionPbObj(out_objs);
  if (pb_ln_msk_) PublishPerceptionPbLnMsk(mask_color, message);
}

void CameraProcessSubnode::ChassisCallback(
    const apollo::canbus::Chassis &message) {
  chassis_.CopyFrom(message);
}

bool CameraProcessSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  *img = cv::Mat(msg.height, msg.width, CV_8UC3);
  int pixel_num = msg.width * msg.height;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    yuyv2bgr(yuv, img->data, pixel_num);
  } else {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *img = cv_ptr->image;
  }

  return true;
}

bool CameraProcessSubnode::MatToMessage(const cv::Mat &img,
                                        sensor_msgs::Image *msg) {
  if (img.type() == CV_8UC1) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8, img.rows,
                           img.cols, static_cast<unsigned int>(img.step),
                           img.data);
    return true;
  } else if (img.type() == CV_32FC1) {
    cv::Mat uc_img(img.rows, img.cols, CV_8UC1);
    uc_img.setTo(cv::Scalar(0));
    for (int h = 0; h < uc_img.rows; ++h) {
      for (int w = 0; w < uc_img.cols; ++w) {
        if (img.at<float>(h, w) >= ln_msk_threshold_) {
          uc_img.at<unsigned char>(h, w) = 1;
        }
      }
    }

    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           uc_img.rows, uc_img.cols,
                           static_cast<unsigned int>(uc_img.step), uc_img.data);
    return true;
  } else if (img.type() == CV_8UC3) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::BGR8, img.rows,
                           img.cols, static_cast<unsigned int>(img.step),
                           img.data);
    return true;
  } else {
    AERROR << "invalid input Mat type: " << img.type();
    return false;
  }
}

float CalIou(int upx1, int upy1, int upx2, int upy2,
            int lowx1, int lowy1, int lowx2, int lowy2) {
  float iou = 0.0;
  
  int area_x1 = std::max(upx1, lowx1);
  int area_y1 = std::max(upy1, lowy1);
  int area_x2 = std::min(upx2, lowx2);
  int area_y2 = std::min(upy2, lowy2);
  float area = std::max(0, (area_x2 - area_x1) * (area_y2 - area_y1));
  float box1 = (upx2 - upx1) * (upy2 - upy1);
  float box2 = (lowx2 - lowx1) * (lowy2 - lowy1);
  float unium = box1 + box2 - area;
  iou = unium > 0 ? (area / unium) : 0;
  return iou;
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects, FilterOptions options) {
  (*sensor_objects)->sensor_type = SensorType::CAMERA;
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;
  if (FLAGS_use_navigation_mode) {
    (*sensor_objects)->sensor2world_pose_static = camera_to_car_;
    (*sensor_objects)->sensor2world_pose = camera_to_car_adj_;
  } else {
    (*sensor_objects)->sensor2world_pose_static = *(options.camera_trans);
    (*sensor_objects)->sensor2world_pose = *(options.camera_trans);
    AINFO << "camera process sensor2world pose is "
          << (*sensor_objects)->sensor2world_pose;
  }
  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  if (!CameraFrameSupplement::state_vars.initialized_) {
    CameraFrameSupplement::state_vars.process_noise *= 10;
    CameraFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
        0.0f, 0.33f, 0.0f;
    CameraFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
        1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in CameraFrameSupplement is \n"
           << CameraFrameSupplement::state_vars.trans_matrix << std::endl;
    CameraFrameSupplement::state_vars.initialized_ = true;
  }

  //save 5 frame objects
  if (FLAGS_save_framenum) {
    if (saveobjects_.size() > 0) {
      // del more 5 frame
      std::vector<std::shared_ptr<VisualObject>>::iterator itor;
      for (itor = saveobjects_.begin(); itor != saveobjects_.end();) {
        if (seq_num_ - (*itor)->frameid > (unsigned int)FLAGS_save_framenum) { //del greater 5 frame
          itor = saveobjects_.erase(itor);
        } else {
          ++itor;
        } 
      }

      for (auto sobj : objects) {
        if (std::isnan(sobj->center[0]) || std::isnan(sobj->center[1]) || std::isnan(sobj->center[2])) {
          continue;
        }
        int flag = 0;
        for (itor = saveobjects_.begin(); itor != saveobjects_.end();) {
          if (CalIou((int)sobj->upper_left[0], (int)sobj->upper_left[1], (int)sobj->lower_right[0], (int)sobj->lower_right[1],
            (int)(*itor)->upper_left[0], (int)(*itor)->upper_left[1], (int)(*itor)->lower_right[0], (int)(*itor)->lower_right[1]) >= 0.8) {
            ObjectCopy(sobj, *itor, 0);
            flag = 1;
            break;
          }
          ++itor;
        }

        if (flag == 0) {
          std::shared_ptr<VisualObject> obj(new VisualObject);
          ObjectCopy(sobj, obj, 1);
          saveobjects_.emplace_back(obj);
        }
      }
    } else {
      for (auto sobj : objects) {
        if (std::isnan(sobj->center[0]) || std::isnan(sobj->center[1]) || std::isnan(sobj->center[2])) {
          continue;
        }
        std::shared_ptr<VisualObject> obj(new VisualObject);
        ObjectCopy(sobj, obj, 1);
        saveobjects_.emplace_back(obj);
      }
    }
    
    for (auto vobj : saveobjects_) {
      std::unique_ptr<Object> obj(new Object());

      obj->id = vobj->id;
      obj->score = vobj->score;
      obj->direction = vobj->direction.cast<double>();
      obj->theta = vobj->theta;
      obj->center = vobj->center.cast<double>();
      obj->length = vobj->length;
      obj->width = vobj->width;
      obj->height = vobj->height;
      obj->type = vobj->type;
      obj->track_id = vobj->track_id;
      obj->tracking_time = vobj->track_age;
      obj->latest_tracked_time = vobj->last_track_timestamp;
      obj->velocity = vobj->velocity.cast<double>();
      obj->anchor_point = obj->center;
      obj->state_uncertainty = vobj->state_uncertainty;

      (obj->camera_supplement).reset(new CameraSupplement());
      obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
      obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
      obj->camera_supplement->alpha = vobj->alpha;
      obj->camera_supplement->pts8 = vobj->pts8;
      ((*sensor_objects)->objects).emplace_back(obj.release());
    }
  } else {
    for (auto vobj : objects) {
      if (std::isnan(vobj->center[0]) || std::isnan(vobj->center[1]) || std::isnan(vobj->center[2])) {
        continue;
      }
      std::unique_ptr<Object> obj(new Object());

      obj->id = vobj->id;
      obj->score = vobj->score;
      obj->direction = vobj->direction.cast<double>();
      obj->theta = vobj->theta;
      obj->center = vobj->center.cast<double>();
      obj->length = vobj->length;
      obj->width = vobj->width;
      obj->height = vobj->height;
      obj->type = vobj->type;
      obj->track_id = vobj->track_id;
      obj->tracking_time = vobj->track_age;
      obj->latest_tracked_time = vobj->last_track_timestamp;
      obj->velocity = vobj->velocity.cast<double>();
      obj->anchor_point = obj->center;
      obj->state_uncertainty = vobj->state_uncertainty;

      (obj->camera_supplement).reset(new CameraSupplement());
      obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
      obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
      obj->camera_supplement->alpha = vobj->alpha;
      obj->camera_supplement->pts8 = vobj->pts8;
      ((*sensor_objects)->objects).emplace_back(obj.release());
    }
  }
}

void CameraProcessSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
  const CommonSharedDataKey key(timestamp, device_id_);
  cam_obj_data_->Add(key, sensor_objects);
  cam_shared_data_->Add(key, camera_item);

  for (const EventMeta& event_meta : pub_meta_events_) {
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

void CameraProcessSubnode::PublishPerceptionPbObj(
    const SharedDataPtr<SensorObjects> &sensor_objects) {
  PerceptionObstacles obstacles;

  // Header
  AdapterManager::FillPerceptionObstaclesHeader("perception_obstacle",
                                                &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(timestamp_ns_);
  header->set_radar_timestamp(0);
  obstacles.set_error_code(sensor_objects->error_code);

  // Serialize each Object
  for (const auto &obj : sensor_objects->objects) {
    PerceptionObstacle *obstacle = obstacles.add_perception_obstacle();
    obj->Serialize(obstacle);
  }

  // Relative speed of objects + latest ego car speed in X
  for (auto obstacle : obstacles.perception_obstacle()) {
    obstacle.mutable_velocity()->set_x(
        obstacle.velocity().x() + chassis_.speed_mps());
  }

  AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "PublishPerceptionObstacles: " << obstacles.ShortDebugString();
}

void CameraProcessSubnode::PublishPerceptionPbLnMsk(
    const cv::Mat &mask, const sensor_msgs::Image &message) {
  sensor_msgs::Image lane_mask_msg;
  lane_mask_msg.header = message.header;
  lane_mask_msg.header.frame_id = "lane_mask";
  MatToMessage(mask, &lane_mask_msg);

  AdapterManager::PublishPerceptionLaneMask(lane_mask_msg);
  ADEBUG << "PublishPerceptionLaneMask";
}

void CameraProcessSubnode::ObjectCopy(std::shared_ptr<VisualObject> from, std::shared_ptr<VisualObject> &to, int isnew) {
  Eigen::Vector3d vehicle_location(VehicleStateProvider::instance()->x(),
                                   VehicleStateProvider::instance()->y(),
                                   VehicleStateProvider::instance()->z());
  double vehicle_heading = VehicleStateProvider::instance()->heading();

  to->id = from->id;
  to->score = from->score;
  //:to->direction = from->direction;
  to->theta = from->theta;
  //to->center = from->center;
  to->length = from->length;
  to->width = from->width;
  to->height = from->height;
  to->type = from->type;
  to->track_id = from->track_id;
  to->track_age = from->track_age;
  to->velocity = from->velocity;
  to->upper_left = from->upper_left;
  to->lower_right = from->lower_right;
  to->alpha = from->alpha;
  to->pts8 = from->pts8;
  to->farflag = from->farflag;
  if (isnew) {
    to->direction = from->direction;
    to->center = from->center;
    to->frameid = seq_num_;
    to->vehicle_location = from->vehicle_location;
    to->vehicle_heading = from->vehicle_heading;
  } else {
    to->frameid = from->frameid;
    to->center -= (vehicle_location - from->vehicle_location).cast<float>();
    to->direction -= Eigen::Vector3f(cos(vehicle_heading - from->vehicle_heading),
                                     0.0f, -sin(vehicle_heading - from->vehicle_heading));
  }
}

}  // namespace perception
}  // namespace apollo
