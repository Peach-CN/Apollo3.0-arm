/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/gnss/parser/data_parser.h"

#include <cmath>
#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "boost/array.hpp"
#include "ros/include/ros/ros.h"
#include "std_msgs/String.h"
#include "modules/common/math/math_utils.h"

#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/zhd_gps.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/gnss/gnss_gflags.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::drivers::gnss::InsStat;
using ::apollo::drivers::gnss::GnssBestPose;
using ::apollo::drivers::gnss::GnssEphemeris;
using ::apollo::drivers::gnss::EpochObservation;
using ::apollo::drivers::gnss::Heading;
using ::apollo::drivers::gnss::Imu;
using ::apollo::localization::CorrectedImu;
using ::apollo::drivers::gnss::Ins;
using ::apollo::localization::Gps;
using ::apollo::common::adapter::AdapterManager;
using ::apollo::drivers::gnss::ZhdGps;

namespace {

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";

// covariance data for pose if can not get from novatel inscov topic
static const boost::array<double, 36> POSE_COVAR = {
    2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

template <class T>
void PublishMessageRaw(const ros::Publisher &pub, const T *pb) {
  std_msgs::String msg_pub;

  if (pb->SerializeToString(&msg_pub.data)) {
    pub.publish(msg_pub);
    return;
  }
  AERROR << "Failed to serialize message.";
}

Parser *CreateParser(config::Config config, bool is_base_station = false) {
  switch (config.data().format()) {
    case config::Stream::NOVATEL_BINARY:
      return Parser::CreateNovatel(config);

    case config::Stream::MARVELMIND:
      return Parser::CreateMarvelmind(config);

    case config::Stream::NOVATEL_TEXT:
      return Parser::CreateNovatel(config);

    case config::Stream::NEWTONM2_BINARY:
      return Parser::CreateNewtonM2(config);

    case config::Stream::ZHD_BINARY:
      return Parser::createZhd(config);

    default:
      return nullptr;
  }
}

}  // namespace

DataParser::DataParser(const config::Config &config) : config_(config) {
  std::string utm_target_param;

  wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
  utm_target_ = pj_init_plus(config_.proj4_text().c_str());
  gnss_status_.set_solution_status(0);
  gnss_status_.set_num_sats(0);
  gnss_status_.set_position_type(0);
  gnss_status_.set_solution_completed(false);
  ins_status_.set_type(InsStatus::INVALID);
}

bool DataParser::Init() {
  ins_status_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  gnss_status_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

  AdapterManager::FillInsStatusHeader(FLAGS_sensor_node_name, &ins_status_);
  AdapterManager::PublishInsStatus(ins_status_);
  AdapterManager::FillGnssStatusHeader(FLAGS_sensor_node_name, &gnss_status_);
  AdapterManager::PublishGnssStatus(gnss_status_);

  AINFO << "Creating data parser of format: " << config_.data().format();
  data_parser_.reset(CreateParser(config_, false));
  if (!data_parser_) {
    AFATAL << "Failed to create data parser.";
    return false;
  }

  inited_flag_ = true;
  return true;
}

void DataParser::ParseRawData(const std_msgs::String::ConstPtr &msg) {
  if (!inited_flag_) {
    AERROR << "Data parser not init.";
    return;
  }

  data_parser_->Update(msg->data);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  while (ros::ok()) {
    type = data_parser_->GetMessage(&msg_ptr);
    if (type == Parser::MessageType::NONE) break;
    DispatchMessage(type, msg_ptr);
  }
}

void DataParser::CheckInsStatus(::apollo::drivers::gnss::Ins *ins) {
  if (1/*ins_status_record_ != static_cast<uint32_t>(ins->type())*/) {
    ins_status_record_ = static_cast<uint32_t>(ins->type());
    switch (ins->type()) {
      case apollo::drivers::gnss::Ins::GOOD:
        ins_status_.set_type(apollo::drivers::gnss_status::InsStatus::GOOD);
        break;

      case apollo::drivers::gnss::Ins::CONVERGING:
        ins_status_.set_type(
            apollo::drivers::gnss_status::InsStatus::CONVERGING);
        break;

      case apollo::drivers::gnss::Ins::INVALID:
      default:
        ins_status_.set_type(apollo::drivers::gnss_status::InsStatus::INVALID);
        break;
    }
    AdapterManager::FillInsStatusHeader(FLAGS_sensor_node_name, &ins_status_);
    AdapterManager::PublishInsStatus(ins_status_);
  }
}

void DataParser::CheckZhdInsStatus(::apollo::drivers::gnss::ZhdGps *zhd_ins) {
  if (1/*ins_status_record_ != static_cast<uint32_t>(zhd_ins->instype())*/) {
    ins_status_record_ = static_cast<uint32_t>(zhd_ins->instype());
    switch (zhd_ins->instype()) {
      case apollo::drivers::gnss::ZhdGps::GOOD:
        ins_status_.set_type(apollo::drivers::gnss_status::InsStatus::GOOD);
        break;

      case apollo::drivers::gnss::ZhdGps::CONVERGING:
        ins_status_.set_type(
            apollo::drivers::gnss_status::InsStatus::CONVERGING);
        break;

      case apollo::drivers::gnss::ZhdGps::INVALID:
      default:
        ins_status_.set_type(apollo::drivers::gnss_status::InsStatus::INVALID);
        break;
    }
    AdapterManager::FillInsStatusHeader(FLAGS_sensor_node_name, &ins_status_);
    AdapterManager::PublishInsStatus(ins_status_);
  }
}

void DataParser::CheckGnssStatus(::apollo::drivers::gnss::Gnss *gnss) {
  gnss_status_.set_solution_status(
      static_cast<uint32_t>(gnss->solution_status()));
  gnss_status_.set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
  gnss_status_.set_position_type(static_cast<uint32_t>(gnss->position_type()));

  if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
    gnss_status_.set_solution_completed(true);
  } else {
    gnss_status_.set_solution_completed(false);
  }
  AdapterManager::FillGnssStatusHeader(FLAGS_sensor_node_name, &gnss_status_);
  AdapterManager::PublishGnssStatus(gnss_status_);
}

void DataParser::DispatchMessage(Parser::MessageType type, MessagePtr message) {
  std_msgs::String msg_pub;

  switch (type) {
    case Parser::MessageType::GNSS:
      CheckGnssStatus(As<::apollo::drivers::gnss::Gnss>(message));
      break;

    case Parser::MessageType::BEST_GNSS_POS:
      PublishBestpos(message);
      break;

    case Parser::MessageType::IMU:
      PublishImu(message);
      break;

    case Parser::MessageType::INS:
      CheckInsStatus(As<::apollo::drivers::gnss::Ins>(message));
      PublishCorrimu(message);
      PublishOdometry(message);
      break;

    case Parser::MessageType::ZHD_GPS:
      CheckZhdInsStatus(As<::apollo::drivers::gnss::ZhdGps>(message));
      PublishZhdOdometry(message);
      break;

    case Parser::MessageType::INS_STAT:
      PublishInsStat(message);
      break;

    case Parser::MessageType::BDSEPHEMERIDES:
    case Parser::MessageType::GPSEPHEMERIDES:
    case Parser::MessageType::GLOEPHEMERIDES:
      PublishEphemeris(message);
      break;

    case Parser::MessageType::OBSERVATION:
      PublishObservation(message);
      break;

    case Parser::MessageType::HEADING:
      PublishHeading(message);
      break;

    case Parser::MessageType::GPCHC:
      break;

    default:
      break;
  }
}

void DataParser::PublishInsStat(const MessagePtr message) {
  InsStat ins_stat = InsStat(*As<InsStat>(message));
  AdapterManager::FillInsStatHeader(FLAGS_sensor_node_name, &ins_stat);
  AdapterManager::PublishInsStat(ins_stat);
}

void DataParser::PublishBestpos(const MessagePtr message) {
  GnssBestPose bestpos = GnssBestPose(*As<GnssBestPose>(message));
  AdapterManager::FillGnssBestPoseHeader(FLAGS_sensor_node_name, &bestpos);
  AdapterManager::PublishGnssBestPose(bestpos);
}

void DataParser::PublishImu(const MessagePtr message) {
  Imu raw_imu = Imu(*As<Imu>(message));
  Imu *imu = As<Imu>(message);
  if (config_.data().format() != config::Stream::ZHD_BINARY) {
  raw_imu.mutable_linear_acceleration()->set_x(-imu->linear_acceleration().y());
  raw_imu.mutable_linear_acceleration()->set_y(imu->linear_acceleration().x());
  raw_imu.mutable_linear_acceleration()->set_z(imu->linear_acceleration().z());

  raw_imu.mutable_angular_velocity()->set_x(-imu->angular_velocity().y());
  raw_imu.mutable_angular_velocity()->set_y(imu->angular_velocity().x());
  raw_imu.mutable_angular_velocity()->set_z(imu->angular_velocity().z());
  } else {
  if (fabs(imu->angular_velocity().y()) > 1.57 || fabs(imu->angular_velocity().x()) > 1.57)
    return ;
  raw_imu.mutable_linear_acceleration()->set_y(imu->linear_acceleration().y());
  raw_imu.mutable_linear_acceleration()->set_x(imu->linear_acceleration().x());
  raw_imu.mutable_linear_acceleration()->set_z(imu->linear_acceleration().z());

  raw_imu.mutable_angular_velocity()->set_y(imu->angular_velocity().y());
  raw_imu.mutable_angular_velocity()->set_x(imu->angular_velocity().x());
  raw_imu.mutable_angular_velocity()->set_z(imu->angular_velocity().z());

  }
  AdapterManager::FillRawImuHeader(FLAGS_sensor_node_name, &raw_imu);
  AdapterManager::PublishRawImu(raw_imu);
}

void DataParser::PublishOdometry(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  Gps gps;
  static double cx = 0.0, cy = 0.0;
  double unix_sec = common::time::TimeUtil::Gps2unix(ins->measurement_time());
  gps.mutable_header()->set_timestamp_sec(unix_sec);
  auto *gps_msg = gps.mutable_localization();

  // 1. pose xyz
  double x = ins->position().lon();
  double y = ins->position().lat();
  
  if(!FLAGS_indoor_localization) {
  	x *= DEG_TO_RAD_LOCAL;
  	y *= DEG_TO_RAD_LOCAL;
  	pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);
  }
  if (fabs(x - cx) > 100. || fabs(y -cy) > 100.) {
    cx = x; cy = y;
    return ;
  }
  cx = x; cy = y;
  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(ins->position().height());

  // 2. orientation
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z()/* - 90 * DEG_TO_RAD_LOCAL*/,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
  gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
  gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());

  AdapterManager::FillGpsHeader("Gps", &gps);
  AdapterManager::PublishGps(gps);

  geometry_msgs::TransformStamped transform;
  GpsToTransformStamped(gps, &transform);
  tf_broadcaster_.sendTransform(transform);
}

void DataParser::PublishZhdOdometry(const MessagePtr message) {
  ZhdGps *zhdgps = As<ZhdGps>(message);
  Gps gps;

  // double unix_sec = common::time::TimeUtil::Gps2unix(ins->measurement_time());
  gps.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

  auto *gps_msg = gps.mutable_localization();

  // 1. pose xyz
  double x = zhdgps->longitude();
  double y = zhdgps->latitude();
  x *= DEG_TO_RAD_LOCAL;
  y *= DEG_TO_RAD_LOCAL;

  pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  if ( config_.has_gnss_offset() ) {
    double flu_x = config_.gnss_offset().x();
    double flu_y = config_.gnss_offset().y();
    double enu_x = 0.0;
    double enu_y = 0.0;
    common::math::RotateAxis(-(zhdgps->angle_heading() *
    DEG_TO_RAD_LOCAL), flu_x, flu_y, &enu_x, &enu_y);
    x += enu_x;
    y += enu_y;
  }

  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(zhdgps->altitude());

  // 2. orientation  M_PI / 180.0;
  // zhdgps_.set_vel_u_m_s //弧度转换 使用

  Eigen::Quaterniond q =
      Eigen::AngleAxisd(zhdgps->vel_u_m_s(),
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  // Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
  // Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  gps_msg->mutable_linear_velocity()->set_x(zhdgps->vel_e_m_s());
  gps_msg->mutable_linear_velocity()->set_y(zhdgps->vel_n_m_s());
  gps_msg->mutable_linear_velocity()->set_z(0);

  // gps_msg->mutable_linear_velocity()->set_z(zhdgps->vel_u_m_s());


  // gps.mutable_zhdgps()->CopyFrom(ZhdGps(*As<ZhdGps>(zhdgps)));

  if ( config_.has_zhd_postype_enable() && config_.zhd_postype_enable() == true) {
      if ((apollo::drivers::gnss::ZhdGps::FIX_TYPE_RT2 != zhdgps->fix_type())
      || (apollo::drivers::gnss::ZhdGps::POS_TYPE_NARROW_INT
            != zhdgps->angle_postype()) ) {
        AERROR << "zhd error_cur angle_type=" << zhdgps->angle_postype();
      } else {
        AdapterManager::FillGpsHeader("Gps", &gps);
        AdapterManager::PublishGps(gps);
      }
  } else {
      if (apollo::drivers::gnss::ZhdGps::FIX_TYPE_INVALID != zhdgps->fix_type()) {
        AdapterManager::FillGpsHeader("Gps", &gps);
        AdapterManager::PublishGps(gps);
      } else {
         AERROR << "=zhd gps INVALID=";
      }
  }
  
  geometry_msgs::TransformStamped transform;
  GpsToTransformStamped(gps, &transform);
  tf_broadcaster_.sendTransform(transform);
}

void DataParser::PublishCorrimu(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  CorrectedImu imu;
  double unix_sec = common::time::TimeUtil::Gps2unix(ins->measurement_time());
  imu.mutable_header()->set_timestamp_sec(unix_sec);

  auto *imu_msg = imu.mutable_imu();
  if (config_.data().format() != config::Stream::ZHD_BINARY) {
  imu_msg->mutable_linear_acceleration()->set_x(
      -ins->linear_acceleration().y());
  imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().x());
  imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());

  imu_msg->mutable_angular_velocity()->set_x(-ins->angular_velocity().y());
  imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().x());
  imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());
  } else  {
    imu_msg->mutable_linear_acceleration()->set_x(
      ins->linear_acceleration().x());
  imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().y());
  imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());

  imu_msg->mutable_angular_velocity()->set_x(ins->angular_velocity().x());
  imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().y());
  imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());
  }
  imu_msg->mutable_euler_angles()->set_x(ins->euler_angles().x());
  imu_msg->mutable_euler_angles()->set_y(-ins->euler_angles().y());
  imu_msg->mutable_euler_angles()->set_z(ins->euler_angles().z() -
                                         90 * DEG_TO_RAD_LOCAL);

  AdapterManager::PublishImu(imu);
}

void DataParser::PublishEphemeris(const MessagePtr message) {
  GnssEphemeris eph = GnssEphemeris(*As<GnssEphemeris>(message));
  AdapterManager::PublishGnssRtkEph(eph);
}

void DataParser::PublishObservation(const MessagePtr message) {
  EpochObservation observation =
      EpochObservation(*As<EpochObservation>(message));
  AdapterManager::PublishGnssRtkObs(observation);
}

void DataParser::PublishHeading(const MessagePtr message) {
  Heading heading = Heading(*As<Heading>(message));
  AdapterManager::FillGnssHeadingHeader(FLAGS_sensor_node_name, &heading);
  AdapterManager::PublishGnssHeading(heading);
}

void DataParser::GpsToTransformStamped(
    const ::apollo::localization::Gps &gps,
    geometry_msgs::TransformStamped *transform) {
  ros::Time time;
  transform->header.stamp = time.fromSec(gps.header().timestamp_sec());
  transform->header.frame_id = config_.tf().frame_id();
  transform->child_frame_id = config_.tf().child_frame_id();
  transform->transform.translation.x = gps.localization().position().x();
  transform->transform.translation.y = gps.localization().position().y();
  transform->transform.translation.z = gps.localization().position().z();
  transform->transform.rotation.x = gps.localization().orientation().qx();
  transform->transform.rotation.y = gps.localization().orientation().qy();
  transform->transform.rotation.z = gps.localization().orientation().qz();
  transform->transform.rotation.w = gps.localization().orientation().qw();
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
