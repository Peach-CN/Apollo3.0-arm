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
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <iomanip>

#include <limits>
#include <iostream>
#include <cmath>
#include <vector>
#include <atomic>

#include <eigen3/Eigen/Geometry> 

#include "ros/include/ros/ros.h"
#include "ros/include/sensor_msgs/Imu.h"
#include "ros/include/sensor_msgs/MagneticField.h"

#include "modules/common/log.h"
#include "modules/drivers/gnss/parser/zhd_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"
#include "modules/drivers/gnss/proto/zhd_gps.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
// Anonymous namespace that contains helper constants and functions.
namespace {
    constexpr size_t BUFFER_SIZE = 256;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

    constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();
    constexpr double azimuth_deg_to_yaw_rad(double azimuth, double angle) {
      return (angle - azimuth) * DEG_TO_RAD;
    }
}  // namespace

typedef std::function<bool(uint8_t *,int32_t)> RawFrameHandler;

class PackageProcessor {
public:
  PackageProcessor() = default;
  virtual ~PackageProcessor() {}
  
  virtual int UpdateData(const uint8_t* data, size_t length);
  virtual int UnpackData() = 0;

protected:
  uint8_t  data_raw_[16384];
  uint32_t nbuf_length_ = 0;
  uint8_t  *pbuf_ = &data_raw_[0];
};

int PackageProcessor::UpdateData(const uint8_t* data, size_t length) {
  if (length > 0) {
    memcpy(&pbuf_[nbuf_length_], data, length);
    nbuf_length_ += length;
  }

  return 0;
}

class RtkPackageProcessor : public PackageProcessor {
public:
  RtkPackageProcessor(RawFrameHandler handler) : handler_(handler) {}

  virtual int UnpackData();

private:
  RawFrameHandler handler_;
};

#if 0
// GINS(binary)
const PackageProcessor::FIELD RtkPackageProcessor::Fields[] = {
  { 0,  3,  header,          (void *)"\xAC\xCA\x0C",  0       , LITTLEENDIAN},
  { 3, 44,  data,            nullptr       ,  0       , LITTLEENDIAN},
  {47,  1,  byte_checksum,   (void *)3/*(3 << 16) | 44*/,  0       , LITTLEENDIAN},
  {48,  1,  tail,            (void *)"\xBF"        ,  0       , LITTLEENDIAN},
};
//#else
// GINS(ASCII)
const PackageProcessor::FIELD RtkPackageProcessor::Fields[] = {
  { 0,  5,  header,     "$GINS"       ,  0       , LITTLEENDIAN},
  { 5,  0,  data,       nullptr       ,  0       , LITTLEENDIAN},
  {-2,  2,  tail,       "\r\n"        ,  0       , LITTLEENDIAN},
};
#endif

int RtkPackageProcessor::UnpackData() {
  while (nbuf_length_ >= sizeof(zhd::gps_rtk_zhd_packet_t)) {
    zhd::gps_rtk_zhd_packet_t *rtk_ptr = reinterpret_cast<zhd::gps_rtk_zhd_packet_t *>(pbuf_);
    if (rtk_ptr->head[0] == 0xAC && rtk_ptr->head[1] == 0xCA \
        && rtk_ptr->head[2] == 0x0C && 0xBF == rtk_ptr->tail) {
      uint8_t checksum = 0;
      for(int i = 3; i < sizeof(zhd::gps_rtk_zhd_packet_t) - 2; ++i)
        checksum ^= pbuf_[i];

      if (rtk_ptr->checksum != checksum) {
         --nbuf_length_; ++pbuf_;
         continue;
      }

      handler_(pbuf_, static_cast<int32_t>(sizeof(zhd::gps_rtk_zhd_packet_t)));
      nbuf_length_ -= sizeof(zhd::gps_rtk_zhd_packet_t);
      pbuf_ += sizeof(zhd::gps_rtk_zhd_packet_t);
      return 0;
    } else {
      --nbuf_length_; ++pbuf_;
    }
  }

  if (nbuf_length_ > 0 && pbuf_ != data_raw_) {
    memmove(&data_raw_[0], pbuf_, nbuf_length_);
  }
  pbuf_ = &data_raw_[0];

  return 1;
}

class ImuPackageProcessor : public PackageProcessor {
public:
  ImuPackageProcessor(RawFrameHandler handler) : handler_(handler) {}
  
  virtual int UnpackData();

private:
  RawFrameHandler handler_;
};

#if 0
// IMU(binary)
const PackageProcessor::FIELD ImuPackageProcessor::Fields[] = {
  { 0,  3,  header,          (void *)"\x7E\x7E\x0C",  0       , LITTLEENDIAN},
  { 3, 30,  data,            nullptr       ,  0       , LITTLEENDIAN},
  {33,  1,  byte_checksum,   (void *)3/*(3 << 16) | 30*/,    0, LITTLEENDIAN},
  {34,  1,  tail,            (void *)"\xBF"        ,  0       , LITTLEENDIAN},
};
//#else
// IMU(ASCII)
const PackageProcessor::FIELD ImuPackageProcessor::Fields[] = {
  { 0,  6,  header,     "$GRIMU",        0       , LITTLEENDIAN},
  { 6,  0,  data,       nullptr       ,  0       , LITTLEENDIAN},
  {-2,  2,  tail,       "\r\n"        ,  0       , LITTLEENDIAN},
};
#endif

int ImuPackageProcessor::UnpackData() {
  while (nbuf_length_ >= sizeof(zhd::imu_zhd_packet_t)) {
    zhd::imu_zhd_packet_t *imu_ptr = reinterpret_cast<zhd::imu_zhd_packet_t *>(pbuf_);
    if (imu_ptr->head[0] == 0x7E && imu_ptr->head[1] == 0x7E \
        && imu_ptr->head[2] == 0x0C && 0xBF == imu_ptr->tail) {
      uint8_t checksum = 0;
      for(int i = 3; i < sizeof(zhd::imu_zhd_packet_t) - 2; ++i)
        checksum ^= pbuf_[i];

      if (imu_ptr->checksum != checksum) {
         --nbuf_length_; ++pbuf_;
         continue;
      }

      handler_(pbuf_, static_cast<int32_t>(sizeof(zhd::imu_zhd_packet_t)));
      nbuf_length_ -= sizeof(zhd::imu_zhd_packet_t);
      pbuf_ += sizeof(zhd::imu_zhd_packet_t);
      return 0;
    } else {
      --nbuf_length_; ++pbuf_;
    }
  }

  if (nbuf_length_ > 0 && pbuf_ != data_raw_) {
    memmove(&data_raw_[0], pbuf_, nbuf_length_);
  }
  pbuf_ = &data_raw_[0];

  return 1;
}

class StaPackageProcessor : public PackageProcessor {
public:
  StaPackageProcessor(RawFrameHandler handler) : handler_(handler) {}

  virtual int UnpackData();

private:
  RawFrameHandler handler_;
};

// STA(binary)
#if 0
const PackageProcessor::FIELD StaPackageProcessor::Fields[] = {
  { 0,  3,  header,         (void *)"\xBD\xDB\x0B",  0       , LITTLEENDIAN},
  { 3,  7,  data,           nullptr       ,  0       , LITTLEENDIAN},
  {10,  1,  byte_checksum,  (void *)3/*(3 << 16) |  7*/,  0       , LITTLEENDIAN},
  {11,  1,  tail,           (void *)"\xBF"        ,  0       , LITTLEENDIAN},
};
#endif

int StaPackageProcessor::UnpackData()
{
  while (nbuf_length_ >= sizeof(zhd::status_zhd_packet_t)) {
    zhd::status_zhd_packet_t *sta_ptr = reinterpret_cast<zhd::status_zhd_packet_t *>(pbuf_);
    if (sta_ptr->head[0] == 0xBD && sta_ptr->head[1] == 0xDB \
        && sta_ptr->head[2] == 0x0B && 0xBF == sta_ptr->tail) {
      uint8_t checksum = 0;
      for(int i = 3; i < sizeof(zhd::status_zhd_packet_t) - 2; ++i)
        checksum ^= pbuf_[i];

      if (sta_ptr->checksum != checksum) {
         --nbuf_length_; ++pbuf_;
         continue;
      }

      handler_(pbuf_, static_cast<int32_t>(sizeof(zhd::status_zhd_packet_t)));
      nbuf_length_ -= sizeof(zhd::status_zhd_packet_t);
      pbuf_ += sizeof(zhd::status_zhd_packet_t);
      return 0;
    } else {
      --nbuf_length_; ++pbuf_;
    }
  }

  if (nbuf_length_ > 0 && pbuf_ != data_raw_) {
    memmove(&data_raw_[0], pbuf_, nbuf_length_);
  }
  pbuf_ = &data_raw_[0];

  return 1;
}

class ZhdParser : public Parser {
 public:
    ZhdParser();
    explicit ZhdParser(const config::Config& config);
    virtual MessageType GetMessage(MessagePtr* message_ptr);
    void RegisterPackages();
    virtual void Update(const uint8_t *data, size_t length);

 private:
    Parser::MessageType PrepareMessage(MessagePtr* message_ptr);
    // -1 is an unused value.

    bool HandleInsPva(uint8_t *pbuf, int32_t length);
    bool HandleRawImu(uint8_t *pbuf, int32_t length);
    bool HandleStaData(uint8_t *pbuf, int32_t length);

    double imu_measurement_time_previous_ = 0.0;

    ::apollo::drivers::gnss::Ins  ins_;
    ::apollo::drivers::gnss::Imu  imu_;
    ::apollo::drivers::gnss::ZhdGps zhdgps_;
    ::apollo::drivers::gnss::GnssBestPose bestpos_;
    
    config::ZhdConfig zhd_config_;
    
    std::atomic<bool> switch_flag_{false};
    
    std::shared_ptr<PackageProcessor> rtk_parser_;
    std::shared_ptr<PackageProcessor> imu_parser_;
    std::shared_ptr<PackageProcessor> sta_parser_;
    
    sensor_msgs::Imu msg_imu_;
    sensor_msgs::MagneticField msg_mag_;
    ros::Publisher imu_pub_, mag_pub_;
};

Parser* Parser::createZhd(const config::Config& config) {
  return new ZhdParser(config);
}

ZhdParser::ZhdParser() {
  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  RegisterPackages();
}

ZhdParser::ZhdParser(const config::Config& config) {
  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  if (config.has_zhd_config()) {
    zhd_config_ = config.zhd_config();
  } else {
    zhd_config_.set_angle_heading(90);
    AERROR << "angle_heading=" << std::hex << zhd_config_.angle_heading();
  }
  
  ros::NodeHandle n("~");
  imu_pub_ = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
  mag_pub_ = n.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);

  RegisterPackages();
}

void ZhdParser::RegisterPackages() {
  rtk_parser_ = std::make_shared<RtkPackageProcessor>(
    std::bind(&ZhdParser::HandleInsPva, this, std::placeholders::_1, std::placeholders::_2));
  imu_parser_ = std::make_shared<ImuPackageProcessor>(
    std::bind(&ZhdParser::HandleRawImu, this, std::placeholders::_1, std::placeholders::_2));
  sta_parser_ = std::make_shared<StaPackageProcessor>(
    std::bind(&ZhdParser::HandleStaData, this, std::placeholders::_1, std::placeholders::_2));
}

void ZhdParser::Update(const uint8_t *data, size_t length) {
  rtk_parser_->UpdateData(data, length);
  imu_parser_->UpdateData(data, length);
  sta_parser_->UpdateData(data, length);
}

Parser::MessageType ZhdParser::GetMessage(MessagePtr* message_ptr) {
  if (!(switch_flag_.load())) {
    if (!rtk_parser_->UnpackData()) {
      *message_ptr = &ins_;
      return MessageType::INS;
    }
    if (!imu_parser_->UnpackData()) {
      *message_ptr = &imu_;
      return MessageType::IMU;
    }
    switch_flag_ = true;
  } else {
    if (!imu_parser_->UnpackData()) {
      *message_ptr = &imu_;
      return MessageType::IMU;
    }
    if (!rtk_parser_->UnpackData()) {
      *message_ptr = &ins_;
      return MessageType::INS;
    }
    switch_flag_ = false;
  }
  if (!sta_parser_->UnpackData()) {
    *message_ptr = &bestpos_;
    return MessageType::BEST_GNSS_POS;
  }

  return MessageType::NONE;
}

Parser::MessageType ZhdParser::PrepareMessage(MessagePtr* message_ptr) {
  return MessageType::ZHD_GPS;
}

bool ZhdParser::HandleInsPva(uint8_t *pbuf, int32_t length) {
  const zhd::gps_rtk_zhd_packet_t* pva = reinterpret_cast<zhd::gps_rtk_zhd_packet_t*>(pbuf);

  ins_.mutable_position()->set_lon(pva->longitude * 1e-7);
  ins_.mutable_position()->set_lat(pva->latitude * 1e-7);
  ins_.mutable_position()->set_height(pva->altitude * 1e-3);

  ins_.mutable_euler_angles()->set_x(pva->angle_roll * 360 / 32768. * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(pva->angle_pitch * 360 / 32768. * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_z(
      azimuth_deg_to_yaw_rad(pva->angle_azimuth * 360 / 32768. - 90, 0));

#if 0
  Eigen::Vector3d ea0(pva->angle_azimuth * 360 / 32768. * DEG_TO_RAD, 
                      pva->angle_roll * 360 / 32768. * DEG_TO_RAD, 
                      pva->angle_pitch * 360 / 32768. * DEG_TO_RAD);
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q;
  q = R;
#else
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(azimuth_deg_to_yaw_rad(pva->angle_azimuth * 360 / 32768. - 90, 0),
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-pva->angle_pitch * 360 / 32768. * DEG_TO_RAD, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pva->angle_roll * 360 / 32768. * DEG_TO_RAD, Eigen::Vector3d::UnitY());
#endif
  msg_imu_.orientation.w = (double)q.w();
  msg_imu_.orientation.x = (double)q.x();
  msg_imu_.orientation.y = (double)q.y();
  msg_imu_.orientation.z = (double)q.z();
  msg_imu_.orientation_covariance[0] = 0.0;
  msg_imu_.orientation_covariance[4] = 0.0;
  msg_imu_.orientation_covariance[8] = 0.0;

  ins_.mutable_linear_velocity()->set_y(pva->vel_e_m_s * 1e2 / 32768.);
  ins_.mutable_linear_velocity()->set_x(pva->vel_n_m_s * 1e2 / 32768.);
  ins_.mutable_linear_velocity()->set_z(pva->vel_u_m_s * 1e2 / 32768.);

  bestpos_.set_latitude(pva->latitude * 1e-7);
  bestpos_.set_longitude(pva->longitude * 1e-7);
  bestpos_.set_height_msl(pva->altitude * 1e-3);
  bestpos_.set_latitude_std_dev(0.51);
  bestpos_.set_longitude_std_dev(0.51);
  bestpos_.set_height_std_dev(0.51);

  switch (pva->fix_type) {
    case ::apollo::drivers::gnss::zhd::FIX_TYPE_RT2:
      ins_.set_type(apollo::drivers::gnss::Ins::GOOD);
      bestpos_.set_sol_status(
          apollo::drivers::gnss::SolutionStatus::SOL_COMPUTED);
      bestpos_.set_sol_type(
          apollo::drivers::gnss::SolutionType::NARROW_INT);
      bestpos_.set_latitude_std_dev(0.01);
      bestpos_.set_longitude_std_dev(0.01);
      bestpos_.set_height_std_dev(0.01);
      break;
    case ::apollo::drivers::gnss::zhd::FIX_TYPE_SPOINT:
      ins_.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      bestpos_.set_sol_type(
          apollo::drivers::gnss::SolutionType::SINGLE);
      break;
    case ::apollo::drivers::gnss::zhd::FIX_TYPE_PSRDIFF:
      ins_.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      bestpos_.set_sol_type(
          apollo::drivers::gnss::SolutionType::PSRDIFF);
      break;
    case ::apollo::drivers::gnss::zhd::FIX_TYPE_FPOINT:
      ins_.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      bestpos_.set_sol_type(
          apollo::drivers::gnss::SolutionType::NARROW_FLOAT);
      break;
    default:
      ins_.set_type(apollo::drivers::gnss::Ins::INVALID);
      bestpos_.set_sol_status(
          apollo::drivers::gnss::SolutionStatus::INSUFFICIENT_OBS);
      bestpos_.set_sol_type(
          apollo::drivers::gnss::SolutionType::NONE);
      break;
  }

  bestpos_.set_reserved(pva->heading_status);
  bestpos_.set_num_sats_tracked(pva->satellites_track);
  bestpos_.set_num_sats_in_solution(pva->satellites_track);
  bestpos_.set_differential_age(pva->diff_age);

  double seconds =
      pva->week * SECONDS_PER_WEEK + pva->seconds;
  if (ins_.measurement_time() != seconds) {
    ins_.set_measurement_time(seconds);
  }
  bestpos_.set_measurement_time(seconds);

  ins_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  return true;
}

bool ZhdParser::HandleRawImu(uint8_t *pbuf, int32_t length) {
  const zhd::imu_zhd_packet_t* imu = reinterpret_cast<zhd::imu_zhd_packet_t*>(pbuf);
  float imu_measurement_span = 1.0 / 200.0;

  double time = imu->week * SECONDS_PER_WEEK + imu->seconds;
  if (imu_measurement_time_previous_ > 0.0 &&
      fabs(time - imu_measurement_time_previous_ - imu_measurement_span) >
          1.5e-2) {
    AWARN << "Unexpected delay between two IMU measurements at: "
          << time - imu_measurement_time_previous_;
  }

  imu_.set_measurement_time(time);

  ins_.mutable_linear_acceleration()->set_y(imu->vel_acc_x * 8. *  9.8 / 32768.);
  ins_.mutable_linear_acceleration()->set_x(imu->vel_acc_y * 8. *  9.8 / 32768.);
  ins_.mutable_linear_acceleration()->set_z(imu->vel_acc_z * 8. * -9.8 / 32768.);
  
  ins_.mutable_angular_velocity()->set_y( imu->vel_gyro_x * 250. / 32768. * DEG_TO_RAD);
  ins_.mutable_angular_velocity()->set_x( imu->vel_gyro_y * 250. / 32768. * DEG_TO_RAD);
  ins_.mutable_angular_velocity()->set_z(-imu->vel_gyro_z * 250. / 32768. * DEG_TO_RAD);
#if 0
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                       msg.orientation.y, msg.orientation.z);
  Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(0,1,2);
  ins_.mutable_euler_angles()->set_x(eulerAngle.x());
  ins_.mutable_euler_angles()->set_y(eulerAngle.y());
  ins_.mutable_euler_angles()->set_z(eulerAngle.z());
#endif
  imu_.mutable_linear_acceleration()->set_y(imu->vel_acc_x * 8. *  9.8 / 32768.);
  imu_.mutable_linear_acceleration()->set_x(imu->vel_acc_y * 8. *  9.8 / 32768.);
  imu_.mutable_linear_acceleration()->set_z(imu->vel_acc_z * 8. * -9.8 / 32768.);

  imu_.mutable_angular_velocity()->set_y( imu->vel_gyro_x * 250. / 32768. * DEG_TO_RAD);
  imu_.mutable_angular_velocity()->set_x( imu->vel_gyro_y * 250. / 32768. * DEG_TO_RAD);
  imu_.mutable_angular_velocity()->set_z(-imu->vel_gyro_z * 250. / 32768. * DEG_TO_RAD);
#if 0
  double magx = msg_mag.magnetic_field.x * cos(pitch) +
                msg_mag.magnetic_field.y * sin(roll) * sin(pitch) + 
                msg_mag.magnetic_field.z * cos(roll) * sin(pitch);
  double magy = msg_mag.magnetic_field.y * cos(roll) -
                msg_mag.magnetic_field.z * sin(roll);
  double yaw = 180 * atan2(-magy, magx) / M_PI;
#endif
  msg_imu_.header.stamp = ros::Time::now();
  msg_imu_.header.frame_id = "zhd_imu";

  msg_imu_.angular_velocity.y =  imu->vel_gyro_x * 250. / 32768. * DEG_TO_RAD;
  msg_imu_.angular_velocity.x =  imu->vel_gyro_y * 250. / 32768. * DEG_TO_RAD;
  msg_imu_.angular_velocity.z = -imu->vel_gyro_z * 250. / 32768. * DEG_TO_RAD;
  msg_imu_.angular_velocity_covariance[0] = 0.0;
  msg_imu_.angular_velocity_covariance[4] = 0.0;
  msg_imu_.angular_velocity_covariance[8] = 0.0;

  msg_imu_.linear_acceleration.y = imu->vel_acc_x * 8. *  9.8 / 32768.;
  msg_imu_.linear_acceleration.x = imu->vel_acc_y * 8. *  9.8 / 32768.;
  msg_imu_.linear_acceleration.z = imu->vel_acc_z * 8. * -9.8 / 32768.;
  msg_imu_.linear_acceleration_covariance[0] = 0.0;
  msg_imu_.linear_acceleration_covariance[4] = 0.0;
  msg_imu_.linear_acceleration_covariance[8] = 0.0;
  imu_pub_.publish(msg_imu_);

  msg_mag_.magnetic_field.x = imu->vel_magn_x;
  msg_mag_.magnetic_field.y = imu->vel_magn_y;
  msg_mag_.magnetic_field.z = imu->vel_magn_z;
  msg_mag_.header.stamp = ros::Time::now();
  msg_mag_.header.frame_id = "imu";
  mag_pub_.publish(msg_mag_);

  imu_measurement_time_previous_ = time;
  return true;
}

bool ZhdParser::HandleStaData(uint8_t *pbuf, int32_t length) {
  const zhd::status_zhd_packet_t* sta = reinterpret_cast<zhd::status_zhd_packet_t*>(pbuf);
  uint32_t reserved = sta->calib_stat;
  bestpos_.set_datum_id(apollo::drivers::gnss::DatumId::WGS84);
  if (sta->rtcm_stat) {
    bestpos_.set_base_station_id("0");
  } else {
    bestpos_.set_base_station_id("");
  }
  reserved <<= 8;
  reserved |= sta->gnss_stat;
  reserved <<= 8;
  reserved |= sta->imu_stat;
  reserved <<= 8;
  reserved |= sta->odo_stat;
  bestpos_.set_extended_solution_status(reserved);
  bestpos_.set_galileo_beidou_used_mask(sta->status);
  bestpos_.set_gps_glonass_used_mask(sta->expired_stat);

  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
