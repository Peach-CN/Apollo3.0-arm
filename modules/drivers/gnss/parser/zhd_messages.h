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
#pragma once

#include <stdint.h>
#include "modules/drivers/gnss/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace zhd {

#pragma pack(1)

enum SyncByte : uint8_t {
  GINS_SYNC_0 = 0xAC,
  GINS_SYNC_1 = 0xCA,
  GINS_SYNC_2 = 0x0C,
  TAIL_0 = 0xBF,
  IMU_SYNC_0 = 0x7E,
  IMU_SYNC_1 = 0x7E,
  IMU_SYNC_2 = 0x0C,
  STA_SYNC_0 = 0xBD,
  STA_SYNC_1 = 0xDB,
  STA_SYNC_2 = 0x0B,
};

// Fix_type 值,代表 RTK 定位解状态,Fix_type=4 为定位固定解有效工作状态
enum {
  FIX_TYPE_NONE = 0,    // 初始化
  FIX_TYPE_INS_CALC = 1,// 纯惯导推算
  FIX_TYPE_FPOINT = 2,  // 组合解算的浮点解
  FIX_TYPE_PSRDIFF = 3, // 组合解算的伪距差分
  FIX_TYPE_RT2 = 4,     // 组合解算的固定解
  FIX_TYPE_SPOINT = 5,  // 组合解算的单点解
};

enum {
  POS_TYPE_INVALID = 0,
  POS_TYPE_AUTONOMOUS_POS = 1,
  POS_TYPE_DOPPLER_VELOCITY = 1,
  POS_TYPE_DGPS = 2,
  POS_TYPE_NARROW_INT = 4,
  POS_TYPE_NARROW_FLOAT = 5,
};

typedef struct {
  unsigned char head[3];  // 数据头标志 AC CA 0C
  int32_t week;        // GPS周
  double seconds;      // GPS周内秒
  int32_t latitude;    // 纬度(度)
  int32_t longitude;   // 经度(度)
  int32_t altitude;    // 海拔高(m)

  int16_t vel_e_m_s;   // GPS 东 向速度 m/s
  int16_t vel_n_m_s;   // GPS 北 向速度 m/s
  int16_t vel_u_m_s;   // GPS 天 向速度,向上为正 m/s

  int16_t angle_pitch; // 俯仰角度值(±90°)
  int16_t angle_roll;  // 横滚角
  int16_t angle_azimuth;  // 方位角

  uint8_t satellites_track;  // 主天线卫星
  uint8_t fix_type;   // GINS惯导状态
  uint8_t align_status; // 初始对准状态
  uint8_t odo_status;   // Odo解算状态
  uint8_t heading_status;  // GNSS航向状态
  uint8_t vel_status; // GNSS速度状态
  uint16_t diff_age;  // 差分延时

  uint8_t checksum;   // 校验位，异或校验
  unsigned char tail; // 结尾标志 0xBF
} gps_rtk_zhd_packet_t;  //  total 49 bytes.

static_assert(sizeof(gps_rtk_zhd_packet_t) == 49, "structure size no align to one byte");

typedef struct {
  unsigned char head[3];// 数据头标志 7E 7E 0C
  int32_t week;         // GPS周
  double seconds;       // GPS周内秒

  int16_t vel_acc_x;    // X轴加速度，单位：g
  int16_t vel_acc_y;    // Y轴加速度，单位：g
  int16_t vel_acc_z;    // Z轴加速度，单位：g

  int16_t vel_gyro_x;   // 陀螺 X 轴，单位：deg/s
  int16_t vel_gyro_y;   // 陀螺 Y 轴，单位：deg/s
  int16_t vel_gyro_z;   // 陀螺 Z 轴，单位：deg/s

  int16_t vel_magn_x;   // 磁力计 X 轴，单位：G
  int16_t vel_magn_y;   // 磁力计 Y 轴，单位：G
  int16_t vel_magn_z;   // 磁力计 Z 轴，单位：G

  uint8_t checksum;     // 校验位，3~32 字节异
  unsigned char tail;   // 结尾标志 0xBF
} imu_zhd_packet_t;     //  total 35 bytes.

static_assert(sizeof(imu_zhd_packet_t) == 35, "structure size no align to one byte");

typedef struct {
  unsigned char head[3];     // 数据头标志 BD DB 0B
  unsigned char status;      // 状态字
  unsigned char expired_stat;// 设备过期状态说明：1（表示设备注册正常）；0（表示设备过期）
  unsigned char calib_stat;  // 标定状态：0（表示标定模式）
  unsigned char gnss_stat;   // GNSS数据源状态：0（表示GNSS数据源异常），1（表示GNSS数据源正常）
  unsigned char imu_stat;    // IMU数据源状态：0（表示IMU数据源异常），1（表示IMU数据源正常）
  unsigned char rtcm_stat;   // 差分数据源状态：0（表示差分数据源异常），1（表示差分数据源正常）
  unsigned char odo_stat;    // 轮速数据源状态：0（表示轮速数据源异常），1（表示轮速数据源正常）
  unsigned char checksum;    // 校验位，3~9字节异或和
  unsigned char tail;        // 帧尾 BF
} status_zhd_packet_t;

static_assert(sizeof(status_zhd_packet_t) == 12, "structure size no align to one byte");

#pragma pack()

}  // namespace zhd
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
