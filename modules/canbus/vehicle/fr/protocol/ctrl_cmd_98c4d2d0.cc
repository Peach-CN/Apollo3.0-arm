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

#include "modules/canbus/vehicle/fr/protocol/ctrl_cmd_98c4d2d0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

const int32_t Ctrlcmd98c4d2d0::ID = 0x98C4D2D0;

// public
Ctrlcmd98c4d2d0::Ctrlcmd98c4d2d0() { Reset(); }

uint32_t Ctrlcmd98c4d2d0::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}


void Ctrlcmd98c4d2d0::UpdateData(uint8_t* data) {
  set_p_ctrl_cmd_brake(data, ctrl_cmd_brake_);
  set_p_ctrl_cmd_steering(data, ctrl_cmd_steering_);
  set_p_ctrl_cmd_velocity(data, ctrl_cmd_velocity_);
  set_p_ctrl_cmd_gear(data, ctrl_cmd_gear_);

  
  set_p_ctrl_cmd_alive_cnt(data, ctrl_cmd_alive_cnt_);
  ctrl_cmd_check_bcc_ = data[0];
  for (int i = 1; i < 7; ++i) ctrl_cmd_check_bcc_ ^= data[i];
  set_p_ctrl_cmd_check_bcc(data, ctrl_cmd_check_bcc_);
}

void Ctrlcmd98c4d2d0::Reset() {
  // TODO you should check this manually
  ctrl_cmd_check_bcc_ = 0;
  ctrl_cmd_alive_cnt_ = 0;
  ctrl_cmd_brake_ = 0;
  ctrl_cmd_steering_ = 0.0;
  ctrl_cmd_velocity_ = 0.0;
  ctrl_cmd_gear_ = 0;
}

Ctrlcmd98c4d2d0* Ctrlcmd98c4d2d0::set_ctrl_cmd_check_bcc(
    int ctrl_cmd_check_bcc) {
  ctrl_cmd_check_bcc_ = ctrl_cmd_check_bcc;
  return this;
 }

// config detail: {'name': 'ctrl_cmd_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Ctrlcmd98c4d2d0::set_p_ctrl_cmd_check_bcc(uint8_t* data,
    int ctrl_cmd_check_bcc) {
  ctrl_cmd_check_bcc = ProtocolData::BoundedValue(0, 255, ctrl_cmd_check_bcc);
  int x = ctrl_cmd_check_bcc;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Ctrlcmd98c4d2d0* Ctrlcmd98c4d2d0::set_ctrl_cmd_alive_cnt(
    int ctrl_cmd_alive_cnt) {
  ctrl_cmd_alive_cnt_ = ctrl_cmd_alive_cnt;
  return this;
 }

// config detail: {'name': 'ctrl_cmd_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Ctrlcmd98c4d2d0::set_p_ctrl_cmd_alive_cnt(uint8_t* data,
    int ctrl_cmd_alive_cnt) {
  ctrl_cmd_alive_cnt = ProtocolData::BoundedValue(0, 15, ctrl_cmd_alive_cnt);
  int x = ctrl_cmd_alive_cnt;

  static int ctrl_cmd_alive_cnt_temp = 0;
  ctrl_cmd_alive_cnt_temp ++;
  x = ctrl_cmd_alive_cnt_temp / 2;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 4);
}


Ctrlcmd98c4d2d0* Ctrlcmd98c4d2d0::set_ctrl_cmd_brake(
    int ctrl_cmd_brake) {
  ctrl_cmd_brake_ = ctrl_cmd_brake;
  return this;
 }

// config detail: {'name': 'ctrl_cmd_Brake', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|127]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Ctrlcmd98c4d2d0::set_p_ctrl_cmd_brake(uint8_t* data,
    int ctrl_cmd_brake) {
  ctrl_cmd_brake = ProtocolData::BoundedValue(0, 127, ctrl_cmd_brake);
  int x = ctrl_cmd_brake;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 4);
}


Ctrlcmd98c4d2d0* Ctrlcmd98c4d2d0::set_ctrl_cmd_steering(
    double ctrl_cmd_steering) {
  ctrl_cmd_steering_ = ctrl_cmd_steering;
  AINFO << "ctrl_cmd_steering : " << ctrl_cmd_steering;
  return this;
 }

// config detail: {'name': 'ctrl_cmd_steering', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 20, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Ctrlcmd98c4d2d0::set_p_ctrl_cmd_steering(uint8_t* data,
    double ctrl_cmd_steering) {
  ctrl_cmd_steering = ProtocolData::BoundedValue(-25.0, 25.0, ctrl_cmd_steering);
  int x = ctrl_cmd_steering / 0.010000;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xF;
  Byte to_set2(data + 4);
  to_set2.set_value(t, 0, 4);
}


Ctrlcmd98c4d2d0* Ctrlcmd98c4d2d0::set_ctrl_cmd_velocity(
    double ctrl_cmd_velocity) {
  ctrl_cmd_velocity_ = ctrl_cmd_velocity;
  return this;
 }

// config detail: {'name': 'ctrl_cmd_velocity', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 4, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Ctrlcmd98c4d2d0::set_p_ctrl_cmd_velocity(uint8_t* data,
    double ctrl_cmd_velocity) {
  ctrl_cmd_velocity = ProtocolData::BoundedValue(0.0, 1.5, ctrl_cmd_velocity);
  int x = ctrl_cmd_velocity / 0.001000;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xF;
  Byte to_set2(data + 2);
  to_set2.set_value(t, 0, 4);
}


Ctrlcmd98c4d2d0* Ctrlcmd98c4d2d0::set_ctrl_cmd_gear(
    int ctrl_cmd_gear) {
  ctrl_cmd_gear_ = ctrl_cmd_gear;
  return this;
 }

// config detail: {'name': 'ctrl_cmd_gear', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Ctrlcmd98c4d2d0::set_p_ctrl_cmd_gear(uint8_t* data,
    int ctrl_cmd_gear) {
  ctrl_cmd_gear = ProtocolData::BoundedValue(0, 4, ctrl_cmd_gear);
  int x = ctrl_cmd_gear;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 4);
}

}  // namespace fr
}  // namespace canbus
}  // namespace apollo
