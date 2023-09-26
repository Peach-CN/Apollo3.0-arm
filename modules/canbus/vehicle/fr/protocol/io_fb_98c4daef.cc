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

#include "modules/canbus/vehicle/fr/protocol/io_fb_98c4daef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

Iofb98c4daef::Iofb98c4daef() {}
const int32_t Iofb98c4daef::ID = 0x98C4DAEF;

void Iofb98c4daef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_chargeen(io_fb_chargeen(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_dischargeflg(io_fb_dischargeflg(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_check_bcc(io_fb_check_bcc(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_alive_cnt(io_fb_alive_cnt(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_rr_drop_sensor(io_fb_rr_drop_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_rm_drop_sensor(io_fb_rm_drop_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_rl_drop_sensor(io_fb_rl_drop_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fr_drop_sensor(io_fb_fr_drop_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fm_drop_sensor(io_fb_fm_drop_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fl_drop_sensor(io_fb_fl_drop_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_rr_impact_sensor(io_fb_rr_impact_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_rm_impact_sensor(io_fb_rm_impact_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_rl_impact_sensor(io_fb_rl_impact_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fr_impact_sensor(io_fb_fr_impact_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fm_impact_sensor(io_fb_fm_impact_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fl_impact_sensor(io_fb_fl_impact_sensor(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_speaker(io_fb_speaker(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_fog_lamp(io_fb_fog_lamp(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_clearance_lamp(io_fb_clearance_lamp(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_braking_lamp(io_fb_braking_lamp(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_turn_lamp(io_fb_turn_lamp(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_upper_beam_headlamp(io_fb_upper_beam_headlamp(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_lower_beam_headlamp(io_fb_lower_beam_headlamp(bytes, length));
  chassis->mutable_fr()->mutable_io_fb_98c4daef()->set_io_fb_enable(io_fb_enable(bytes, length));
}

// config detail: {'name': 'io_fb_chargeen', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 41, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_chargeen(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_dischargeflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_dischargeflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Iofb98c4daef::io_fb_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Iofb98c4daef::io_fb_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_rr_drop_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 37, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_rr_drop_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_rm_drop_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_rm_drop_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_rl_drop_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 35, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_rl_drop_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fr_drop_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 34, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fr_drop_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fm_drop_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fm_drop_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fl_drop_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fl_drop_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_rr_impact_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 29, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_rr_impact_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_rm_impact_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 28, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_rm_impact_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_rl_impact_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 27, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_rl_impact_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fr_impact_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fr_impact_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fm_impact_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 25, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fm_impact_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fl_impact_sensor', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fl_impact_sensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_speaker', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_speaker(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_fog_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_fog_lamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_clearance_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_clearance_lamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_braking_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_braking_lamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_turn_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Iofb98c4daef::io_fb_turn_lamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_upper_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_upper_beam_headlamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_lower_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_lower_beam_headlamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'io_fb_enable', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Iofb98c4daef::io_fb_enable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace fr
}  // namespace canbus
}  // namespace apollo
