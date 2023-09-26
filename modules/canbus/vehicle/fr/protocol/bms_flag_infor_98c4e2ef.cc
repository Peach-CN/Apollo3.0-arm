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

#include "modules/canbus/vehicle/fr/protocol/bms_flag_infor_98c4e2ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

Bmsflaginfor98c4e2ef::Bmsflaginfor98c4e2ef() {}
const int32_t Bmsflaginfor98c4e2ef::ID = 0x98C4E2EF;

void Bmsflaginfor98c4e2ef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_check_bcc(bms_flag_infor_check_bcc(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_alive_cnt(bms_flag_infor_alive_cnt(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_reserved(reserved(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_low_temperature(bms_flag_infor_low_temperature(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_hight_temperature(bms_flag_infor_hight_temperature(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_charge_flag(bms_flag_infor_charge_flag(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_lock_mos(bms_flag_infor_lock_mos(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_ic_error(bms_flag_infor_ic_error(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_short(bms_flag_infor_short(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_discharge_oc(bms_flag_infor_discharge_oc(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_charge_oc(bms_flag_infor_charge_oc(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_discharge_ut(bms_flag_infor_discharge_ut(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_discharge_ot(bms_flag_infor_discharge_ot(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_charge_ut(bms_flag_infor_charge_ut(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_charge_ot(bms_flag_infor_charge_ot(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_uv(bms_flag_infor_uv(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_ov(bms_flag_infor_ov(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_single_uv(bms_flag_infor_single_uv(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_single_ov(bms_flag_infor_single_ov(bytes, length));
  chassis->mutable_fr()->mutable_bms_flag_infor_98c4e2ef()->set_bms_flag_infor_soc(bms_flag_infor_soc(bytes, length));
}

// config detail: {'name': 'bms_flag_infor_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Bmsflaginfor98c4e2ef::bms_flag_infor_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Bmsflaginfor98c4e2ef::bms_flag_infor_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'reserved', 'offset': 0.0, 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 22, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Bmsflaginfor98c4e2ef::reserved(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_low_temperature', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Bmsflaginfor98c4e2ef::bms_flag_infor_low_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_hight_temperature', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 28, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Bmsflaginfor98c4e2ef::bms_flag_infor_hight_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_charge_flag', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 21, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_charge_flag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_lock_mos', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 20, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_lock_mos(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_ic_error', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 19, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_ic_error(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_short', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 18, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_short(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_discharge_oc', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 17, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_discharge_oc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_charge_oc', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_charge_oc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_discharge_ut', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_discharge_ut(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_discharge_ot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_discharge_ot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_charge_ut', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_charge_ut(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_charge_ot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_charge_ot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_uv', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 11, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_uv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_ov', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_ov(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_single_uv', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_single_uv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_single_ov', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Bmsflaginfor98c4e2ef::bms_flag_infor_single_ov(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'bms_flag_infor_soc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Bmsflaginfor98c4e2ef::bms_flag_infor_soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace fr
}  // namespace canbus
}  // namespace apollo
