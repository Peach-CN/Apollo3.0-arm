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

#include "modules/canbus/vehicle/fr/protocol/bms_infor_98c4e1ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

Bmsinfor98c4e1ef::Bmsinfor98c4e1ef() {}
const int32_t Bmsinfor98c4e1ef::ID = 0x98C4E1EF;

void Bmsinfor98c4e1ef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_fr()->mutable_bms_infor_98c4e1ef()->set_bms_infor_check_bcc(bms_infor_check_bcc(bytes, length));
  chassis->mutable_fr()->mutable_bms_infor_98c4e1ef()->set_bms_infor_alive_cnt(bms_infor_alive_cnt(bytes, length));
  chassis->mutable_fr()->mutable_bms_infor_98c4e1ef()->set_bms_infor_remaining_capacity(bms_infor_remaining_capacity(bytes, length));
  chassis->mutable_fr()->mutable_bms_infor_98c4e1ef()->set_bms_infor_current(bms_infor_current(bytes, length));
  chassis->mutable_fr()->mutable_bms_infor_98c4e1ef()->set_bms_infor_voltage(bms_infor_voltage(bytes, length));
}

// config detail: {'name': 'bms_infor_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Bmsinfor98c4e1ef::bms_infor_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_infor_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Bmsinfor98c4e1ef::bms_infor_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_infor_remaining_capacity', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Bmsinfor98c4e1ef::bms_infor_remaining_capacity(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'bms_infor_current', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Bmsinfor98c4e1ef::bms_infor_current(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'bms_infor_voltage', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Bmsinfor98c4e1ef::bms_infor_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace fr
}  // namespace canbus
}  // namespace apollo
