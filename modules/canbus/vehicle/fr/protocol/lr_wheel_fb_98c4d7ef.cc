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

#include "modules/canbus/vehicle/fr/protocol/lr_wheel_fb_98c4d7ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

Lrwheelfb98c4d7ef::Lrwheelfb98c4d7ef() {}
const int32_t Lrwheelfb98c4d7ef::ID = 0x98C4D7EF;

void Lrwheelfb98c4d7ef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_fr()->mutable_lr_wheel_fb_98c4d7ef()->set_lr_wheel_fb_check_bcc(lr_wheel_fb_check_bcc(bytes, length));
  chassis->mutable_fr()->mutable_lr_wheel_fb_98c4d7ef()->set_lr_wheel_fb_alive_cnt(lr_wheel_fb_alive_cnt(bytes, length));
  chassis->mutable_fr()->mutable_lr_wheel_fb_98c4d7ef()->set_lr_wheel_fb_pulse(lr_wheel_fb_pulse(bytes, length));
  chassis->mutable_fr()->mutable_lr_wheel_fb_98c4d7ef()->set_lr_wheel_fb_velocity(lr_wheel_fb_velocity(bytes, length));
}

// config detail: {'name': 'lr_wheel_fb_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Lrwheelfb98c4d7ef::lr_wheel_fb_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'lr_wheel_fb_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Lrwheelfb98c4d7ef::lr_wheel_fb_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'lr_wheel_fb_pulse', 'offset': 0.0, 'precision': 1.0, 'len': 32, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Lrwheelfb98c4d7ef::lr_wheel_fb_pulse(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 3);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 2);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}

// config detail: {'name': 'lr_wheel_fb_velocity', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Lrwheelfb98c4d7ef::lr_wheel_fb_velocity(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace fr
}  // namespace canbus
}  // namespace apollo
