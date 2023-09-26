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

#include "modules/canbus/vehicle/fr/protocol/drive_fb_mcuecoder_98c4dcef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

Drivefbmcuecoder98c4dcef::Drivefbmcuecoder98c4dcef() {}
const int32_t Drivefbmcuecoder98c4dcef::ID = 0x98C4DCEF;

void Drivefbmcuecoder98c4dcef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_fr()->mutable_drive_fb_mcuecoder_98c4dcef()->set_drive_fb_mcuecoder_check_bcc(drive_fb_mcuecoder_check_bcc(bytes, length));
  chassis->mutable_fr()->mutable_drive_fb_mcuecoder_98c4dcef()->set_drive_fb_mcuecoder_alive_cnt(drive_fb_mcuecoder_alive_cnt(bytes, length));
  chassis->mutable_fr()->mutable_drive_fb_mcuecoder_98c4dcef()->set_drive_fb_mcuecoder(drive_fb_mcuecoder(bytes, length));
}

// config detail: {'name': 'drive_fb_mcuecoder_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Drivefbmcuecoder98c4dcef::drive_fb_mcuecoder_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'drive_fb_mcuecoder_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Drivefbmcuecoder98c4dcef::drive_fb_mcuecoder_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'drive_fb_mcuecoder', 'offset': 0.0, 'precision': 1.0, 'len': 32, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Drivefbmcuecoder98c4dcef::drive_fb_mcuecoder(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 1);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 0);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}
}  // namespace fr
}  // namespace canbus
}  // namespace apollo
