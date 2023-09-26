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

#include "modules/canbus/vehicle/fr/protocol/sensor_reset_98ffffff.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

const int32_t Sensorreset98ffffff::ID = 0x98FFFFFF;

// public
Sensorreset98ffffff::Sensorreset98ffffff() { Reset(); }

uint32_t Sensorreset98ffffff::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Sensorreset98ffffff::UpdateData(uint8_t* data) {
  set_p_close_candiag(data, close_candiag_);
  set_p_brake_reset(data, brake_reset_);
  set_p_steer_reset(data, steer_reset_);
}

void Sensorreset98ffffff::Reset() {
  // TODO you should check this manually
  close_candiag_ = false;
  brake_reset_ = false;
  steer_reset_ = false;
}

Sensorreset98ffffff* Sensorreset98ffffff::set_close_candiag(
    bool close_candiag) {
  close_candiag_ = close_candiag;
  return this;
 }

// config detail: {'name': 'Close_candiag', 'offset': 0.0, 'precision': -1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[-1|0]', 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Sensorreset98ffffff::set_p_close_candiag(uint8_t* data,
    bool close_candiag) {
  int x = close_candiag / -1.000000;

  Byte to_set(data + 7);
  to_set.set_value(x, 7, 1);
}


Sensorreset98ffffff* Sensorreset98ffffff::set_brake_reset(
    bool brake_reset) {
  brake_reset_ = brake_reset;
  return this;
 }

// config detail: {'name': 'Brake_reset', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Sensorreset98ffffff::set_p_brake_reset(uint8_t* data,
    bool brake_reset) {
  int x = brake_reset;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}


Sensorreset98ffffff* Sensorreset98ffffff::set_steer_reset(
    bool steer_reset) {
  steer_reset_ = steer_reset;
  return this;
 }

// config detail: {'name': 'steer_reset', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Sensorreset98ffffff::set_p_steer_reset(uint8_t* data,
    bool steer_reset) {
  int x = steer_reset;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

}  // namespace fr
}  // namespace canbus
}  // namespace apollo
