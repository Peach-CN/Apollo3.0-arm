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

#include "modules/canbus/vehicle/fr/protocol/io_cmd_98c4d7d0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

const int32_t Iocmd98c4d7d0::ID = 0x98C4D7D0;

// public
Iocmd98c4d7d0::Iocmd98c4d7d0() { Reset(); }

uint32_t Iocmd98c4d7d0::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Iocmd98c4d7d0::UpdateData(uint8_t* data) {
  set_p_io_cmd_discharge(data, io_cmd_discharge_);
  set_p_io_cmd_check_bcc(data, io_cmd_check_bcc_);
  set_p_io_cmd_alive_cnt(data, io_cmd_alive_cnt_);
  set_p_io_cmd_speaker(data, io_cmd_speaker_);
  set_p_io_cmd_fog_lamp(data, io_cmd_fog_lamp_);
  set_p_io_cmd_clearance_lamp(data, io_cmd_clearance_lamp_);
  set_p_io_cmd_braking_lamp(data, io_cmd_braking_lamp_);
  set_p_io_cmd_turn_lamp(data, io_cmd_turn_lamp_);
  set_p_io_cmd_upper_beam_headlamp(data, io_cmd_upper_beam_headlamp_);
  set_p_io_cmd_lower_beam_headlamp(data, io_cmd_lower_beam_headlamp_);
  set_p_io_cmd_enable(data, io_cmd_enable_);
}

void Iocmd98c4d7d0::Reset() {
  // TODO you should check this manually
  io_cmd_discharge_ = false;
  io_cmd_check_bcc_ = 0;
  io_cmd_alive_cnt_ = 0;
  io_cmd_speaker_ = false;
  io_cmd_fog_lamp_ = false;
  io_cmd_clearance_lamp_ = false;
  io_cmd_braking_lamp_ = false;
  io_cmd_turn_lamp_ = 0;
  io_cmd_upper_beam_headlamp_ = false;
  io_cmd_lower_beam_headlamp_ = false;
  io_cmd_enable_ = false;
}

Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_discharge(
    bool io_cmd_discharge) {
  io_cmd_discharge_ = io_cmd_discharge;
  return this;
 }

// config detail: {'name': 'io_cmd_disCharge', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_discharge(uint8_t* data,
    bool io_cmd_discharge) {
  int x = io_cmd_discharge;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_check_bcc(
    int io_cmd_check_bcc) {
  io_cmd_check_bcc_ = io_cmd_check_bcc;
  return this;
 }

// config detail: {'name': 'io_cmd_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_check_bcc(uint8_t* data,
    int io_cmd_check_bcc) {
  io_cmd_check_bcc = ProtocolData::BoundedValue(0, 0, io_cmd_check_bcc);
  int x = io_cmd_check_bcc;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_alive_cnt(
    int io_cmd_alive_cnt) {
  io_cmd_alive_cnt_ = io_cmd_alive_cnt;
  return this;
 }

// config detail: {'name': 'io_cmd_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_alive_cnt(uint8_t* data,
    int io_cmd_alive_cnt) {
  io_cmd_alive_cnt = ProtocolData::BoundedValue(0, 0, io_cmd_alive_cnt);
  int x = io_cmd_alive_cnt;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 4);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_speaker(
    bool io_cmd_speaker) {
  io_cmd_speaker_ = io_cmd_speaker;
  return this;
 }

// config detail: {'name': 'io_cmd_speaker', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_speaker(uint8_t* data,
    bool io_cmd_speaker) {
  int x = io_cmd_speaker;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_fog_lamp(
    bool io_cmd_fog_lamp) {
  io_cmd_fog_lamp_ = io_cmd_fog_lamp;
  return this;
 }

// config detail: {'name': 'io_cmd_fog_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_fog_lamp(uint8_t* data,
    bool io_cmd_fog_lamp) {
  int x = io_cmd_fog_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 6, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_clearance_lamp(
    bool io_cmd_clearance_lamp) {
  io_cmd_clearance_lamp_ = io_cmd_clearance_lamp;
  return this;
 }

// config detail: {'name': 'io_cmd_clearance_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_clearance_lamp(uint8_t* data,
    bool io_cmd_clearance_lamp) {
  int x = io_cmd_clearance_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_braking_lamp(
    bool io_cmd_braking_lamp) {
  io_cmd_braking_lamp_ = io_cmd_braking_lamp;
  return this;
 }

// config detail: {'name': 'io_cmd_braking_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_braking_lamp(uint8_t* data,
    bool io_cmd_braking_lamp) {
  int x = io_cmd_braking_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_turn_lamp(
    int io_cmd_turn_lamp) {
  io_cmd_turn_lamp_ = io_cmd_turn_lamp;
  return this;
 }

// config detail: {'name': 'io_cmd_turn_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_turn_lamp(uint8_t* data,
    int io_cmd_turn_lamp) {
  io_cmd_turn_lamp = ProtocolData::BoundedValue(0, 0, io_cmd_turn_lamp);
  int x = io_cmd_turn_lamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 2);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_upper_beam_headlamp(
    bool io_cmd_upper_beam_headlamp) {
  io_cmd_upper_beam_headlamp_ = io_cmd_upper_beam_headlamp;
  return this;
 }

// config detail: {'name': 'io_cmd_upper_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_upper_beam_headlamp(uint8_t* data,
    bool io_cmd_upper_beam_headlamp) {
  int x = io_cmd_upper_beam_headlamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_lower_beam_headlamp(
    bool io_cmd_lower_beam_headlamp) {
  io_cmd_lower_beam_headlamp_ = io_cmd_lower_beam_headlamp;
  return this;
 }

// config detail: {'name': 'io_cmd_lower_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_lower_beam_headlamp(uint8_t* data,
    bool io_cmd_lower_beam_headlamp) {
  int x = io_cmd_lower_beam_headlamp;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}


Iocmd98c4d7d0* Iocmd98c4d7d0::set_io_cmd_enable(
    bool io_cmd_enable) {
  io_cmd_enable_ = io_cmd_enable;
  return this;
 }

// config detail: {'name': 'io_cmd_enable', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Iocmd98c4d7d0::set_p_io_cmd_enable(uint8_t* data,
    bool io_cmd_enable) {
  int x = io_cmd_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

}  // namespace fr
}  // namespace canbus
}  // namespace apollo
