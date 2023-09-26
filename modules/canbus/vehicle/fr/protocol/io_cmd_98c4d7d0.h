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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_IO_CMD_98C4D7D0_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_IO_CMD_98C4D7D0_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Iocmd98c4d7d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Iocmd98c4d7d0();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'io_cmd_disCharge', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_discharge(bool io_cmd_discharge);

  // config detail: {'name': 'io_cmd_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_check_bcc(int io_cmd_check_bcc);

  // config detail: {'name': 'io_cmd_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_alive_cnt(int io_cmd_alive_cnt);

  // config detail: {'name': 'io_cmd_speaker', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_speaker(bool io_cmd_speaker);

  // config detail: {'name': 'io_cmd_fog_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_fog_lamp(bool io_cmd_fog_lamp);

  // config detail: {'name': 'io_cmd_clearance_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_clearance_lamp(bool io_cmd_clearance_lamp);

  // config detail: {'name': 'io_cmd_braking_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_braking_lamp(bool io_cmd_braking_lamp);

  // config detail: {'name': 'io_cmd_turn_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_turn_lamp(int io_cmd_turn_lamp);

  // config detail: {'name': 'io_cmd_upper_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_upper_beam_headlamp(bool io_cmd_upper_beam_headlamp);

  // config detail: {'name': 'io_cmd_lower_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_lower_beam_headlamp(bool io_cmd_lower_beam_headlamp);

  // config detail: {'name': 'io_cmd_enable', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Iocmd98c4d7d0* set_io_cmd_enable(bool io_cmd_enable);

 private:

  // config detail: {'name': 'io_cmd_disCharge', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_discharge(uint8_t* data, bool io_cmd_discharge);

  // config detail: {'name': 'io_cmd_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_check_bcc(uint8_t* data, int io_cmd_check_bcc);

  // config detail: {'name': 'io_cmd_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_alive_cnt(uint8_t* data, int io_cmd_alive_cnt);

  // config detail: {'name': 'io_cmd_speaker', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_speaker(uint8_t* data, bool io_cmd_speaker);

  // config detail: {'name': 'io_cmd_fog_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_fog_lamp(uint8_t* data, bool io_cmd_fog_lamp);

  // config detail: {'name': 'io_cmd_clearance_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_clearance_lamp(uint8_t* data, bool io_cmd_clearance_lamp);

  // config detail: {'name': 'io_cmd_braking_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_braking_lamp(uint8_t* data, bool io_cmd_braking_lamp);

  // config detail: {'name': 'io_cmd_turn_lamp', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_turn_lamp(uint8_t* data, int io_cmd_turn_lamp);

  // config detail: {'name': 'io_cmd_upper_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_upper_beam_headlamp(uint8_t* data, bool io_cmd_upper_beam_headlamp);

  // config detail: {'name': 'io_cmd_lower_beam_headlamp', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_lower_beam_headlamp(uint8_t* data, bool io_cmd_lower_beam_headlamp);

  // config detail: {'name': 'io_cmd_enable', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_io_cmd_enable(uint8_t* data, bool io_cmd_enable);

 private:
  bool io_cmd_discharge_;
  int io_cmd_check_bcc_;
  int io_cmd_alive_cnt_;
  bool io_cmd_speaker_;
  bool io_cmd_fog_lamp_;
  bool io_cmd_clearance_lamp_;
  bool io_cmd_braking_lamp_;
  int io_cmd_turn_lamp_;
  bool io_cmd_upper_beam_headlamp_;
  bool io_cmd_lower_beam_headlamp_;
  bool io_cmd_enable_;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_IO_CMD_98C4D7D0_H_
