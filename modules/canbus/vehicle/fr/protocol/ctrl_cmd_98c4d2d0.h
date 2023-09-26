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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_CTRL_CMD_98C4D2D0_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_CTRL_CMD_98C4D2D0_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Ctrlcmd98c4d2d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Ctrlcmd98c4d2d0();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'ctrl_cmd_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Ctrlcmd98c4d2d0* set_ctrl_cmd_check_bcc(int ctrl_cmd_check_bcc);

  // config detail: {'name': 'ctrl_cmd_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Ctrlcmd98c4d2d0* set_ctrl_cmd_alive_cnt(int ctrl_cmd_alive_cnt);

  // config detail: {'name': 'ctrl_cmd_Brake', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|127]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Ctrlcmd98c4d2d0* set_ctrl_cmd_brake(int ctrl_cmd_brake);

  // config detail: {'name': 'ctrl_cmd_steering', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 20, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  Ctrlcmd98c4d2d0* set_ctrl_cmd_steering(double ctrl_cmd_steering);

  // config detail: {'name': 'ctrl_cmd_velocity', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 4, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  Ctrlcmd98c4d2d0* set_ctrl_cmd_velocity(double ctrl_cmd_velocity);

  // config detail: {'name': 'ctrl_cmd_gear', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Ctrlcmd98c4d2d0* set_ctrl_cmd_gear(int ctrl_cmd_gear);

 private:

  // config detail: {'name': 'ctrl_cmd_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ctrl_cmd_check_bcc(uint8_t* data, int ctrl_cmd_check_bcc);

  // config detail: {'name': 'ctrl_cmd_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ctrl_cmd_alive_cnt(uint8_t* data, int ctrl_cmd_alive_cnt);

  // config detail: {'name': 'ctrl_cmd_Brake', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|127]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ctrl_cmd_brake(uint8_t* data, int ctrl_cmd_brake);

  // config detail: {'name': 'ctrl_cmd_steering', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 20, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  void set_p_ctrl_cmd_steering(uint8_t* data, double ctrl_cmd_steering);

  // config detail: {'name': 'ctrl_cmd_velocity', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 4, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  void set_p_ctrl_cmd_velocity(uint8_t* data, double ctrl_cmd_velocity);

  // config detail: {'name': 'ctrl_cmd_gear', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_ctrl_cmd_gear(uint8_t* data, int ctrl_cmd_gear);

 private:
  int ctrl_cmd_check_bcc_;
  int ctrl_cmd_alive_cnt_;
  int ctrl_cmd_brake_;
  double ctrl_cmd_steering_;
  double ctrl_cmd_velocity_;
  int ctrl_cmd_gear_;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_CTRL_CMD_98C4D2D0_H_
