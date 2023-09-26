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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_SENSOR_RESET_98FFFFFF_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_SENSOR_RESET_98FFFFFF_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Sensorreset98ffffff : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Sensorreset98ffffff();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Close_candiag', 'offset': 0.0, 'precision': -1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[-1|0]', 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Sensorreset98ffffff* set_close_candiag(bool close_candiag);

  // config detail: {'name': 'Brake_reset', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Sensorreset98ffffff* set_brake_reset(bool brake_reset);

  // config detail: {'name': 'steer_reset', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Sensorreset98ffffff* set_steer_reset(bool steer_reset);

 private:

  // config detail: {'name': 'Close_candiag', 'offset': 0.0, 'precision': -1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[-1|0]', 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_close_candiag(uint8_t* data, bool close_candiag);

  // config detail: {'name': 'Brake_reset', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_brake_reset(uint8_t* data, bool brake_reset);

  // config detail: {'name': 'steer_reset', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_steer_reset(uint8_t* data, bool steer_reset);

 private:
  bool close_candiag_;
  bool brake_reset_;
  bool steer_reset_;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_SENSOR_RESET_98FFFFFF_H_
