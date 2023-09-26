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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_BMS_INFOR_98C4E1EF_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_BMS_INFOR_98C4E1EF_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Bmsinfor98c4e1ef : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bmsinfor98c4e1ef();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'bms_Infor_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bms_infor_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_Infor_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bms_infor_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_Infor_remaining_capacity', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double bms_infor_remaining_capacity(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_Infor_current', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double bms_infor_current(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_Infor_voltage', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double bms_infor_voltage(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_BMS_INFOR_98C4E1EF_H_
