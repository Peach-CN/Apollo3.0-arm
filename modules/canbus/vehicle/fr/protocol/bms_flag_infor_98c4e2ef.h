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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_BMS_FLAG_INFOR_98C4E2EF_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_BMS_FLAG_INFOR_98C4E2EF_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Bmsflaginfor98c4e2ef : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bmsflaginfor98c4e2ef();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'bms_flag_Infor_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bms_flag_infor_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bms_flag_infor_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'reserved', 'offset': 0.0, 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 22, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int reserved(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_low_temperature', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double bms_flag_infor_low_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_hight_temperature', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 28, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double bms_flag_infor_hight_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_charge_flag', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 21, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_charge_flag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_lock_mos', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 20, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_lock_mos(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_ic_error', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 19, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_ic_error(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_short', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 18, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_short(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_discharge_oc', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 17, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_discharge_oc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_charge_oc', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_charge_oc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_discharge_ut', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 15, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_discharge_ut(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_discharge_ot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_discharge_ot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_charge_ut', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_charge_ut(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_charge_ot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_charge_ot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_uv', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 11, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_uv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_ov', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_ov(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_single_uv', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_single_uv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_single_ov', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bms_flag_infor_single_ov(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'bms_flag_Infor_soc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bms_flag_infor_soc(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_BMS_FLAG_INFOR_98C4E2EF_H_
