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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_CTRL_FB_98C4D2EF_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_CTRL_FB_98C4D2EF_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Ctrlfb98c4d2ef : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ctrlfb98c4d2ef();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ctrl_fb_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int ctrl_fb_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ctrl_fb_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int ctrl_fb_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ctrl_fb_mode', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int ctrl_fb_mode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ctrl_fb_Brake', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int ctrl_fb_brake(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ctrl_fb_steering', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 20, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double ctrl_fb_steering(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ctrl_fb_velocity', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 4, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double ctrl_fb_velocity(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ctrl_fb_gear', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int ctrl_fb_gear(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_CTRL_FB_98C4D2EF_H_
