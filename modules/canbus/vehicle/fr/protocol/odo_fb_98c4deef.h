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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_ODO_FB_98C4DEEF_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_ODO_FB_98C4DEEF_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Odofb98c4deef : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Odofb98c4deef();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'odo_fb_accumulative_angular', 'offset': 0.0, 'precision': 0.001, 'len': 32, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double odo_fb_accumulative_angular(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'odo_fb_accumulative_mileage', 'offset': 0.0, 'precision': 0.001, 'len': 32, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double odo_fb_accumulative_mileage(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_ODO_FB_98C4DEEF_H_
