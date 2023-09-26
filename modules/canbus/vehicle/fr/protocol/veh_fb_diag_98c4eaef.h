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

#ifndef MODULES_CANBUS_VEHICLE_FR_PROTOCOL_VEH_FB_DIAG_98C4EAEF_H_
#define MODULES_CANBUS_VEHICLE_FR_PROTOCOL_VEH_FB_DIAG_98C4EAEF_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace fr {

class Vehfbdiag98c4eaef : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vehfbdiag98c4eaef();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Veh_fb_DrvMCUDisCtrl', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcudisctrl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBOilFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 30, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehboilfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBOilPressSensorFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 29, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehboilpresssensorfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBMotorFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 28, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbmotorfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBsensorAbnomal', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 27, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbsensorabnomal(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBPowerFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbpowerfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBOT', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 25, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBAnguleFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbangulefault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBDisEn', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbdisen(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBWorkModelFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 22, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbworkmodelfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EPSOverCurrent', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_epsovercurrent(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EPSDisWork', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_epsdiswork(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EPSWarning', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 11, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_epswarning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_AuxRemoteDisOnline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_auxremotedisonline(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_AuxRemoteClose', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 46, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_auxremoteclose(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_AUXBMSDisOnline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 44, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_auxbmsdisonline(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUDisOnline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcudisonline(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_AutoIOCANCmd', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 5, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_autoiocancmd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_AutoCANCtrlCmd', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 4, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_autocanctrlcmd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_Infor_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int veh_fb_infor_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_Infor_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int veh_fb_infor_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_AuxScram', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 45, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_auxscram(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUMOSFEF', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcumosfef(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUHall', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 38, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcuhall(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUScram', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 37, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcuscram(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUShort', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcushort(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUUV', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 35, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcuuv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUOV', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 34, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcuov(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_DrvMCUOT', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_drvmcuot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBDisOnline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 21, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbdisonline(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EHBecuFault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 20, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_ehbecufault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EPSMosfetOT', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_epsmosfetot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EPSfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_epsfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_EPSDisOnline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool veh_fb_epsdisonline(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Veh_fb_FaultLevel', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int veh_fb_faultlevel(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace fr
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_FR_PROTOCOL_VEH_FB_DIAG_98C4EAEF_H_
