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

#include "modules/canbus/vehicle/fr/protocol/veh_fb_diag_98c4eaef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::Byte;

Vehfbdiag98c4eaef::Vehfbdiag98c4eaef() {}
const int32_t Vehfbdiag98c4eaef::ID = 0x98C4EAEF;

void Vehfbdiag98c4eaef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcudisctrl(veh_fb_drvmcudisctrl(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehboilfault(veh_fb_ehboilfault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehboilpresssensorfault(veh_fb_ehboilpresssensorfault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbmotorfault(veh_fb_ehbmotorfault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbsensorabnomal(veh_fb_ehbsensorabnomal(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbpowerfault(veh_fb_ehbpowerfault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbot(veh_fb_ehbot(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbangulefault(veh_fb_ehbangulefault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbdisen(veh_fb_ehbdisen(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbworkmodelfault(veh_fb_ehbworkmodelfault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_epsovercurrent(veh_fb_epsovercurrent(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_epsdiswork(veh_fb_epsdiswork(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_epswarning(veh_fb_epswarning(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_auxremotedisonline(veh_fb_auxremotedisonline(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_auxremoteclose(veh_fb_auxremoteclose(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_auxbmsdisonline(veh_fb_auxbmsdisonline(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcudisonline(veh_fb_drvmcudisonline(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_autoiocancmd(veh_fb_autoiocancmd(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_autocanctrlcmd(veh_fb_autocanctrlcmd(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_infor_check_bcc(veh_fb_infor_check_bcc(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_infor_alive_cnt(veh_fb_infor_alive_cnt(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_auxscram(veh_fb_auxscram(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcumosfef(veh_fb_drvmcumosfef(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcuhall(veh_fb_drvmcuhall(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcuscram(veh_fb_drvmcuscram(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcushort(veh_fb_drvmcushort(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcuuv(veh_fb_drvmcuuv(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcuov(veh_fb_drvmcuov(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_drvmcuot(veh_fb_drvmcuot(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbdisonline(veh_fb_ehbdisonline(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_ehbecufault(veh_fb_ehbecufault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_epsmosfetot(veh_fb_epsmosfetot(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_epsfault(veh_fb_epsfault(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_epsdisonline(veh_fb_epsdisonline(bytes, length));
  chassis->mutable_fr()->mutable_veh_fb_diag_98c4eaef()->set_veh_fb_faultlevel(veh_fb_faultlevel(bytes, length));
}

// config detail: {'name': 'veh_fb_drvmcudisctrl', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcudisctrl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehboilfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 30, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehboilfault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehboilpresssensorfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 29, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehboilpresssensorfault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbmotorfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 28, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbmotorfault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbsensorabnomal', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 27, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbsensorabnomal(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbpowerfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbpowerfault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 25, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbangulefault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbangulefault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbdisen', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbdisen(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbworkmodelfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 22, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbworkmodelfault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_epsovercurrent', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_epsovercurrent(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_epsdiswork', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_epsdiswork(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_epswarning', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 11, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_epswarning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_auxremotedisonline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_auxremotedisonline(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_auxremoteclose', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 46, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_auxremoteclose(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_auxbmsdisonline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 44, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_auxbmsdisonline(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcudisonline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcudisonline(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_autoiocancmd', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 5, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_autoiocancmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_autocanctrlcmd', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 4, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_autocanctrlcmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_infor_check_bcc', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vehfbdiag98c4eaef::veh_fb_infor_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_infor_alive_cnt', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vehfbdiag98c4eaef::veh_fb_infor_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_auxscram', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 45, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_auxscram(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcumosfef', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcumosfef(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcuhall', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 38, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcuhall(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcuscram', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 37, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcuscram(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcushort', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcushort(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcuuv', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 35, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcuuv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcuov', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 34, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcuov(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_drvmcuot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_drvmcuot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbdisonline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 21, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbdisonline(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_ehbecufault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 20, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_ehbecufault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_epsmosfetot', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 10, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_epsmosfetot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_epsfault', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_epsfault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_epsdisonline', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Vehfbdiag98c4eaef::veh_fb_epsdisonline(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_fb_faultlevel', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Vehfbdiag98c4eaef::veh_fb_faultlevel(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}
}  // namespace fr
}  // namespace canbus
}  // namespace apollo
