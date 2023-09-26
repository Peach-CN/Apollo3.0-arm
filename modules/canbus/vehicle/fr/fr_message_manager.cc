/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/fr/fr_message_manager.h"

#include "modules/canbus/vehicle/fr/protocol/ctrl_cmd_98c4d2d0.h"
#include "modules/canbus/vehicle/fr/protocol/io_cmd_98c4d7d0.h"
#include "modules/canbus/vehicle/fr/protocol/sensor_reset_98ffffff.h"

#include "modules/canbus/vehicle/fr/protocol/bms_flag_infor_98c4e2ef.h"
#include "modules/canbus/vehicle/fr/protocol/bms_infor_98c4e1ef.h"
#include "modules/canbus/vehicle/fr/protocol/ctrl_fb_98c4d2ef.h"
#include "modules/canbus/vehicle/fr/protocol/drive_fb_mcuecoder_98c4dcef.h"
#include "modules/canbus/vehicle/fr/protocol/error_70f.h"
#include "modules/canbus/vehicle/fr/protocol/io_fb_98c4daef.h"
#include "modules/canbus/vehicle/fr/protocol/lr_wheel_fb_98c4d7ef.h"
#include "modules/canbus/vehicle/fr/protocol/odo_fb_98c4deef.h"
#include "modules/canbus/vehicle/fr/protocol/rr_wheel_fb_98c4d8ef.h"
#include "modules/canbus/vehicle/fr/protocol/veh_fb_diag_98c4eaef.h"

namespace apollo {
namespace canbus {
namespace fr {

FrMessageManager::FrMessageManager() {
  // Control Messages
  AddSendProtocolData<Ctrlcmd98c4d2d0, true>();
  AddSendProtocolData<Iocmd98c4d7d0, true>();
  AddSendProtocolData<Sensorreset98ffffff, true>();

  // Report Messages
  AddRecvProtocolData<Bmsflaginfor98c4e2ef, true>();
  AddRecvProtocolData<Bmsinfor98c4e1ef, true>();
  AddRecvProtocolData<Ctrlfb98c4d2ef, true>();
  AddRecvProtocolData<Drivefbmcuecoder98c4dcef, true>();
  AddRecvProtocolData<Error70f, true>();
  AddRecvProtocolData<Iofb98c4daef, true>();
  AddRecvProtocolData<Lrwheelfb98c4d7ef, true>();
  AddRecvProtocolData<Odofb98c4deef, true>();
  AddRecvProtocolData<Rrwheelfb98c4d8ef, true>();
  AddRecvProtocolData<Vehfbdiag98c4eaef, true>();
}

FrMessageManager::~FrMessageManager() {}

}  // namespace fr
}  // namespace canbus
}  // namespace apollo
