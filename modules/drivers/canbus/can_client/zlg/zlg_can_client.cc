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

/**
 * @file zlg_can_client.cc
 * @brief the encapsulate call the api of zlg can card according to can_client.h
 *interface
 **/

#include "modules/drivers/canbus/can_client/zlg/zlg_can_client.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool ZlgCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();

  socket_can_.reset(new SocketCanClientRaw());
  CANCardParameter param = parameter;
  param.set_channel_id(CANCardParameter::CHANNEL_ID_THREE);
  socket_can_->Init(param);

  return true;
}

ZlgCanClient::~ZlgCanClient() {
    Stop();
}

ErrorCode ZlgCanClient::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }
  {
    VCI_BOARD_INFO info[50];
    int num_device = VCI_FindUsbDevice2(info);
    AINFO << "USBCAN DEVICE NUM: [" <<  num_device << "] PCS.";

    for(int index = 0; index < num_device; index++) {
      AINFO << "Device: " << index;
      AINFO << "Get VCI_ReadBoardInfo success!";
      info[index].str_Serial_Num[19] = '\0';
      AINFO << "\tSerial_Num: " << info[index].str_Serial_Num;
      info[index].str_hw_Type[10] = '\0';
      AINFO << "\thw_Type: " << info[index].str_hw_Type;
      std::stringstream ss;
      ss << "Firmware Version:V" << std::hex << ((info[index].fw_Version & 0x0F00) >> 8)
         << "." << std::hex << ((info[index].fw_Version & 0x00F0) >> 4)
         << std::hex << (info[index].fw_Version & 0x0F);
      AINFO << "\t" << ss.str();
    }
  }
  // open device
  // guss net is the device minor number, if one card is 0,1
  // if more than one card, when install driver u can specify the minior id
  if (port_ > ZLG_MAXIMUM_CHANNEL_NUM || port_ < 0) {
    AERROR << "can port number [" << port_ << "] is out of the range [0,"
           << ZLG_MAXIMUM_CHANNEL_NUM << "]";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  int32_t ret = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
  if (ret != ZLGCAN_SUCCESS) {
    AERROR << "open device error code [" << ret << "]: " << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // Read device serial numner and version info 
  VCI_BOARD_INFO info = {0};
  if(ZLGCAN_SUCCESS == VCI_ReadBoardInfo(VCI_USBCAN2, 0, &info)) {
    AINFO << "\tGet VCI_ReadBoardInfo success!";

    info.str_Serial_Num[19] = '\0';
    AINFO << "\tSerial_Num: " << info.str_Serial_Num;
    info.str_hw_Type[10] = '\0';
    AINFO << "\thw_Type: " << info.str_hw_Type;
    AINFO << "\tFirmware Version:V" << std::hex
          << ((info.fw_Version & 0x0F00) >> 8) << "." << std::hex
          << ((info.fw_Version & 0x00F0) >> 4) << std::hex
          << (info.fw_Version & 0x0F);
  }
  // init config and state
  // After a CAN handle is created with canOpen() the CAN-ID filter is
  // cleared
  // (no CAN messages
  // will pass the filter). To receive a CAN message with a certain CAN-ID
  // or an
  // NTCAN-Event with
  // a certain Event-ID it is required to enable this ID in the handle
  // filter as
  // otherwise a
  // received  message or event is discarded by the driver for this handle.
  // 1. set receive message_id filter, ie white list
  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xFFFFFFFF;
  config.Filter = 1;  // 0~3
  config.Timing0 = 0x00;  // CAN_BAUDRATE_500K
  config.Timing1 = 0x1C;
  config.Mode = 0;
	
  if(ZLGCAN_SUCCESS != VCI_InitCAN(VCI_USBCAN2, 0, port_, &config)) {
    AERROR << "Init CAN1 Error";
    VCI_CloseDevice(VCI_USBCAN2, 0);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  if(ZLGCAN_SUCCESS != VCI_StartCAN(VCI_USBCAN2, 0, port_)) {
    AERROR << "Start CAN1 Error";
    VCI_CloseDevice(VCI_USBCAN2, 0);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  socket_can_->Start();
  is_started_ = true;
  return ErrorCode::OK;
}

void ZlgCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    if (socket_can_)
      socket_can_->Stop();
    // VCI_ResetCAN(VCI_USBCAN2, 0, port_);// Reset CAN Channel
    int32_t ret = VCI_CloseDevice(VCI_USBCAN2, 0);
    if (ret != ZLGCAN_SUCCESS) {
      AERROR << "close error code:" << ret;
    } else {
      AINFO << "close zlg can ok. port:" << port_;
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode ZlgCanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_started_) {
    AERROR << "Zlg can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
    send_frames_[i].SendType = 0;
    send_frames_[i].RemoteFlag = 0; // 0: Data; 1: Remote
    send_frames_[i].ExternFlag = (frames[i].id & 0x80000000) ? 1 : 0; // 0: Standard; 1: Extend
    send_frames_[i].ID = frames[i].id & 0x1FFFFFFF;
    send_frames_[i].DataLen = frames[i].len;
    std::memcpy(send_frames_[i].Data, frames[i].data, frames[i].len);
  }

  // Synchronous transmission of CAN messages
  int32_t ret = VCI_Transmit(VCI_USBCAN2, 0, port_, send_frames_, *frame_num);
  if (ret != ZLGCAN_SUCCESS) {
    AERROR << "send message failed, error code: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode ZlgCanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "Zlg can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }
#if 0
  {
    int32_t vframe_num = 1;
    std::vector<CanFrame> vframes;

    if (socket_can_ && ErrorCode::OK == socket_can_->Receive(&vframes, &vframe_num)) {
      if (vframes.size()) {
        Send(vframes, &vframe_num);
      }
    }
  }
#endif
  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  do {
    int32_t ret = VCI_Receive(VCI_USBCAN2, 0, port_, recv_frames_, MAX_CAN_RECV_FRAME_LEN - frames->size(), 100);
    if (ret > 0) {
      for (int32_t i = 0; i < ret && i < *frame_num; ++i) {
        CanFrame cf;
        cf.id = (recv_frames_[i].ExternFlag & 0x1) ? recv_frames_[i].ID | 0x80000000 : recv_frames_[i].ID;
        cf.len = recv_frames_[i].DataLen;
        std::memcpy(cf.data, recv_frames_[i].Data, recv_frames_[i].DataLen);
        frames->push_back(cf);
        //AERROR << "[ZlgCanClient::Receive] Extended = " << (int)recv_frames_[i].ExternFlag << "\tID = " << std::hex << cf.id;
      }
    }
  } while(frames->size() < MAX_CAN_RECV_FRAME_LEN);

  if (socket_can_) {
    int32_t number = static_cast<int32_t>(frames->size());
    socket_can_->Send(*frames, &number);
  }

  return ErrorCode::OK;
}

/************************************************************************/
/************************************************************************/
/* Function: GetErrorString()                                            */
/* Return ASCII representation of NTCAN return code                     */
/************************************************************************/
/************************************************************************/

std::string ZlgCanClient::GetErrorString(const int32_t status) {
  if (ZLGCAN_SUCCESS == status)
      return std::string("NTCAN_SUCCESS");
  else
    return std::string("Unknown Error");
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
