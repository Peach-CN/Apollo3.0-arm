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
 * @file
 * @brief Defines the ZlgCanClient class which inherits CanClient.
 */

#ifndef MODULES_DRIVERS_CANBUS_CAN_CLIENT_CLIENT_ZLG_CAN_CLIENT_H_
#define MODULES_DRIVERS_CANBUS_CAN_CLIENT_CLIENT_ZLG_CAN_CLIENT_H_

#include <string>
#include <vector>
#include <mutex>

#include "modules/drivers/canbus/can_client/zlg/controlcan.h"
#include "gflags/gflags.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"

/**
 * @namespace apollo::drivers::canbus::can
 * @brief apollo::drivers::canbus::can
 */
namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

class SocketCanClientRaw;

/**
 * @class ZlgCanClient
 * @brief The class which defines a ZLG CAN client which inherits CanClient.
 */
class ZlgCanClient : public CanClient {
  enum {
    ZLGCAN_ERROR = 0,
    ZLGCAN_SUCCESS = 1,
    ZLG_MAXIMUM_CHANNEL_NUM = 2,
  };

  enum {
    CAN_BAUDRATE_10K = 0x311C,
    CAN_BAUDRATE_20K = 0x181C,
    CAN_BAUDRATE_40K = 0x87FF,
    CAN_BAUDRATE_50K = 0x091C,
    CAN_BAUDRATE_80K = 0x83FF,
    CAN_BAUDRATE_100K = 0x041C,
    CAN_BAUDRATE_125K = 0x031C,
    CAN_BAUDRATE_200K = 0x81FA,
    CAN_BAUDRATE_250K = 0x011C,
    CAN_BAUDRATE_400K = 0x80FA,
    CAN_BAUDRATE_500K = 0x001C,
    CAN_BAUDRATE_666K = 0x80B6,
    CAN_BAUDRATE_800K = 0x0016,
    CAN_BAUDRATE_1000K = 0x0014,
  };

 public:
  /**
   * @brief Initialize the ZLG CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter &parameter) override;

  /**
   * @brief Destructor
   */
  virtual ~ZlgCanClient();

  /**
   * @brief Start the ZLG CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Start() override;

  /**
   * @brief Stop the ZLG CAN client.
   */
  void Stop() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Send(const std::vector<CanFrame> &frames,
                                 int32_t *const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame> *const frames,
                                    int32_t *const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

 private:
  CANCardParameter::CANChannelId port_;
  VCI_CAN_OBJ send_frames_[MAX_CAN_SEND_FRAME_LEN];
  VCI_CAN_OBJ recv_frames_[MAX_CAN_RECV_FRAME_LEN];
  std::mutex mutex_;
  std::unique_ptr<SocketCanClientRaw> socket_can_ = nullptr;
};

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_CANBUS_CAN_CLIENT_CLIENT_ZLG_CAN_CLIENT_H_
