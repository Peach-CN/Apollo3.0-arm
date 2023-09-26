/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 */

#include "modules/drivers/radar/ultrasonic_radar/ultrasonic_radar.h"
#include "modules/drivers/radar/ultrasonic_radar/ultrasonic_radar_message_manager.h"
#include "modules/drivers/proto/ultrasonic_radar.pb.h"

/**
 * @namespace apollo::drivers::ultrasonic_radar
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

std::string UltrasonicRadar::Name() const {
  return FLAGS_canbus_driver_name;
}

apollo::common::Status UltrasonicRadar::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);
  AINFO << "The adapter manager is successfully initialized.";
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_sensor_conf_file,
                                                &ultrasonic_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_sensor_conf_file);
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_sensor_conf_file;
  ADEBUG << "Canbus_conf:" << ultrasonic_radar_conf_.ShortDebugString();

  if (ultrasonic_radar_conf_.has_serial_conf()) {
    serial_stream_.reset(new UltrasonicRadarSerialStream(
      ultrasonic_radar_conf_.serial_conf().device().c_str(),
      ultrasonic_radar_conf_.serial_conf().baud_rate()));
    AINFO << "Serial stream is successfully created.";
  } else if (ultrasonic_radar_conf_.has_can_conf()) {
    // Init can client
    auto *can_factory = CanClientFactory::instance();
    can_factory->RegisterCanClients();
    can_client_ = can_factory->CreateCANClient(
        ultrasonic_radar_conf_.can_conf().can_card_parameter());
    if (!can_client_) {
      return OnError("Failed to create can client.");
    }
    AINFO << "Can client is successfully created.";

    sensor_message_manager_ =
        std::unique_ptr<UltrasonicRadarMessageManager>(
            new UltrasonicRadarMessageManager(
                ultrasonic_radar_conf_.entrance_num()));
    if (sensor_message_manager_ == nullptr) {
      return OnError("Failed to create message manager.");
    }
    sensor_message_manager_->set_can_client(can_client_);
    AINFO << "Sensor message manager is successfully created.";

    bool enable_receiver_log =
        ultrasonic_radar_conf_.can_conf().enable_receiver_log();
    if (can_receiver_.Init(can_client_.get(),
                           sensor_message_manager_.get(),
                           enable_receiver_log) !=
        ErrorCode::OK) {
      return OnError("Failed to init can receiver.");
    }
    AINFO << "The can receiver is successfully initialized.";
  } else {
    return OnError("Unknown Config.");
  }
  return Status::OK();
}

apollo::common::Status UltrasonicRadar::Start() {
  if (ultrasonic_radar_conf_.has_can_conf()) {
    // 1. init and start the can card hardware
    if (can_client_->Start() != ErrorCode::OK) {
      return OnError("Failed to start can client");
    }
    AINFO << "Can client is started.";

    // 2. start receive first then send
    if (can_receiver_.Start() != ErrorCode::OK) {
      return OnError("Failed to start can receiver.");
    }
    AINFO << "Can receiver is started.";
  } else if (ultrasonic_radar_conf_.has_serial_conf()) {
    if (!serial_stream_->Connect()) {
      return OnError("Failed to start serial stream");
    }
    const auto& update_func = [this] { SecurityDogThreadFunc(); };
    thread_.reset(new std::thread(update_func));
    AINFO << "Serial stream is started.";
  }
  // last step: publish monitor messages
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Canbus is started.");

  return Status::OK();
}

void UltrasonicRadar::Stop() {
  if (can_client_) {
    can_receiver_.Stop();
    can_client_->Stop();
  }
  if (serial_stream_) {
    serial_stream_->Disconnect();
  }
  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "SecurityDogThread stopped.";
  }
}

void UltrasonicRadar::SecurityDogThreadFunc() {
  if (serial_stream_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because serial_stream_ is "
              "nullptr.";
    return;
  }
  while (!serial_stream_->IsRunning()) {
    AINFO << "waiting for serial stream to be ready";
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0, end = 0;
  int32_t failover_cnt = 0, recv_bytes = 0;
  uint8_t buf[1024];
  ssize_t bytes_read = 0;
  while (serial_stream_->IsRunning()) {
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());

    uint8_t trigger_cmd[] = {0x7F ,0x01 ,0x12 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x03,0x16};
    if (serial_stream_->Write(trigger_cmd, sizeof(trigger_cmd)) <= 0) {
      ++failover_cnt;
      if (failover_cnt > 100) {
        AERROR << "SerialStream FailOver exceed 100 times." "\tTerminate the process.";
        exit(-1);
      }
    } else {
      failover_cnt = 0;
	  bytes_read = serial_stream_->Read(buf + recv_bytes,
                          sizeof(buf) - recv_bytes, false);
      if (bytes_read > 0) {
        //TODO: process recv data
        recv_bytes += bytes_read;
        Parse(buf, recv_bytes);
      }
    }

    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in SerialStream looping process:"
             << elapsed.count();
    }
  }
}

void UltrasonicRadar::PublishSensorData() {
  //AdapterManager::FillUltrasonicHeader(FLAGS_sensor_node_name, &sensor_data_);
  //AdapterManager::PublishUltrasonic(sensor_data_);
}

void UltrasonicRadar::Parse(uint8_t *data, int32_t &length) {
  Ultrasonic sensor_data;
  uint8_t *buf = data;
  //47 61 70 3d 32 30 38 32 6d 6d 0d 0a             	Gap=2082  mm
  sensor_data.mutable_ranges()->Resize(ultrasonic_radar_conf_.entrance_num(), 0.0);
  while (length > 8) {
    if (buf[0] == 'G' && buf[1] == 'a' && buf[2] == 'p') {
      int32_t index = 3;
      while (index < length && buf[index] != 0x0D) ++index;

      if (index >= length) break;

      if (0x0A == buf[index + 1]) {
        const char *number = reinterpret_cast<const char *>(&buf[4]);
        while (!isdigit(*number) && number < reinterpret_cast<const char *>(&buf[index])) ++number;

        if (number < reinterpret_cast<const char *>(&buf[index])) {
          sensor_data.set_ranges(0, atoi(number) / 1000.);
          AdapterManager::FillUltrasonicHeader(FLAGS_sensor_node_name, &sensor_data);
          AdapterManager::PublishUltrasonic(sensor_data);
        }

        buf += index + 2;
        length -= (index + 2);
      } else {
        ++buf; --length;
        continue;
      }
    } else {
      ++buf; --length;
      continue;
    }
  }
  if (buf != data && length > 0)
    memmove(data, buf, length);
}

// Send the error to monitor and return it
Status UltrasonicRadar::OnError(const std::string &error_msg) {
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
