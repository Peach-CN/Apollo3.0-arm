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
 * @file ultrasonic_radar_serial_stream.h
 * @brief The class of UltrasonicRadarSerialStream
 */
#ifndef MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_SERIAL_STREAM_H_
#define MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_SERIAL_STREAM_H_

#include <string>
#include <termios.h>
#include <unistd.h>

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

class UltrasonicRadarSerialStream {
private:
  enum class Status {
    DISCONNECTED,
    CONNECTED,
    ERROR,
  };

public:
  explicit UltrasonicRadarSerialStream(const char* device_name, uint32_t baud_rate);
  ~UltrasonicRadarSerialStream() { Close();}
  
  bool Connect();
  bool Disconnect();
  size_t Read(uint8_t* buffer, size_t max_length, bool wait = true);
  size_t Write(const uint8_t* data, size_t length);
  bool IsRunning();

private:
  void Open();
  void Close();
  bool ConfigurePort(int fd);
  bool WaitReadable(uint32_t timeout_us);
  bool WaitWritable(uint32_t timeout_us);
  void CheckRemove();

private:
  std::string device_name_;
  uint32_t baud_rate_;
  uint32_t bytesize_;
  uint32_t parity_;
  uint32_t stopbits_;
  uint32_t flowcontrol_;
  uint32_t byte_time_us_;
  Status status_ = Status::DISCONNECTED;
  int fd_;
  int errno_;
  bool is_open_;
};

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_SERIAL_STREAM_H_
