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
#include "modules/guardian/guardian.h"

#include <cmath>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/guardian/common/guardian_gflags.h"
#include "ros/include/ros/ros.h"
#include <queue>

namespace apollo {
namespace guardian {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::control::ControlCommand;
using apollo::guardian::GuardianCommand;
using apollo::monitor::SystemStatus;
using apollo::common::time::Clock;

std::string Guardian::Name() const { return FLAGS_module_name; }

Status Guardian::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  CHECK(AdapterManager::GetSystemStatus())
      << "SystemStatus is not initialized.";
  CHECK(AdapterManager::GetControlCommand()) << "Control is not initialized.";
  CHECK(AdapterManager::GetUltrasonic()) << "Ultrasonic is not initialized.";
  return Status::OK();
}

Status Guardian::Start() {
  if (apollo::common::util::GetProtoFromASCIIFile(
           FLAGS_ultrasonic_radar_config_filename, &config_)) {
    for (int i = 0; i < config_.config_size(); i++) {
      if (config_.config(i).direction() != UltrasonicRadarConfig::INVALID) {
        UltrasonicSample<double> ultrasonic_sample(config_.config(i).index(), 30);//30个采样
        ultrasonic_samples_.push_back(ultrasonic_sample);
      }
    }
  }
  AdapterManager::AddChassisCallback(&Guardian::OnChassis, this);
  AdapterManager::AddSystemStatusCallback(&Guardian::OnSystemStatus, this);
  AdapterManager::AddControlCommandCallback(&Guardian::OnControl, this);
  AdapterManager::AddUltrasonicCallback(&Guardian::OnUltrasonic, this);
  const double duration = 1.0 / FLAGS_guardian_cmd_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &Guardian::OnTimer, this);

  return Status::OK();
}

void Guardian::Stop() { timer_.stop(); }

void Guardian::OnTimer(const ros::TimerEvent&) {
  //ADEBUG << "Timer is triggered: publish Guardian result";
  bool safety_mode_triggered = false;
  if (FLAGS_guardian_enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    safety_mode_triggered = system_status_.has_safety_mode_trigger_time();
  }
  if (std::fabs(ros::Time::now().toSec() - ultrasonic_data_.header().timestamp_sec()) < 0.5) {
    for (int i = 0; i < ultrasonic_samples_.size(); i++) {
      ultrasonic_samples_[i].append(ultrasonic_data_);
    }

    for (int i = 0; i < ultrasonic_samples_.size(); i++) {
      for (auto& config : config_.config()) {
        if (config.index() == ultrasonic_samples_[i].index() && \
            ultrasonic_samples_[i].distance() < config.distance()) {
          safety_mode_triggered = true;
        }
      }
    }
  }
  if (safety_mode_triggered) {
    ADEBUG << "Safety mode triggered, enable safty mode";
    TriggerSafetyMode();
  } else {
    ADEBUG << "Safety mode not triggered, bypass control command";
    PassThroughControlCommand();
  }

  AdapterManager::FillGuardianHeader(FLAGS_node_name, &guardian_cmd_);
  AdapterManager::PublishGuardian(guardian_cmd_);
}

void Guardian::OnChassis(const Chassis& message) {
  //ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  chassis_.CopyFrom(message);
}

void Guardian::OnSystemStatus(const SystemStatus& message) {
  //ADEBUG << "Received monitor data: run monitor callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  system_status_.CopyFrom(message);
}

void Guardian::OnControl(const ControlCommand& message) {
  //ADEBUG << "Received control data: run control command callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  control_cmd_.CopyFrom(message);
}

void Guardian::OnUltrasonic(const apollo::drivers::Ultrasonic& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  ultrasonic_data_.CopyFrom(message);
}

void Guardian::PassThroughControlCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  guardian_cmd_.mutable_control_command()->CopyFrom(control_cmd_);
}

void Guardian::TriggerSafetyMode() {
  AINFO << "Safety state triggered, with system safety mode trigger time : "
        << system_status_.safety_mode_trigger_time();
  std::lock_guard<std::mutex> lock(mutex_);
  bool sensor_malfunction = false, obstacle_detected = false;
  if (!chassis_.surround().sonar_enabled() ||
      chassis_.surround().sonar_fault()) {
    AINFO << "Ultrasonic sensor not enabled for faulted, will do emergency "
             "stop!";
    sensor_malfunction = true;
  } else {
    // TODO(QiL) : Load for config
    for (int i = 0; i < chassis_.surround().sonar_range_size(); ++i) {
      if ((chassis_.surround().sonar_range(i) > 0.0 &&
           chassis_.surround().sonar_range(i) < 2.5) ||
          chassis_.surround().sonar_range(i) > 30) {
        AINFO << "Object detected or ultrasonic sensor fault output, will do "
                 "emergency stop!";
        obstacle_detected = true;
      }
    }
  }
#if 1
  double throttle_break_value = 0.0;
  if(chassis_.throttle_percentage() > 1)
    throttle_break_value = -chassis_.throttle_percentage();
  else
    throttle_break_value = 0.0;
  guardian_cmd_.mutable_control_command()->set_throttle(throttle_break_value);
#else
  guardian_cmd_.mutable_control_command()->set_throttle(0.0);
#endif
  guardian_cmd_.mutable_control_command()->set_steering_target(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_rate(25.0);
  guardian_cmd_.mutable_control_command()->set_is_in_safe_mode(true);

  // TODO(QiL) : Remove this one once hardware re-alignment is done.
  sensor_malfunction = false;
  obstacle_detected = false;
  AINFO << "Temporarily ignore the ultrasonic sensor output during hardware "
           "re-alignment!";

  if (system_status_.require_emergency_stop() || sensor_malfunction ||
      obstacle_detected) {
    AINFO << "Emergency stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        FLAGS_guardian_cmd_emergency_stop_percentage);
  } else {
    AINFO << "Soft stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        FLAGS_guardian_cmd_soft_stop_percentage);
  }
}

}  // namespace guardian
}  // namespace apollo
