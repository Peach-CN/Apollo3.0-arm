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

#ifndef MODEULES_GUARDIAN_GUARDIAN_H_
#define MODEULES_GUARDIAN_GUARDIAN_H_

#include <map>
#include <mutex>
#include <queue>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/time/time.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/drivers/proto/ultrasonic_radar.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::guardian
 * @brief apollo::guardian
 */
namespace apollo {
namespace guardian {

using apollo::common::time::Clock;

template<class Dtype>
class UltrasonicSample {
private:
  int current_index_;            // 当前滤波器索引
  uint32_t sample_count_ = 10;  // 当前滤波器采样大小
  std::deque<Dtype> ultrasonic_;// 当前滤波器队列

public:
  explicit UltrasonicSample(int index,uint32_t sample_count = 10) 
    : sample_count_(sample_count) ,current_index_(index){ }

  uint32_t index() { return current_index_; }
  // 追加超声波距离信息
  void append(apollo::drivers::Ultrasonic& ultrasonic_data) {
    if (apollo::common::time::ToSecond(Clock::Now()) - ultrasonic_data.header().timestamp_sec() < 0.1) {
      if(ultrasonic_.size() >= sample_count_) {
        ultrasonic_.pop_front();
      }
      if (current_index_ < ultrasonic_data.ranges_size())
        ultrasonic_.push_back(ultrasonic_data.ranges(current_index_));
    }
  }

  // 通过均值滤波计算当前传感器的距离
  Dtype distance() {
    if (ultrasonic_.size() < 1) return Dtype(0);
    std::deque<double> ultrasonic(ultrasonic_);
    if(ultrasonic.size() >= sample_count_) {
      std::sort(ultrasonic.begin(), ultrasonic.end());
      ultrasonic.pop_front();
      ultrasonic.pop_back();
    }
    return std::accumulate(ultrasonic.begin(), ultrasonic.end(), Dtype(0)) / ultrasonic.size();
  }
};

class Guardian : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  void OnTimer(const ros::TimerEvent&);
  void OnChassis(const apollo::canbus::Chassis& message);
  void OnControl(const apollo::control::ControlCommand& message);
  void OnSystemStatus(const apollo::monitor::SystemStatus& message);
  void OnUltrasonic(const apollo::drivers::Ultrasonic& message);
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  apollo::canbus::Chassis chassis_;
  apollo::monitor::SystemStatus system_status_;
  apollo::control::ControlCommand control_cmd_;
  apollo::guardian::GuardianCommand guardian_cmd_;
  apollo::drivers::Ultrasonic ultrasonic_data_;
  apollo::guardian::UltrasonicRadarConfigs config_;
  std::vector<UltrasonicSample<double>> ultrasonic_samples_;
  std::mutex mutex_;

  ros::Timer timer_;
};

}  // namespace guardian
}  // namespace apollo

#endif  // MODULES_GUARDIAN_GUARDIAN_H_
