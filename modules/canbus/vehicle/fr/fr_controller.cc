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

#include "modules/canbus/vehicle/fr/fr_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "modules/canbus/vehicle/fr/fr_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

DEFINE_double(fr_max_speed, 0.7, "Ratio of steering target");

namespace apollo {
namespace canbus {
namespace fr {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
const double PI = 3.1415926;
}

ErrorCode FrController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "FrController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }
  vehicle_params_.CopyFrom(
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  ctrl_cmd_98c4d2d0_ = dynamic_cast<Ctrlcmd98c4d2d0*>(
      message_manager_->GetMutableProtocolDataById(Ctrlcmd98c4d2d0::ID));
  if (ctrl_cmd_98c4d2d0_ == nullptr) {
    AERROR
        << "ctrl_cmd_98c4d2d0_ does not exist in the LotusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Ctrlcmd98c4d2d0::ID, ctrl_cmd_98c4d2d0_,
                          false);

  // need sleep to ensure all messages received
  AINFO << "FrController is initialized.";

  EnableAutoMode();

  is_initialized_ = true;
  return ErrorCode::OK;
}

FrController::~FrController() {}

bool FrController::Start() {
  if (!is_initialized_) {
    AERROR << "FrController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void FrController::Stop() {
  if (!is_initialized_) {
    AERROR << "LotusController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "LotusController stopped.";
  }
}

Chassis FrController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  AINFO << "Driving mode " << driving_mode();

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // 5
  if (chassis_detail.fr().has_ctrl_fb_98c4d2ef() &&
      chassis_detail.fr().ctrl_fb_98c4d2ef().has_ctrl_fb_velocity()) {
    chassis_.set_speed_mps(
        chassis_detail.fr().ctrl_fb_98c4d2ef().ctrl_fb_velocity());
  } else {
    chassis_.set_speed_mps(0);
  }

  // 7
  chassis_.set_fuel_range_m(0);
  // 8
  chassis_.set_throttle_percentage(chassis_.speed_mps() / 5.1);

  // 9
  if (chassis_detail.fr().has_ctrl_fb_98c4d2ef() &&
      chassis_detail.fr().ctrl_fb_98c4d2ef().has_ctrl_fb_brake()) {
    chassis_.set_brake_percentage(
        chassis_detail.fr().ctrl_fb_98c4d2ef().ctrl_fb_brake());
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 23, previously 10
  if (chassis_detail.fr().has_ctrl_fb_98c4d2ef() &&
      chassis_detail.fr().ctrl_fb_98c4d2ef().has_ctrl_fb_gear()) {
    switch (
        chassis_detail.fr().ctrl_fb_98c4d2ef().ctrl_fb_gear()) {
      case 3:
        chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
        break;

      case 2:
        chassis_.set_gear_location(Chassis::GEAR_DRIVE);
        break;

      case 4:
        chassis_.set_gear_location(Chassis::GEAR_DRIVE);
        break;

      default:
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
        break;
    }
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }

  // 11
  // TODO(QiL) : verify the unit here.
  if (chassis_detail.fr().has_ctrl_fb_98c4d2ef() &&
      chassis_detail.fr()
          .ctrl_fb_98c4d2ef()
          .has_ctrl_fb_steering()) {
    chassis_.set_steering_percentage(
       100.0 *  (chassis_detail.fr().ctrl_fb_98c4d2ef().ctrl_fb_steering() * PI /180 )
        / vehicle_params_.max_steer_angle() );
  } else {
    chassis_.set_steering_percentage(0);
  }


  // TODO(all): implement the rest here/
  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  // give engage_advice based on error_code and canbus feedback
  chassis_.mutable_engage_advice()->set_advice(
      apollo::common::EngageAdvice::READY_TO_ENGAGE);

  return chassis_;
}

void FrController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode FrController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  /*auto_brakingcmd_98c4d4d0_->set_auto_brakingcmd_brakingenable(true);
  auto_drivecmd_98c4d3d0_->set_auto_drivecmd_driveenable(true);
  auto_steeringcmd_98c4d2d0_->set_auto_steeringcmd_steeringenable(true);
  auto_gearcmd_98c4d1d0_->set_auto_gearcmd_gearenable(true);
  auto_parkingcmd_98c4d5d0_->set_auto_parkingcmd_parkingenable(true);*/

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode FrController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode FrController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }

  /*auto_brakingcmd_98c4d4d0_->set_auto_brakingcmd_brakingenable(false);
  auto_drivecmd_98c4d3d0_->set_auto_drivecmd_driveenable(false);
  auto_steeringcmd_98c4d2d0_->set_auto_steeringcmd_steeringenable(false);*/

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode FrController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }

  /*auto_brakingcmd_98c4d4d0_->set_auto_brakingcmd_brakingenable(true);
  auto_drivecmd_98c4d3d0_->set_auto_drivecmd_driveenable(true);
  auto_steeringcmd_98c4d2d0_->set_auto_steeringcmd_steeringenable(true);*/

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

// NEUTRAL, REVERSE, DRIVE
void FrController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }

  AINFO << "FrController::Gear_" << gear_position;

  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      ctrl_cmd_98c4d2d0_->set_ctrl_cmd_gear(3);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      ctrl_cmd_98c4d2d0_->set_ctrl_cmd_gear(2);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      ctrl_cmd_98c4d2d0_->set_ctrl_cmd_gear(4);
      break;
    }
    case Chassis::GEAR_PARKING: {
      ctrl_cmd_98c4d2d0_->set_ctrl_cmd_gear(1);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!" << gear_position;
      ctrl_cmd_98c4d2d0_->set_ctrl_cmd_gear(0);
      break;
    }
    default: {
      ctrl_cmd_98c4d2d0_->set_ctrl_cmd_gear(0);
      break;
    }
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void FrController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO Update brake value based on mode
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }

  AERROR << "Brake is " << pedal;
  if (pedal > 0.1) ctrl_cmd_98c4d2d0_->set_ctrl_cmd_brake(true);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void FrController::Throttle(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }

  //pedal = pedal * 5.0 / 100;
  pedal = pedal / 100.0 * FLAGS_fr_max_speed;

  ctrl_cmd_98c4d2d0_->set_ctrl_cmd_velocity(pedal);
}

void FrController::Acceleration(double acc) {}

// lotus default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void FrController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  Brake(0);
  /*const double real_angle =
      vehicle_params_.max_steer_angle() * angle / 100.0 * 180 / PI;*/
  const double real_angle =
      (vehicle_params_.max_steer_angle() * angle/100.0 ) * 180 / PI;
  ctrl_cmd_98c4d2d0_->set_ctrl_cmd_steering(real_angle);
  AERROR << " Steer is " << real_angle;
  AERROR << "Angle is " << angle;
  AERROR << " max agnle is "<< vehicle_params_.max_steer_angle();
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void FrController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  Brake(0);
  const double real_angle =
      (vehicle_params_.max_steer_angle() * angle / 100.0) *180 / PI;
  const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate(),
          vehicle_params_.max_steer_angle_rate(),
          vehicle_params_.max_steer_angle_rate() * angle_spd / 100.0);
  ctrl_cmd_98c4d2d0_->set_ctrl_cmd_steering(real_angle);
}

void FrController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    ctrl_cmd_98c4d2d0_->set_ctrl_cmd_brake(true);
  } else {
    ctrl_cmd_98c4d2d0_->set_ctrl_cmd_brake(false);
  }
}

void FrController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void FrController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void FrController::SetTurningSignal(const ControlCommand& command) {
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    // auto_lightcmd_98c4d7d0_->set_auto_leftlightcmd_leftlighten(true);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    // auto_lightcmd_98c4d7d0_->set_auto_rightlightcmd_rightlighten(true);
  } else {
    // auto_lightcmd_98c4d7d0_->set_auto_leftlightcmd_leftlighten(false);
    // auto_lightcmd_98c4d7d0_->set_auto_rightlightcmd_rightlighten(false);
  }
}

void FrController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool FrController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void FrController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in LotusController looping process:"
             << elapsed.count();
    }
  }
}

bool FrController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return true;
}

void FrController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t FrController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode FrController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void FrController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace lotus
}  // namespace canbus
}  // namespace apollo
