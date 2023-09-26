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

#include "modules/dreamview/backend/hmi/hmi.h"

#include <cstdlib>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>
#include <chrono>

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/file.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/hmi/hmi_worker.h"
#include "modules/dreamview/proto/audio_capture.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/dreamview/backend/hmi/gnss_setting.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/guardian/common/guardian_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::VehicleConfigHelper;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::common::util::JsonUtil;
using apollo::common::util::DirectoryExists;
using apollo::common::util::PathExists;
using apollo::common::util::EnsureDirectory;
using Json = WebSocketHandler::Json;
using RLock = boost::shared_lock<boost::shared_mutex>;

HMI::HMI(WebSocketHandler *websocket, MapService *map_service)
    : websocket_(websocket),
      map_service_(map_service),
      logger_(apollo::common::monitor::MonitorMessageItem::HMI) {
  // Register websocket message handlers.
  if (websocket_) {
    RegisterMessageHandlers();
    StartBroadcastHMIStatusThread();
  }
  map_gen_.reset(new MapGenerator());
  if (map_gen_)
    map_gen_->Start();
}

void HMI::RegisterMessageHandlers() {
  // Send current config and status to new HMI client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        const auto &config = HMIWorker::instance()->GetConfig();
        websocket_->SendData(
            conn, JsonUtil::ProtoToTypedJson("HMIConfig", config).dump());
        websocket_->SendData(
            conn, JsonUtil::ProtoToTypedJson("HMIStatus",
                                             HMIWorker::instance()->GetStatus())
                      .dump());

        SendVehicleParam(conn);
        SendGnssParam(conn);
        SendRadarParam(conn);
      });

  // HMI client sends audio data, publish to AudioCapture topic.
  websocket_->RegisterMessageHandler(
      "AudioPiece",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {data: "<base64 encoded audio/wav piece>"}.
        std::string data;
        if (JsonUtil::GetStringFromJson(json, "data", &data)) {
          AudioCapture audio;
          audio.set_connection_id(reinterpret_cast<uint64_t>(conn));
          audio.set_wav_stream(apollo::common::util::DecodeBase64(data));
          AdapterManager::PublishAudioCapture(audio);
        } else {
          AERROR << "Truncated audio piece.";
        }
      });

  websocket_->RegisterMessageHandler(
	  "VehicleSettingCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string result;
        auto iter = json.find("value");
        if (iter != json.end()) {
          auto value = *iter;
	  std::map<std::string, double> mvalues;
	  std::vector<std::string> vkeys {
              "front_edge_to_center", "back_edge_to_center",
              "left_edge_to_center",  "right_edge_to_center",
              "min_turn_radius",      "max_steer_angle",
              "wheel_base",           "maximum_speed"
          };

          for (auto key : vkeys) {
            std::string key_value;
            if (JsonUtil::GetStringFromJson(value, key, &key_value)) {
              AERROR << "[VehicleSettingCommand] key = " << key << ", value = " << key_value;
              if (!key_value.empty()) {
                mvalues[key] = std::strtod(key_value.c_str(), NULL);
              } else {
                return ;
              }
              const float EPSINON = 0.00001;
              if (mvalues[key] <= EPSINON) return ;
            } else {
              result = "{\"type\": \"VehicleSettingCommand\", \"result\": \"ERROR\", \"reason\": \"parameter is empty\"}";
              websocket_->SendData(conn, result.c_str());
              AERROR << "Malformed json string when running VehicleSettingCommand.";
              return ;
            }
	  }

          //--vehicle_config_path=/apollo/modules/common/data/microcar_config.pb.txt
          apollo::common::VehicleConfig params;
          if (!apollo::common::util::GetProtoFromFile(FLAGS_vehicle_config_path, &params)) {
            AERROR << "Unable to parse vehicle config file " << FLAGS_vehicle_config_path;
            result = "{\"type\": \"VehicleSettingCommand\", \"result\": \"ERROR\", \"reason\": \"Unable to parse vehicle config file\"}";
            websocket_->SendData(conn, result.c_str());
            return ;
          }

          params.mutable_vehicle_param()->set_front_edge_to_center(mvalues["front_edge_to_center"]);
          params.mutable_vehicle_param()->set_back_edge_to_center(mvalues["back_edge_to_center"]);
          params.mutable_vehicle_param()->set_left_edge_to_center(mvalues["left_edge_to_center"]);
          params.mutable_vehicle_param()->set_right_edge_to_center(mvalues["right_edge_to_center"]);
          params.mutable_vehicle_param()->set_length(mvalues["front_edge_to_center"] + mvalues["back_edge_to_center"]);
          params.mutable_vehicle_param()->set_width(mvalues["left_edge_to_center"] + mvalues["right_edge_to_center"]);
          params.mutable_vehicle_param()->set_height(1.0);
          params.mutable_vehicle_param()->set_min_turn_radius(mvalues["min_turn_radius"]);
          params.mutable_vehicle_param()->set_max_acceleration(1.0);
          params.mutable_vehicle_param()->set_max_deceleration(-1.0);
          params.mutable_vehicle_param()->set_max_steer_angle(mvalues["max_steer_angle"]);
          params.mutable_vehicle_param()->set_max_steer_angle_rate(mvalues["max_steer_angle"]);
          params.mutable_vehicle_param()->set_min_steer_angle_rate(0);
          params.mutable_vehicle_param()->set_steer_ratio(1);
          params.mutable_vehicle_param()->set_wheel_base(mvalues["wheel_base"]);
          params.mutable_vehicle_param()->set_wheel_rolling_radius(0.202);
	  
          if (apollo::common::util::SetProtoToASCIIFile(params, FLAGS_vehicle_config_path)) {
            result = "{\"type\": \"VehicleSettingCommand\", \"result\": \"OK\", \"reason\": \"Update vehicle config file successfully\"}";
            websocket_->SendData(conn, result.c_str());
          }
          std::string planning_conf_path = "/apollo/modules/planning/conf/planning.conf";
          if (FLAGS_use_navigation_mode)
            planning_conf_path = "/apollo/modules/planning/conf/planning_navi.conf";
          if (apollo::common::util::PathExists(planning_conf_path)) {
            std::ofstream planning_conf_fs;
            planning_conf_fs.open(planning_conf_path, std::ios::app);
		    planning_conf_fs << "\n--planning_upper_speed_limit=" << mvalues["maximum_speed"] << "\n";
		    planning_conf_fs << "--planning_lower_speed_limit=0.0\n";
            planning_conf_fs.close();
          }
	  
          return;
        }
        result = "{\"type\": \"VehicleSettingCommand\", \"result\": \"ERROR\", \"reason\": \"malformed json string\"}";
        websocket_->SendData(conn, result.c_str());
      });

  websocket_->RegisterMessageHandler(
      "ControlVerify",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {throttle: throttle_cmd, brake: brake_cmd, steer: steer_angle}.
        double throttle_cmd = 0.0;
        double brake_cmd = 0.0;
        double steer_angle = 0.0;
        auto iter = json.find("value");
        if (iter != json.end()) {
          auto value = *iter;
          if (JsonUtil::GetNumberFromJson(value, "throttle", &throttle_cmd) &&
              JsonUtil::GetNumberFromJson(value, "brake", &brake_cmd) &&
              JsonUtil::GetNumberFromJson(value, "steer", &steer_angle)) {
            control::ControlCommand cmd;
            cmd.set_throttle(throttle_cmd);
            cmd.set_brake(brake_cmd);
            cmd.set_steering_target(steer_angle);
            cmd.set_steering_rate(100.0);
            AdapterManager::PublishControlCommand(cmd);
          } else {
            AERROR << "Truncated control command when running ControlVerify.";
          }
        }
      });

  // HMI client asks for executing module command.
  websocket_->RegisterMessageHandler(
      "ExecuteModuleCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {module: "module_name", command: "command_name"}.
        // If module_name is "all", then run the command on all modules.
        std::string module;
        std::string command;
        if (JsonUtil::GetStringFromJson(json, "module", &module) &&
            JsonUtil::GetStringFromJson(json, "command", &command)) {
          HMIWorker::instance()->RunModuleCommand(module, command);
        } else {
          AERROR << "Truncated module command.";
        }
      });

  // HMI client asks for executing tool command.
  websocket_->RegisterMessageHandler(
      "ExecuteToolCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {tool: "tool_name", command: "command_name"}.
        std::string tool;
        std::string command;
        if (JsonUtil::GetStringFromJson(json, "tool", &tool) &&
            JsonUtil::GetStringFromJson(json, "command", &command)) {
          HMIWorker::instance()->RunToolCommand(tool, command);
        } else {
          AERROR << "Truncated tool command.";
        }
      });

  websocket_->RegisterMessageHandler(
      "GnssSettingCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {rtk_base_addr: "", rtk_base_port: "", rtk_base_mountpoint: "", rtk_base_mountpoint: "", rtk_base_username: ""
        //, rtk_base_passwd: "", local_addr: "", local_netmask: "", local_gateway: "", head_offset: "", leverarm_x: "", leverarm_y: "", leverarm_z: ""}.
        int ret_val = 1;
        char result_buf[100] = "";
        std::string rtk_base_addr;
        std::string rtk_base_port;
        std::string rtk_base_mountpoint;
        std::string rtk_base_username;
        std::string rtk_base_passwd;
        std::string local_addr;
        std::string local_netmask;
        std::string local_gateway;
        std::string head_offset;
        std::string leverarm1_x;
        std::string leverarm1_y;
        std::string leverarm1_z;
        std::string leverarm2_x;
        std::string leverarm2_y;
        std::string leverarm2_z;
		AERROR << "[GnssSettingCommand] " << json.dump();
		auto Value = json.find("value");
		if (Value != json.end()) {
      gnss_conf_logincommand_init();
			auto value = *Value;
			if (JsonUtil::GetStringFromJson(value, "rtk_base_addr", &rtk_base_addr) &&
			  JsonUtil::GetStringFromJson(value, "rtk_base_port", &rtk_base_port)) {
			  ret_val &= gnss_basestation_setting(rtk_base_addr, rtk_base_port);
			}
			if (JsonUtil::GetStringFromJson(value, "rtk_base_mountpoint", &rtk_base_mountpoint)) {
			  ret_val &= gnss_mountpoint_setting(rtk_base_mountpoint);
			}
			if (JsonUtil::GetStringFromJson(value, "rtk_base_username", &rtk_base_username) &&
			  JsonUtil::GetStringFromJson(value, "rtk_base_passwd", &rtk_base_passwd)) {
			  ret_val &= gnss_user_setting(rtk_base_username, rtk_base_passwd);
			}
			if (JsonUtil::GetStringFromJson(value, "local_addr", &local_addr)) {
			  ret_val &= gnss_localaddr_setting(local_addr);
			}
			if (JsonUtil::GetStringFromJson(value, "local_netmask", &local_netmask)) {
			  ret_val &= gnss_localnetmask_setting(local_netmask);
			}
			if (JsonUtil::GetStringFromJson(value, "local_gateway", &local_gateway)) {
			  ret_val &= gnss_localgateway_setting(local_gateway);
			}
			if (JsonUtil::GetStringFromJson(value, "head_offset", &head_offset)) {
			  ret_val &= gnss_headoffset_setting(head_offset);
			}
			if (JsonUtil::GetStringFromJson(value, "leverarm1_x", &leverarm1_x) &&
			  JsonUtil::GetStringFromJson(value, "leverarm1_y", &leverarm1_y) &&
			  JsonUtil::GetStringFromJson(value, "leverarm1_z", &leverarm1_z)) {
			  ret_val &= gnss_leverarm1_setting(leverarm1_x,leverarm1_y,leverarm1_z);
			}
      if (JsonUtil::GetStringFromJson(value, "leverarm2_x", &leverarm2_x) &&
        JsonUtil::GetStringFromJson(value, "leverarm2_y", &leverarm2_y) &&
        JsonUtil::GetStringFromJson(value, "leverarm2_z", &leverarm2_z)) {
        ret_val &= gnss_leverarm2_setting(leverarm2_x,leverarm2_y,leverarm2_z);
      }
      gnss_setting_save_and_close();
		} else
			ret_val &= 0;
        sprintf(result_buf,"{\"GnssSettingCommand\": \"GnssSettingCommand\", \"result\": \"%s\",\"reason\": \"\"}",ret_val?"OK":"ERROR");
        websocket_->SendData(conn, result_buf);
      });

  // Save UltrasonicRadarConfigs to File
  websocket_->RegisterMessageHandler(
      "RadarDetectSetting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {

        apollo::guardian::UltrasonicRadarConfigs config;
        
        // convert json to proto
        JsonUtil::JsonStrToProtoMsg(json, config);

        //save proto to file
        apollo::common::util::SetProtoToASCIIFile(config,FLAGS_ultrasonic_radar_config_filename);
        AINFO << "save UltrasonicRadarConfigs";
      });

  // HMI client asks for executing mode command.
  websocket_->RegisterMessageHandler(
      "ExecuteModeCommand",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {command: "command_name"}.
        // Supported commands are: "start", "stop".
        std::string command;
        if (JsonUtil::GetStringFromJson(json, "command", &command)) {
          HMIWorker::instance()->RunModeCommand(command);
        } else {
          AERROR << "Truncated mode command.";
        }
      });

  // HMI client asks for changing driving mode.
  websocket_->RegisterMessageHandler(
      "ChangeDrivingMode",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_mode: "DrivingModeName"}.
        // DrivingModeName should be one of canbus::Chassis::DrivingMode.
        // For now it is either COMPLETE_MANUAL or COMPLETE_AUTO_DRIVE.
        std::string new_mode;
        if (JsonUtil::GetStringFromJson(json, "new_mode", &new_mode)) {
          Chassis::DrivingMode mode;
          if (Chassis::DrivingMode_Parse(new_mode, &mode)) {
            HMIWorker::ChangeToDrivingMode(mode);
          } else {
            AERROR << "Unknown driving mode " << new_mode;
          }
        } else {
          AERROR << "Truncated ChangeDrivingMode request.";
        }
      });

  // HMI client asks for changing map.
  HMIWorker::instance()->RegisterChangeMapHandler(
      [this](const std::string &new_map) {
        // Reload simulation map after changing map.
        CHECK(map_service_->ReloadMap(true))
            << "Failed to load new simulation map: " << new_map;
        // And then broadcast new HMIStatus to all clients.
        DeferredBroadcastHMIStatus();
      });
  websocket_->RegisterMessageHandler(
      "ChangeMap",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_map: "MapName"}.
        // MapName should be a key of config_.available_maps.
        std::string new_map;
        if (JsonUtil::GetStringFromJson(json, "new_map", &new_map)) {
          HMIWorker::instance()->ChangeToMap(new_map);
        } else {
          AERROR << "Truncated ChangeMap request.";
        }
      });

  // HMI client asks for changing vehicle.
  HMIWorker::instance()->RegisterChangeVehicleHandler(
      [this](const std::string &new_vehicle) {
        // Broadcast new HMIStatus and VehicleParam.
        DeferredBroadcastHMIStatus();
        SendVehicleParam();
        SendGnssParam();
        SendRadarParam();
      });
  websocket_->RegisterMessageHandler(
      "ChangeVehicle",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_vehicle: "VehicleName"}.
        // VehicleName should be a key of config_.available_vehicles.
        std::string new_vehicle;
        if (JsonUtil::GetStringFromJson(json, "new_vehicle", &new_vehicle)) {
          HMIWorker::instance()->ChangeToVehicle(new_vehicle);
        } else {
          AERROR << "Truncated ChangeVehicle request.";
        }
      });

  // HMI client asks for changing mode.
  HMIWorker::instance()->RegisterChangeModeHandler(
      [this](const std::string &new_mode) {
        // Broadcast new HMIStatus.
        DeferredBroadcastHMIStatus();
      });
  websocket_->RegisterMessageHandler(
      "ChangeMode",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain {new_mode: "ModeName"}.
        // ModeName should be a key of config_.modes.
        std::string new_mode;
        if (JsonUtil::GetStringFromJson(json, "new_mode", &new_mode)) {
          HMIWorker::instance()->ChangeToMode(new_mode);
        } else {
          AERROR << "Truncated ChangeMode request.";
        }
      });

  // HMI client asks for adding new DriveEvent.
  websocket_->RegisterMessageHandler(
      "SubmitDriveEvent",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        // json should contain event_time_ms and event_msg.
        uint64_t event_time_ms;
        std::string event_msg;
        std::vector<std::string> event_types;
        apollo::common::monitor::MonitorLogBuffer log_buffer(&logger_);
        if (JsonUtil::GetNumberFromJson(json, "event_time_ms",
                                        &event_time_ms) &&
            JsonUtil::GetStringFromJson(json, "event_msg", &event_msg) &&
            JsonUtil::GetStringVectorFromJson(json, "event_type",
                                              &event_types)) {
          HMIWorker::SubmitDriveEvent(event_time_ms, event_msg, event_types);
          log_buffer.INFO("Drive event added.");
        } else {
          AERROR << "Truncated SubmitDriveEvent request.";
          log_buffer.WARN("Failed to submit a drive event.");
        }
      });

  //
  websocket_->RegisterMessageHandler(
	"SubmitRecordingMap",
	[this](const Json &json, WebSocketHandler::Connection *conn) {
          std::string map_dir, bag_name, command, type, mapName;
          if (JsonUtil::GetStringFromJson(json, "cmd", &type)) {
            bool ret_val = false;

            if (type == "start") {
              if (map_gen_) {
                JsonUtil::GetStringFromJson(json, "mapName", &mapName);
                ret_val = map_gen_->OnStartRecordCommand(mapName);
              }

              nlohmann::json response;

              response["type"] = "RecordingMapResponse";
              response["stage"] = "Recording";
              response["result"] = ret_val ? "success" : "error";
              if (conn != nullptr) {
                websocket_->SendData(conn, response.dump());
              } else {
                websocket_->BroadcastData(response.dump());
              }
            } else if (type == "stop") {
              std::string map_path;

              if (map_gen_) {
                map_path =  map_gen_->OnStopRecordCommand();
              }

              nlohmann::json response;

              response["type"] = "RecordingMapResponse";
              response["stage"] = "Mapping";
              response["result"] = map_path.empty() ? "error" : "success";
              if (conn != nullptr) {
                websocket_->SendData(conn, response.dump());
              } else {
                websocket_->BroadcastData(response.dump());
              }

              HMIWorker::instance()->UpdateMappingList();
              const auto &config = HMIWorker::instance()->GetConfig();
              websocket_->SendData(conn, JsonUtil::ProtoToTypedJson("HMIConfig", config).dump());
            }
          }
        });

  // Received new system status, broadcast to clients.
  AdapterManager::AddSystemStatusCallback(
      [this](const monitor::SystemStatus &system_status) {
        if (Clock::NowInSeconds() - system_status.header().timestamp_sec() <
            FLAGS_system_status_lifetime_seconds) {
          HMIWorker::instance()->UpdateSystemStatus(system_status);
          DeferredBroadcastHMIStatus();
        }
      });

  // Received Chassis, trigger action if there is high beam signal.
  AdapterManager::AddChassisCallback([this](const Chassis &chassis) {
    if (Clock::NowInSeconds() - chassis.header().timestamp_sec() <
        FLAGS_system_status_lifetime_seconds) {
      if (chassis.signal().high_beam()) {
        const bool ret = HMIWorker::instance()->Trigger(
            HMIWorker::instance()->GetConfig().chassis_high_beam_action());
        AERROR_IF(!ret) << "Failed to execute high_beam action.";
      }
    }
  });
}

void HMI::StartBroadcastHMIStatusThread() {
  constexpr int kMinBroadcastIntervalMs = 200;
  broadcast_hmi_status_thread_.reset(new std::thread([this]() {
    while (true) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMinBroadcastIntervalMs));

      {
        std::lock_guard<std::mutex> lock(need_broadcast_mutex_);
        if (!need_broadcast_) {
          continue;
        }
        // Reset to false.
        need_broadcast_ = false;
      }

      // Get a copy of status.
      const auto status = HMIWorker::instance()->GetStatus();
      websocket_->BroadcastData(
          JsonUtil::ProtoToTypedJson("HMIStatus", status).dump());

      // Broadcast messages.
      apollo::common::monitor::MonitorLogBuffer log_buffer(&logger_);
      if (status.current_map().empty()) {
        log_buffer.WARN("You haven't selected a map yet!");
      }
      if (status.current_vehicle().empty()) {
        log_buffer.WARN("You haven't selected a vehicle yet!");
      }
    }
  }));
}

void HMI::DeferredBroadcastHMIStatus() {
  std::lock_guard<std::mutex> lock(need_broadcast_mutex_);
  need_broadcast_ = true;
}

void HMI::SendVehicleParam(WebSocketHandler::Connection *conn) {
  if (websocket_ == nullptr) {
    return;
  }

  const auto json_str =
      JsonUtil::ProtoToTypedJson(
          "VehicleParam", VehicleConfigHelper::GetConfig().vehicle_param())
          .dump();
  if (conn != nullptr) {
    websocket_->SendData(conn, json_str);
  } else {
    websocket_->BroadcastData(json_str);
  }
}

void HMI::SendGnssParam(WebSocketHandler::Connection *conn) {
  if (websocket_ == nullptr) {
    return;
  }
  nlohmann::json json_origin_gnssparam;

  nlohmann::json json_origin_gnssparam_data;

  std::string localip;
  std::string localmask;
  std::string localgateway;
  std::string headoffset;
  std::string leverarm1_x, leverarm1_y, leverarm1_z;
  std::string leverarm2_x, leverarm2_y, leverarm2_z;
  std::string rtk_base_addr, rtk_base_port;
  std::string rtk_base_username, rtk_base_passwd;
  std::string rtk_base_mountpoint;
 
  {
    std::lock_guard<std::mutex> lock(need_broadcast_mutex_);
    gnss_conf_init();
    gnss_config_get();
    gnss_localip_get(localip);
    gnss_localmask_get(localmask);
    gnss_localgateway_get(localgateway);
    gnss_headoffset_get(headoffset);
    gnss_leverarm1_get(leverarm1_x, leverarm1_y, leverarm1_z);
    gnss_leverarm2_get(leverarm2_x, leverarm2_y, leverarm2_z);
    gnss_basestation_get(rtk_base_addr, rtk_base_port);
    gnss_user_get(rtk_base_username, rtk_base_passwd);
    gnss_mountpoint_get(rtk_base_mountpoint);
  }

  json_origin_gnssparam_data["rtkBaseAddr"] = rtk_base_addr.c_str();
  json_origin_gnssparam_data["rtkBasePort"] = rtk_base_port.c_str();
  json_origin_gnssparam_data["rtkBaseUsername"] = rtk_base_username.c_str();
  json_origin_gnssparam_data["rtkBasePasswd"] = rtk_base_passwd.c_str();
  json_origin_gnssparam_data["rtkBaseMountpoint"] = rtk_base_mountpoint.c_str();
  json_origin_gnssparam_data["localAddr"] = localip.c_str();
  json_origin_gnssparam_data["localNetmask"] = localmask.c_str();
  json_origin_gnssparam_data["localGateway"] = localgateway.c_str();
  json_origin_gnssparam_data["headOffset"] = headoffset.c_str();
  json_origin_gnssparam_data["leverarm1X"] = leverarm1_x.c_str();
  json_origin_gnssparam_data["leverarm1Y"] = leverarm1_y.c_str();
  json_origin_gnssparam_data["leverarm1Z"] = leverarm1_z.c_str();
  json_origin_gnssparam_data["leverarm2X"] = leverarm2_x.c_str();
  json_origin_gnssparam_data["leverarm2Y"] = leverarm2_y.c_str();
  json_origin_gnssparam_data["leverarm2Z"] = leverarm2_z.c_str();
  json_origin_gnssparam["type"] = "GnssParam";
  json_origin_gnssparam["data"] = json_origin_gnssparam_data;
  printf("--->%s\n",json_origin_gnssparam.dump().c_str());
  /*
  const auto json_str =
      JsonUtil::ProtoToTypedJson(
          "GnssParam", VehicleConfigHelper::GetConfig().vehicle_param())
          .dump();
          */
  if (conn != nullptr) {
    websocket_->SendData(conn, json_origin_gnssparam.dump());
  } else {
    websocket_->BroadcastData(json_origin_gnssparam.dump());
  }
}

// Send UltrasonicRadarConfigs to frontend
void HMI::SendRadarParam(WebSocketHandler::Connection *conn) {
  if (websocket_ == nullptr) {
    return;
  }
  
  //read config-file
  apollo::guardian::UltrasonicRadarConfigs config;
  apollo::common::util::GetProtoFromASCIIFile(FLAGS_ultrasonic_radar_config_filename, &config);

  //covert protobuf to json
  Json json_obj = JsonUtil::ProtoToTypedJson("RadarParam", config);
  
  // json to string
  const auto str=json_obj.dump();

  printf("--->%s\n",str.c_str());

  if (conn != nullptr) {
    websocket_->SendData(conn, str);
  } else {
    websocket_->BroadcastData(str);
  }
}

}  // namespace dreamview
}  // namespace apollo
