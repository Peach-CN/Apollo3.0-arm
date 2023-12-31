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

#include "modules/common/util/json_util.h"

#include "google/protobuf/util/json_util.h"

namespace apollo {
namespace common {
namespace util {
namespace {

using Json = nlohmann::json;
using google::protobuf::util::MessageToJsonString;
using google::protobuf::util::JsonStringToMessage;

google::protobuf::util::JsonOptions JsonOption() {
  google::protobuf::util::JsonOptions json_option;
  json_option.always_print_primitive_fields = true;
  return json_option;
}

google::protobuf::util::JsonParseOptions JsonParseOption() {
  google::protobuf::util::JsonParseOptions json_option;
  json_option.ignore_unknown_fields = true;
  return json_option;
}

}  // namespace

//convert proto to JsonString
nlohmann::json JsonUtil::ProtoToTypedJson(const std::string &json_type,
                                          const google::protobuf::Message &proto) {
  static const auto kJsonOption = JsonOption();
  std::string json_string;
  const auto status = MessageToJsonString(proto, &json_string, kJsonOption);
  CHECK(status.ok()) << "Cannot convert proto to json:" << proto.DebugString();

  Json json_obj;
  json_obj["type"] = json_type;
  json_obj["data"] = Json::parse(json_string);
  return json_obj;
}

//convert JsonString to proto
void JsonUtil::JsonStrToProtoMsg(const nlohmann::json &json_obj,google::protobuf::Message &msg) {
  std::string json_str = json_obj["data"].dump();

  static const auto kJsonOption = JsonParseOption();
  const auto status = JsonStringToMessage(json_str, &msg, kJsonOption);
  CHECK(status.ok()) << "Cannot convert json to proto:" << json_str;
}

bool JsonUtil::GetStringFromJson(const nlohmann::json &json, const std::string &key,
                                 std::string *value) {
  const auto iter = json.find(key);
  if (iter == json.end()) {
    AERROR << "The json has no such key: " << key;
    return false;
  }
  if (!iter->is_string()) {
    AERROR << "The value of json[" << key << "] is not a string";
    return false;
  }
  *value = *iter;
  return true;
}

bool JsonUtil::GetStringVectorFromJson(const nlohmann::json &json, const std::string &key,
                                       std::vector<std::string> *value) {
  const auto iter = json.find(key);
  if (iter == json.end()) {
    AERROR << "The json has no such key: " << key;
    return false;
  }
  if (!iter->is_array()) {
    AERROR << "The value of json[" << key << "] is not an array";
    return false;
  }

  bool ret = true;
  value->clear();
  value->reserve(iter->size());
  for (const auto &elem : *iter) {
    // Note that we still try to get all string values though there are invalid
    // elements.
    if (!elem.is_string()) {
      AWARN << "The value of json[" << key << "] contains non-string element";
      ret = false;
    } else {
      value->push_back(elem);
    }
  }
  return ret;
}

bool JsonUtil::GetNumberVectorFromJson(const nlohmann::json &json, const std::string &key,  std::vector<float> *value){
  const auto iter = json.find(key);
  if (iter == json.end()) {
    AERROR << "The json has no such key: " << key;
    return false;
  }
  if (!iter->is_array()) {
    AERROR << "The value of json[" << key << "] is not an array";
    return false;
  }

  bool ret = true;
  value->clear();
  value->reserve(iter->size());
  for (const auto &elem : *iter) {
    // Note that we still try to get all string values though there are invalid
    // elements.
    if (!elem.is_number()) {
      AWARN << "The value of json[" << key << "] contains non-float element";
      ret = false;
    } else {
      value->push_back(elem);
    }
  }
  return ret;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
