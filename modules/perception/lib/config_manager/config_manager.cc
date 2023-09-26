// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: YangGuang Li (liyangguang@baidu.com)
// @file: config_manager.cpp
// @brief:
#include "modules/perception/lib/config_manager/config_manager.h"

#include <gflags/gflags.h>
#include <google/protobuf/text_format.h>

#include "modules/perception/lib/base/file_util.h"
//#include "lib/base/perception.h"
#include "modules/perception/lib/config_manager/proto/perception_config_schema.pb.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace config_manager {

DEFINE_string(config_manager_path, "/apollo/modules/perception/model/cnn_segmentation/config_manager.config",
        "The ModelConfig config paths file.");
DEFINE_string(work_root, "", "Project work root direcotry.");
DEFINE_string(adu_data, "/home/caros/adu_data",
        "ADU shared data path, including maps, routings...");

using google::protobuf::TextFormat;

using std::pair;
using std::string;
using std::vector;

ConfigManager::ConfigManager() {
    _work_root = FLAGS_work_root;
    _adu_data = FLAGS_adu_data;
}

bool ConfigManager::init() {
    MutexLock lock(&_mutex);
    return init_internal();
}

bool ConfigManager::init_internal() {
    if (_inited) {
        return true;
    }
    ModelConfigMapIterator iter = _model_config_map.begin();
    for (; iter != _model_config_map.end(); ++iter) {
        delete iter->second;
    }
    _model_config_map.clear();

    // For start at arbitrary path
    if (_work_root.empty()) {
        _work_root = get_env("MODULE_PATH");
        if (_work_root.empty()) {
            _work_root = get_env("CYBERTRON_PATH");
        }
    }

    std::string path = FileUtil::get_absolute_path(_work_root, FLAGS_config_manager_path);

    AINFO << "WORK_ROOT: " << _work_root << " config_manager_path: "
              << path << " ADU_DATA: " << _adu_data;

    std::string content;
    if (!FileUtil::get_file_content(path, &content)) {
        AERROR << "failed to get ConfigManager config path: "
            << path;
        return false;
    }

    ModelConfigFileListProto file_list_proto;

    if (!TextFormat::ParseFromString(content, &file_list_proto)) {
        AERROR << "invalid ModelConfigFileListProto file: "
            << FLAGS_config_manager_path;
        return false;
    }

    for (const std::string& model_config_file : file_list_proto.model_config_path()) {

        std::string abs_path = FileUtil::get_absolute_path(_work_root, model_config_file);

        std::string config_content;
        if (!FileUtil::get_file_content(abs_path, &config_content)) {
            AERROR << "failed to get_file_content: " << abs_path;
            return false;
        }

        MultiModelConfigProto multi_model_config_proto;

        if (!TextFormat::ParseFromString(config_content, &multi_model_config_proto)) {
            AERROR << "invalid MultiModelConfigProto file: "
                << abs_path;
            return false;
        }

        for (const ModelConfigProto& model_config_proto :
                multi_model_config_proto.model_configs()) {

            ModelConfig* model_config = new ModelConfig();
            if (!model_config->reset(model_config_proto)) {
                return false;
            }

            AERROR << "load ModelConfig succ. name: " << model_config->name();

            pair<ModelConfigMapIterator, bool> result =
                _model_config_map.emplace(model_config->name(), model_config);
            if (!result.second) {
                AERROR << "duplicate ModelConfig, name: "
                    << model_config->name();
                return false;
            }
        }
    }

    AINFO << "finish to load ModelConfigs. num_models: " << _model_config_map.size();

    _inited = true;

    return true;
}

bool ConfigManager::reset() {
    MutexLock lock(&_mutex);
    _inited = false;
    return init_internal();
}

std::string ConfigManager::get_env(const std::string& var_name) {
    char* var = nullptr;
    var = getenv(var_name.c_str());
    if (var == nullptr) {
        return std::string("");
    }
    return std::string(var);
}

bool ConfigManager::get_model_config(
        const string& model_name,
        const ModelConfig** model_config) {
    if (!_inited && !init()) {
        return false;
    }

    ModelConfigMapConstIterator citer = _model_config_map.find(model_name);

    if (citer == _model_config_map.end()) {
        return false;
    }
    *model_config = citer->second;
    return true;
}

ConfigManager::~ConfigManager() {
    ModelConfigMapIterator iter = _model_config_map.begin();
    for (; iter != _model_config_map.end(); ++iter) {
        delete iter->second;
    }
}

bool ModelConfig::reset(const ModelConfigProto& proto) {
    _name = proto.name();
    _version = proto.version();

    _integer_param_map.clear();
    _string_param_map.clear();
    _double_param_map.clear();
    _float_param_map.clear();
    _bool_param_map.clear();
    _array_integer_param_map.clear();
    _array_string_param_map.clear();
    _array_double_param_map.clear();
    _array_float_param_map.clear();
    _array_bool_param_map.clear();

    for (const KeyValueInt& pair : proto.integer_params()) {
        _integer_param_map.emplace(pair.name(), pair.value());
    }

    for (const KeyValueString& pair : proto.string_params()) {
        _string_param_map.emplace(pair.name(), pair.value());
    }

    for (const KeyValueDouble& pair : proto.double_params()) {
        _double_param_map.emplace(pair.name(), pair.value());
    }

    for (const KeyValueFloat& pair : proto.float_params()) {
        _float_param_map.emplace(pair.name(), pair.value());
    }

    for (const KeyValueBool& pair : proto.bool_params()) {
        _bool_param_map.emplace(pair.name(), pair.value());
    }

    for (const KeyValueArrayInt& pair : proto.array_integer_params()) {
        vector<int> values;
        repeated_to_vector(pair.values(), &values);
        _array_integer_param_map.emplace(pair.name(), values);
    }

    for (const KeyValueArrayString& pair : proto.array_string_params()) {
        vector<string> values;
        values.reserve(pair.values_size());
        for (const string& value : pair.values()) {
            values.push_back(value);
        }
        _array_string_param_map.emplace(pair.name(), values);
    }

    for (const KeyValueArrayDouble& pair : proto.array_double_params()) {
        vector<double> values;
        repeated_to_vector(pair.values(), &values);
        _array_double_param_map.emplace(pair.name(), values);
    }

    for (const KeyValueArrayFloat& pair : proto.array_float_params()) {
        vector<float> values;
        repeated_to_vector(pair.values(), &values);
        _array_float_param_map.emplace(pair.name(), values);
    }

    for (const KeyValueArrayBool& pair : proto.array_bool_params()) {
        vector<bool> values;
        repeated_to_vector(pair.values(), &values);
        _array_bool_param_map.emplace(pair.name(), values);
    }

    AINFO << "reset ModelConfig. model_name: " << _name
              << " integer_param_map's size: " << _integer_param_map.size()
              << " string_param_map's size: " << _string_param_map.size()
              << " double_param_map's size: " << _double_param_map.size()
              << " float_param_map's size: " << _float_param_map.size()
              << " bool_param_map's size: " << _bool_param_map.size()
              << " array_integer_param_map's size: " << _array_integer_param_map.size()
              << " array_string_param_map's size: " << _array_string_param_map.size()
              << " array_double_param_map's size: " << _array_double_param_map.size()
              << " array_float_param_map's size: " << _array_float_param_map.size()
              << " array_bool_param_map's size: " << _array_bool_param_map.size();

    return true;
}

}  // namespace config_manager
}  // namespace perception
}  // namespace apollo