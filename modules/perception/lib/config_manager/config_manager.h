// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: YangGuang Li (liyangguang@baidu.com)
// @file: config_manager.h
// @brief:
//
// ConfigManager use protobuf text format to manage all your model configs.
//
// CODE SAMPLE:
// you can use such code to access your parameters:
//
//         #include "lib/config_manager/config_manager.h"
//
//         ConfigManager* config_manager = base::Singleton<ConfigManager>::get();
//
//         string model_name = "FrameClassifier";
//         const ModelConfig* model_config = NULL;
//         if (!config_manager->get_model_config(model_name, &model_config)) {
//            LOG_ERROR << "not found model: " << model_name;
//            return false;
//         }
//
//         int int_value = 0;
//         if (!model_config->get_value("my_param_name", &int_value)) {
//             LOG_ERROR << "my_param_name not found."
//             return false;
//         }
//         using int_value....
//
//
// CONFIG FORMAT
//
// First you should define file: conf/config_manager.config,
// you can set the path by gflags
//    --config_manager_path=conf/config_manager.config
//
// file content like as:
// define all model config paths.
// ############################################
//
// model_config_path: "./conf/frame_classifier.config"
// model_config_path: "./conf/track_classifier.config"
//
// ############################################
//
// one line identify one model parameter config path.
// ModelConfig config file like as:
// file: ./conf/frame_classifier.config
// #############################################
//
// model_configs {
//     # FrameClassifier model.
//     name: "FrameClassifier"
//     version: "1.0.0"
//     integer_params {
//         name: "threshold1"
//         value: 1
//     }
//
//     integer_params {
//         name: "threshold2"
//         value: 2
//     }
//
//     string_params {
//         name: "threshold3"
//         value: "str3"
//     }
//
//     double_params {
//         name: "threshold4"
//         value: 4.0
//     }
//
//     array_integer_params {
//         name: "array_p1"
//         values: 1
//         values: 2
//         values: 3
//     }
//
//     array_string_params {
//         name: "array_p2"
//         values: "str1"
//         values: "str2"
//         values: "str3"
//         values: "str4"
//     }
//
//     array_string_params {
//         name: "array_p3"
//         values: "str1"
//         values: "str2"
//         values: "str3"
//         values: "str4"
//     }
//
//     array_double_params {
//         name: "array_p4"
//         values: 1.1
//         values: 1.2
//         values: 1.3
//         values: 1.4
//     }
// }

#ifndef ADU_PERCEPTION_LIB_CONFIG_MANAGER_H
#define ADU_PERCEPTION_LIB_CONFIG_MANAGER_H

#include <typeinfo>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <google/protobuf/message.h>

#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/lib/base/noncopyable.h"
#include "modules/perception/lib/base/singleton.h"

namespace apollo {
namespace perception {
namespace config_manager {

class ModelConfig;
class ModelConfigProto;

class ConfigManager {
public:
    // thread-safe interface.
    bool init();

    // thread-safe interface.
    bool reset();

    bool get_model_config(
            const std::string& model_name,
            const ModelConfig** model_config);

    size_t num_models() const {
        return _model_config_map.size();
    }

    const std::string& work_root() const {
        return _work_root;
    }

    const std::string& adu_data() const {
        return _adu_data;
    }

    void set_adu_data(const std::string& adu_data) {
        _adu_data = adu_data;
    }

    void set_work_root(const std::string& work_root) {
        _work_root = work_root;
    }

private:
    ConfigManager();
    ~ConfigManager();

    bool init_internal();
    std::string get_env(const std::string& var_name);

    friend class Singleton<ConfigManager>;

    typedef std::map<std::string, ModelConfig*> ModelConfigMap;
    typedef ModelConfigMap::iterator ModelConfigMapIterator;
    typedef ModelConfigMap::const_iterator ModelConfigMapConstIterator;

    // key: model_name
    ModelConfigMap _model_config_map;
    Mutex _mutex;  // multi-thread init safe.
    bool _inited = false;
    std::string _work_root;  // ConfigManager work root dir.
    std::string _adu_data;

    DISALLOW_COPY_AND_ASSIGN(ConfigManager);
};

class ModelConfig {
public:
    ModelConfig() {}
    ~ModelConfig() {}

    bool reset(const ModelConfigProto& proto);

    std::string name() const {
        return _name;
    }

    bool get_value(const std::string& name, int* value) const {
        return get_value_from_map<int>(name, _integer_param_map, value);
    }

    bool get_value(const std::string& name, std::string* value) const {
        return get_value_from_map<std::string>(name, _string_param_map, value);
    }

    bool get_value(const std::string& name, double* value) const {
        return get_value_from_map<double>(name, _double_param_map, value);
    }

    bool get_value(const std::string& name, float* value) const {
        return get_value_from_map<float>(name, _float_param_map, value);
    }

    bool get_value(const std::string& name, bool* value) const {
        return get_value_from_map<bool>(name, _bool_param_map, value);
    }

    bool get_value(const std::string& name, std::vector<int>* values) const {
        return get_value_from_map<std::vector<int> >(
                name,
                _array_integer_param_map,
                values);
    }

    bool get_value(const std::string& name, std::vector<double>* values) const {
        return get_value_from_map<std::vector<double> >(
                name,
                _array_double_param_map,
                values);
    }

    bool get_value(const std::string& name, std::vector<float>* values) const {
        return get_value_from_map<std::vector<float> >(
                name,
                _array_float_param_map,
                values);
    }

    bool get_value(const std::string& name, std::vector<std::string>* values) const {
        return get_value_from_map<std::vector<std::string> >(
                name,
                _array_string_param_map,
                values);
    }

    bool get_value(const std::string& name, std::vector<bool>* values) const {
        return get_value_from_map<std::vector<bool> >(
                name,
                _array_bool_param_map,
                values);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(ModelConfig);

    template<typename T>
    bool get_value_from_map(
            const std::string& name,
            const std::map<std::string, T>& container,
            T* value) const;


    template<typename T>
    void repeated_to_vector(
            const google::protobuf::RepeatedField<T>& repeated_values,
            std::vector<T>* vec_values);

    std::string _name;
    std::string _version;

    typedef std::map<std::string, int> IntegerParamMap;
    typedef std::map<std::string, std::string> StringParamMap;
    typedef std::map<std::string, double> DoubleParamMap;
    typedef std::map<std::string, float> FloatParamMap;
    typedef std::map<std::string, bool> BoolParamMap;
    typedef std::map<std::string, std::vector<int> > ArrayIntegerParamMap;
    typedef std::map<std::string, std::vector<std::string> > ArrayStringParamMap;
    typedef std::map<std::string, std::vector<double> > ArrayDoubleParamMap;
    typedef std::map<std::string, std::vector<float> > ArrayFloatParamMap;
    typedef std::map<std::string, std::vector<bool> > ArrayBoolParamMap;

    IntegerParamMap _integer_param_map;
    StringParamMap _string_param_map;
    DoubleParamMap _double_param_map;
    FloatParamMap _float_param_map;
    BoolParamMap _bool_param_map;
    ArrayIntegerParamMap _array_integer_param_map;
    ArrayStringParamMap _array_string_param_map;
    ArrayDoubleParamMap _array_double_param_map;
    ArrayFloatParamMap _array_float_param_map;
    ArrayBoolParamMap _array_bool_param_map;
};

template<typename T>
bool ModelConfig::get_value_from_map(
        const std::string& name,
        const std::map<std::string, T>& container,
        T* value) const {
    typename std::map<std::string, T>::const_iterator citer =
        container.find(name);

    if (citer == container.end()) {
        return false;
    }

    *value = citer->second;
    return true;
}

template<typename T>
void ModelConfig::repeated_to_vector(
        const google::protobuf::RepeatedField<T>& repeated_values,
        std::vector<T>* vec_list) {
    vec_list->reserve(repeated_values.size());
    for (T value : repeated_values) {
        vec_list->push_back(value);
    }
}

class ConfigManagerError {
public:
    ConfigManagerError(const std::string& error_info) :
        _error_info(error_info) {
    }
    std::string what() const {
        return _error_info;
    }
private:
    std::string _error_info;
};

template<typename T>
class ConfigRead {
public:
    static T read(const ModelConfig& config, const std::string& name) {
        T ret;
        if (!config.get_value(name, &ret)) {
            std::stringstream ss;
            ss << "Config name:" << config.name() << " read failed. "
                <<"type:" << typeid(T).name() << " name:" << name;
            throw ConfigManagerError(ss.str());
        }
        return ret;
    }
};

template<typename T>
class ConfigRead<std::vector<T> > {
public:
    static std::vector<T> read(const ModelConfig& config, const std::string& name) {
        std::vector<T> ret;
        if (!config.get_value(name, &ret)) {
            std::stringstream ss;
            ss << "Config name:" << config.name() << " read failed. "
                << "type:vector<" << typeid(T).name() << "> name:" << name;
            throw ConfigManagerError(ss.str());
        }
        return std::move(ret);
    }
};

}  // namespace config_manager
}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_LIB_CONFIG_MANAGER_H
