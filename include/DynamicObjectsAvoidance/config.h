#pragma once
// #ifndef DynamicObjectsAvoidance_CONFIG_H
// #define DynamicObjectsAvoidance_CONFIG_H

#include "DynamicObjectsAvoidance/common_include.h"

namespace DynamicObjectsAvoidance {

/**
 * Config class, use SetParameterFile to set a new config file
 * Then use Get to get value
 */
class Config {
   private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}  // private constructor makes a singleton
   public:
    ~Config();  // close the file when deconstructing

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string &key) {
        return T(Config::config_->file_[key]);
    }
};
}  // namespace DynamicObjectsAvoidance

// #endif  // DynamicObjectsAvoidance_CONFIG_H
