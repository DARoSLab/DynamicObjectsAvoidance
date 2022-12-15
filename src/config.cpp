#include "DynamicObjectsAvoidance/config.h"

namespace DynamicObjectsAvoidance {
bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);
    // config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    config_->file_.open(filename.c_str(), cv::FileStorage::READ); // Works generally

    if (config_->file_.isOpened() == false) {
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release();
        return false;
    }
    return true;
}

Config::~Config() {
    if (file_.isOpened()){
        file_.release();
    }
}

std::shared_ptr<Config> Config::config_ = nullptr;

}