#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"
#include <chrono>
#include "DynamicObjectsAvoidance/config.h"

namespace DynamicObjectsAvoidance {

DynamicObjectsAvoidance::DynamicObjectsAvoidance(std::string &config_path)
    : config_file_path_(config_path) {
}

bool DynamicObjectsAvoidance::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    store_rgbd_ = Config::Get<int>("store_rgbd");
    int mode = Config::Get<int>("mode");
    switch (mode)
    {
    case 2: { // event_rgbd mode
        data_reader_ = DataReader::Ptr(new DataReader(Config::Get<std::string>("event_dataset_path"),
            Config::Get<std::string>("rgbd_dataset_path"), Config::Get<std::string>("cfg_file_path")));
        // Need to init event camera first, because the cameras_[0] should be event camera
        CHECK_EQ(data_reader_->InitEventCamera(), true);
        CHECK_EQ(data_reader_->InitRGBD(), true);
        LOG(INFO) << "Get dataset: event & rgbd offline mode";
    } break;
    case 3: { // event_rgbd_live mode
        data_reader_ = DataReader::Ptr(new DataReader(Config::Get<std::string>("cfg_file_path")));
        CHECK_EQ(data_reader_->InitEventCamera(), true);
        CHECK_EQ(data_reader_->InitRGBD(), true);
        // CHECK_EQ(dataset_->InitRGBD(), true);
    } break;

    default:
        LOG(ERROR) << "Unknown mode!!!";
        break;
    }

    return true;
}

void DynamicObjectsAvoidance::Run() {
    while (1) {
        LOG(INFO) << "Dynamic Objects Avoidance is running";
        if (Step() == false) {
            LOG(INFO)<<"step = false";
            break;
        }
    }

    // backend_->Stop();
    // viewer_->Close();

    LOG(INFO) << "Dynamic Objects Avoidance Exit";
}

bool DynamicObjectsAvoidance::Step() {
    auto t3 = std::chrono::steady_clock::now();
    Frame::Ptr new_frame;
    if (data_reader_) {
        new_frame = data_reader_->NextFrame();
        if (new_frame == nullptr) return false;
    }
    return true;
}

}  // namespace DynamicObjectsAvoidance
