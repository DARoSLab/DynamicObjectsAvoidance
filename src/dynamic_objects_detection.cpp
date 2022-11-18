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
        // std::string event_path = Config::Get<std::string>("event_dataset_path")
        //             +Config::Get<std::string>("event_data_file_name") + ".aedat4";
        // data_reader_ = DataReader::Ptr(new DataReader(event_path,
        //     Config::Get<std::string>("rgbd_dataset_path"), Config::Get<std::string>("cfg_file_path")));
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


    // switch (mode)
    // {
    // case 0: { // stereo mode
    //     frontend_->SetStereoCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
    //     backend_->SetStereoCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
    // } break;
    // case 2: { // event_rgbd mode
    //     frontend_->SetEventRGBDCameras(data_reader_->GetEventCamera(), data_reader_->GetRGBDCamera());
    //     backend_->SetEventRGBDCameras(data_reader_->GetEventCamera(), data_reader_->GetRGBDCamera());
    //     frontend_->SetEventStore(data_reader_->GetEventStore());
    //     frontend_->SetImuLinQueue(data_reader_->GetImuLinQueue());
    //     frontend_->SetImuAngQueue(data_reader_->GetImuAngQueue());
    //     LOG(WARNING) << data_reader_->GetEventCamera()->K();
    // } break;
    // case 3: { // event_rgbd_live mode
    //     //! todo merge
    //     frontend_->SetEventRGBDCameras(data_reader_->GetEventCamera(), data_reader_->GetRGBDCamera());
    //     backend_->SetEventRGBDCameras(data_reader_->GetEventCamera(), data_reader_->GetRGBDCamera());
    //     frontend_->SetEventStore(data_reader_->GetEventStore());
    //     LOG(WARNING) << data_reader_->GetEventCamera()->K();
    // } break;
    // case 1: // rgbd mode
    // case 4: { // rgbd_live mode
    //     frontend_->SetRGBDCamera(dataset_->GetRGBDCamera());
    //     backend_->SetRGBDCamera(dataset_->GetRGBDCamera());
    // } break;
    // default:
    //     LOG(ERROR) << "Unknown mode!!!";
    //     break;
    // }

    LOG(INFO) << "set camera";
    return true;
}

void DynamicObjectsAvoidance::Run() {
    while (1) {
        // LOG(INFO) << "VO is running";
        if (Step() == false) {
            LOG(INFO)<<"step = false";
            break;
        }
    }

    // backend_->Stop();
    // viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool DynamicObjectsAvoidance::Step() {
    auto t3 = std::chrono::steady_clock::now();
    Frame::Ptr new_frame;
    if (data_reader_) {
        new_frame = data_reader_->NextFrame();
        // LOG(INFO)<<"data ready = "<<data_reader_->data_ready_;
        if (new_frame == nullptr) return false;
        // if (!data_reader_->data_ready_) return true; // data is not ready, skip
    }
    // // LOG(INFO)<<"store_rgbd_ = "<<store_rgbd_;
    // if (store_rgbd_) return true;
    // auto t4 = std::chrono::steady_clock::now();
    // auto time_used34 =
    //     std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
    // // LOG(INFO) << "Getting data cost time: " << time_used34.count() << " seconds.";

    // auto t1 = std::chrono::steady_clock::now();
    // bool success = frontend_->AddFrame(new_frame, data_reader_->data_ready_);
    // auto t2 = std::chrono::steady_clock::now();
    // auto time_used =
    //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // if (time_used.count()>1e-4) LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return true;
}

}  // namespace DynamicObjectsAvoidance
