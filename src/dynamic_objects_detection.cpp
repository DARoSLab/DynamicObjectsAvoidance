#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"
#include <chrono>
#include "DynamicObjectsAvoidance/config.h"

namespace DynamicObjectsAvoidance {

DynamicObjectsAvoidance::DynamicObjectsAvoidance(std::string &config_path)
    : config_file_path_(config_path) {

    // if (Config::SetParameterFile(config_path) == false) {
    //     return false;
    // }
    LOG(INFO)<<"1";
    // LOG(INFO)<<"config_path = "<<config_path;
    // std::string cfg_path = Config::Get<std::string>(std::string(config_path));
    // LOG(INFO)<<"cfg_path = "<<cfg_path;
    // LOG(INFO)<<"2";
    time_surface_ = std::make_unique<TimeSurface>(config_path,
                                320, 240, 1);
    LOG(INFO)<<"3";
    time_surface_accumulator_ = std::make_unique<TimeSurfaceAccumulator>(cv::Size(320, 240));
    LOG(INFO)<<"4";
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
    case 5: { // event_live mode
        data_reader_ = DataReader::Ptr(new DataReader(Config::Get<std::string>("cfg_file_path")));
        CHECK_EQ(data_reader_->InitEventCamera(), true);
        SetEventStore(data_reader_->GetEventStore());
    } break;

    default:
        LOG(ERROR) << "Unknown mode!!!";
        break;
    }

    return true;
}

void DynamicObjectsAvoidance::Run() {
    while (1) {
        // LOG(INFO) << "Dynamic Objects Avoidance is running";
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
    // Frame::Ptr new_frame;
    // if (data_reader_) {
    //     new_frame = data_reader_->NextFrame();
    //     if (new_frame == nullptr) return false;
    // }
    if (events_->getHighestTime() > 0) {
        slice_ = events_->sliceTime(events_->getHighestTime()-2000, events_->getHighestTime()-1);//.sliceBack(2000);
        LOG(INFO)<<"event highest time = "<<events_->getHighestTime();
        LOG(INFO)<<"events_->back() = "<<events_->back().timestamp();
        auto len = slice_.size();
        LOG(INFO)<<"event size = "<<len;
        if (len > 10) {
            cv::Mat event_img = cv::Mat::zeros(240, 320, CV_8UC1);
            auto e = events_->back();
            LOG(INFO)<<"1";
            time_surface_->accept(slice_);
            LOG(INFO)<<"2";

            uint64_t x(0),y(0),cnt(1);
            for (uint64_t it = 0; it < slice_.size(); ++it) {
                // time_matrix_(store.at(it).y(), store.at(it).x()) = store.at(it).timestamp();
                
                event_img.ptr<uchar>(slice_.at(it).y())[slice_.at(it).x()] = 255;
            }
            cv::Mat img_med;
            cv::medianBlur(event_img, img_med, 3);
            cv::imshow("event_img", event_img);
            cv::imshow("img_med", img_med);
            cv::waitKey(1);

            for (int16_t v = 0; v < 240; y++) {
				for (int16_t u = 0; u < 320; x++) {
                    if (event_img.ptr<uchar>(v)[u] > 200) {
                        x += u;
                        y += v;
                        cnt++;
                    }

                }
            }


            x /= cnt;
            y /= cnt;
            // LOG(INFO)<<"xy = "<<x<<"  "<<y;



            // auto time = events_->getHighestTime()-1;
            // time_surface_->createTimeSurface(time);
            // LOG(INFO)<<"3";
            LOG(INFO)<<"type = "<<time_surface_->time_surface_map_.type();
            cv::Mat ts_color;
            cv::cvtColor(img_med, ts_color, cv::COLOR_GRAY2BGR);
            cv::circle(ts_color, cv::Point(x, y), 5, cv::Scalar(0,0,255), 2, 8, 0);


            cv::Point2i curr_pos = cv::Point2i(x, y);

            cv::line(ts_color, pre_pos_, curr_pos, cv::Scalar(0, 255, 255), 1);

            pre_pos_ = curr_pos;

            static uint64_t saving_cnt(0);
            LOG(INFO)<<"0";
            cv::imwrite("/home/zh/data/dyn_detection/"+std::to_string(saving_cnt)+".png", ts_color);
            LOG(INFO)<<"1";
            saving_cnt++;

            // // cv::namedWindow("ts img1", cv::WINDOW_NORMAL);
            // cv::namedWindow("ts_color", cv::WINDOW_NORMAL);
            // // cv::imshow("ts img1", event_img);
            // cv::imshow("ts_color", ts_color);
            // cv::waitKey(1);
            // LOG(INFO)<<"2";
        }
    }
    

    return true;
}

}  // namespace DynamicObjectsAvoidance
