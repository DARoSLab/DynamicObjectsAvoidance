#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"
#include <chrono>
#include "DynamicObjectsAvoidance/config.h"


// void Custom::UDPRecv()
// {
//     udp.Recv();
// }

// void Custom::UDPSend()
// {  
//     udp.Send();
// }

// // Run avoiding behavior
// void Custom::RobotControl() 
// {
//     motiontime += 2;
//     udp.GetRecv(state);
//     printf("[%d]  IMU STATE: %f\n", motiontime, state.imu.quaternion[2]);
//     // printf("[%f]  X Velocity: \n", DynamicObjectsAvoidance::DynamicObjectsAvoidance->xVel);

//     // Initialize
//     cmd.mode = 0;    
//     cmd.gaitType = 0;
//     cmd.speedLevel = 0;
//     cmd.footRaiseHeight = 0;
//     cmd.bodyHeight = 0;
//     cmd.euler[0]  = 0;
//     cmd.euler[1] = 0;
//     cmd.euler[2] = 0;
//     cmd.velocity[0] = 0.0f;
//     cmd.velocity[1] = 0.0f;
//     cmd.yawSpeed = 0.0f;
//     cmd.reserve = 0;

//     // Avoid based on time 
//     if(motiontime > 1000 && motiontime < 2000){
//         avoid_mode = Avoid_behavior::FACE_UP;
//     } else if (motiontime > 3000 && motiontime < 4000){
//         avoid_mode = Avoid_behavior::FACE_DOWN;
//     } else if (motiontime > 5000 && motiontime < 6000){
//         avoid_mode = Avoid_behavior::FACE_RIGHT;
//     } else if (motiontime > 7000 && motiontime < 8000){
//         avoid_mode = Avoid_behavior::FACE_LEFT;
//     } else if (motiontime > 9000 && motiontime < 10000){
//         avoid_mode = Avoid_behavior::BODY_DOWN;
//     } else {
//         avoid_mode = Avoid_behavior::INITIAL;
//     }

//     // Pitch control
//     if (avoid_mode == Avoid_behavior::FACE_UP){
//         cmd.mode = 1;
//         cmd.euler[1] = -0.2;
//     }

//     if (avoid_mode == Avoid_behavior::FACE_DOWN){
//         cmd.mode = 1;
//         cmd.euler[1] = 0.2;
//     }

//     // Yaw control
//     if (avoid_mode == Avoid_behavior::FACE_RIGHT){
//         cmd.mode = 1;
//         cmd.euler[2] = -0.2;
//     }

//     if (avoid_mode == Avoid_behavior::FACE_LEFT){
//         cmd.mode = 1;
//         cmd.euler[2] = 0.2;
//     }

//     // Body pos control
//     if (avoid_mode == Avoid_behavior::BODY_DOWN){
//         cmd.mode = 1;
//         cmd.bodyHeight = -0.2;
//     }

//     if (avoid_mode == Avoid_behavior::INITIAL){
//         cmd.mode = 1;
//         cmd.bodyHeight = 0.0;
//     }


    // // Body low
    // if(motiontime > 6000 && motiontime < 7000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = -0.2;
    // }

    // // Body high
    // if(motiontime > 7000 && motiontime < 8000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = 0.1;
    // }

    // // Body initial
    // if(motiontime > 8000 && motiontime < 9000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = 0.0;
    // }


    // MOTOR TORQUE
    // // Motor initial
    // if(motiontime > 9000 && motiontime < 11000){
    //     cmd.mode = 5;
    // }

    // // Motor torque
    // if(motiontime > 11000 && motiontime < 13000){
    //     cmd.mode = 6;
    // }

    // // Initialize
    // if(motiontime > 13000 && motiontime < 14000){
    //     cmd.mode = 0;
    // }

    // WALKING
    // if(motiontime > 14000 && motiontime < 18000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 2;
    //     cmd.velocity[0] = 0.4f; // -1  ~ +1
    //     cmd.yawSpeed = 2;
    //     cmd.footRaiseHeight = 0.1;
    //     // printf("walk\n");
    // }
    // if(motiontime > 18000 && motiontime < 20000){
    //     cmd.mode = 0;
    //     cmd.velocity[0] = 0;
    // }
    // if(motiontime > 20000 && motiontime < 24000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 1;
    //     cmd.velocity[0] = 0.2f; // -1  ~ +1
    //     cmd.bodyHeight = 0.1;
    // }
    
    // if(motiontime>24000 ){
    //     cmd.mode = 1;
    // }

//     udp.SetSend(cmd);
// }

namespace DynamicObjectsAvoidance {

DynamicObjectsAvoidance::DynamicObjectsAvoidance(std::string &config_path)
    : config_file_path_(config_path) {

    // if (Config::SetParameterFile(config_path) == false) {
    //     return false;
    // }
    // LOG(INFO)<<"config_path = "<<config_path;
    // std::string cfg_path = Config::Get<std::string>(std::string(config_path));
    // LOG(INFO)<<"cfg_path = "<<cfg_path;
    time_surface_ = std::make_unique<TimeSurface>(config_path,
                                320, 240, 1);

    time_surface_accumulator_ = std::make_unique<TimeSurfaceAccumulator>(cv::Size(320, 240));
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


// void DynamicObjectsAvoidance::InitCtrl() {
//     std::cout << "Communication level is set to HIGH-level." << std::endl
//               << "WARNING: Make sure the robot is standing on the ground." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     Custom custom(HIGHLEVEL);
//     // InitEnvironment();
//     LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
//     LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
//     LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

//     loop_udpSend.start();
//     loop_udpRecv.start();
//     loop_control.start();
// }

void DynamicObjectsAvoidance::Run() {
    // std::cout << "[HIGH-LEVEL] Communication" << std::endl
    //           << "WARNING: Make sure the robot is standing on the ground." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // // Add for safety check
    // // std::cin.ignore();

    // Custom custom(HIGHLEVEL);
    // // InitEnvironment();
    // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    // LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    // LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    // loop_udpSend.start();
    // loop_udpRecv.start();
    // loop_control.start();

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
    // Drawing
    cv::Point text_position(15, 30);
    double font_size = 0.5;
    cv::Scalar font_Color(255, 255, 0);
    int font_weight = 1;

    // Frame::Ptr new_frame;
    // if (data_reader_) {
    //     new_frame = data_reader_->NextFrame();
    //     if (new_frame == nullptr) return false;
    // }
    if (events_->getHighestTime() > 0) {
        slice_ = events_->sliceTime(events_->getHighestTime()-2000, events_->getHighestTime()-1);//.sliceBack(2000);
        // LOG(INFO)<<"event highest time = "<<events_->getHighestTime();
        // LOG(INFO)<<"events_->back() = "<<events_->back().timestamp();
        auto len = slice_.size();
        // LOG(INFO)<<"event size = "<<len;

        if (len > 10) {

            cv::Mat event_img = cv::Mat::zeros(240, 320, CV_8UC1);
            auto e = events_->back();
            // time_surface_->accept(slice_);

            uint64_t x(0),y(0);
            double cnt(0.01);
            for (uint64_t it = 0; it < slice_.size(); ++it) {
                // time_matrix_(store.at(it).y(), store.at(it).x()) = store.at(it).timestamp();
                
                event_img.ptr<uchar>(slice_.at(it).y())[slice_.at(it).x()] = 255;
            }
            cv::Mat img_med;
            cv::medianBlur(event_img, img_med, 5); // 3
            // cv::imshow("event_img", event_img);
            // cv::imshow("img_med", img_med);
            // cv::waitKey(1);
            // LOG(INFO)<<"img type = "<<img_med.type();

            for (int v = 0; v < 240; v++) {
				for (int u = 0; u < 320; u++) {
                    if (int(img_med.ptr<uchar>(v)[u]) > 250) {
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
            // LOG(INFO)<<"type = "<<time_surface_->time_surface_map_.type();
            cv::Mat ts_color;
            cv::cvtColor(img_med, ts_color, cv::COLOR_GRAY2BGR);
            cv::circle(ts_color, cv::Point(x, y), 5, cv::Scalar(0,0,255), 2, 8, 0);
            cv::Point2i curr_pos = cv::Point2i(x, y);
            cv::line(ts_color, pre_pos_, curr_pos, cv::Scalar(0, 255, 255), 1);

            // Calculate the direction of object
            if (pre_pos_.x - curr_pos.x > 0) {
                xVel = - (pre_pos_.x - curr_pos.x);
                yVel = - (pre_pos_.y - curr_pos.y);
                angle = atan (yVel/xVel) * 180 / PI;
                rCount++;
                lCount = 0;
            } else if (pre_pos_.x - curr_pos.x < 0) {
                xVel = - (pre_pos_.x - curr_pos.x);
                yVel = - (pre_pos_.y - curr_pos.y);
                angle = atan (yVel/xVel) * 180 / PI;
                lCount++;
                rCount = 0;
            }      

            if (rCount > Config::Get<int>("accumThreshold")) {
                putText(ts_color, "Left (" + std::to_string(int(xVel)) + ", " + std::to_string(int(yVel)) + ") " + std::to_string(angle), text_position, cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//
                drawCount++;
                if (drawCount > 20){
                    rCount = 0;
                    drawCount = 0;
                }
            } else if (lCount > Config::Get<int>("accumThreshold")) {
                putText(ts_color, "Right (" + std::to_string(int(xVel)) + ", " + std::to_string(int(yVel))+ ") " + std::to_string(angle), text_position, cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//
                drawCount++;
                if (drawCount > 20){
                    lCount = 0;
                    drawCount = 0;
                }
            } 

            pre_pos_ = curr_pos;

            static uint64_t saving_cnt(0);
            
            // Save image
            // cv::imwrite("/home/zh/data/dyn_detection/"+std::to_string(saving_cnt)+".png", ts_color);
            // cv::imwrite("/home/hochul/Repository/DynamicObjectsAvoidance/data/output/"+std::to_string(saving_cnt)+".png", ts_color);

            // LOG(INFO)<<"1";
            saving_cnt++;

            // // cv::namedWindow("ts img1", cv::WINDOW_NORMAL);
            // cv::namedWindow("Dynamic obstacle avoidance project", cv::WINDOW_NORMAL);
            // // cv::imshow("ts img1", event_img);
            cv::imshow("ts_color", ts_color);
            cv::waitKey(1);
            // LOG(INFO)<<"2";
        }
    }
    

    return true;
}

}  // namespace DynamicObjectsAvoidance
