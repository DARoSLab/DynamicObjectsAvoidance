#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"
#include <chrono>
#include "DynamicObjectsAvoidance/config.h"

namespace DynamicObjectsAvoidance {

DynamicObjectsAvoidance::DynamicObjectsAvoidance(std::string &config_path)
    : config_file_path_(config_path) {
    time_surface_ = std::make_unique<TimeSurface>(config_path,
                                320, 240, 1);
    time_surface_accumulator_ = std::make_unique<TimeSurfaceAccumulator>(cv::Size(320, 240));
    imshowthread_ = std::thread(&DynamicObjectsAvoidance::ImshowThread, this);
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

    LOG(INFO) << "Dynamic Objects Avoidance Exit";
}

void DynamicObjectsAvoidance::ImshowThread() {
    LOG(INFO)<<"imshow thread";
    cv::Mat event_img = cv::Mat::zeros(240, 320, CV_8UC1);
    cv::imshow("key_imshow", event_img);
    while (1) {
        key = cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::microseconds(10000)); // TODO: need to fix this
    }
}

void fitLineRansac(const std::vector<cv::Point2f>& points,
                   cv::Vec4f &line,
                   int iterations = 300,
                   double sigma = 1.,
                   double k_min = -15.,
                   double k_max = 15.)
{
    unsigned int n = points.size();

    if(n<2)
    {
        return;
    }

    cv::RNG rng;
    double bestScore = -1.;
    for(int k=0; k<iterations; k++)
    {
        int i1=0, i2=0;
        while(i1==i2)
        {
            i1 = rng(n);
            i2 = rng(n);
        }
        const cv::Point2f& p1 = points[i1];
        const cv::Point2f& p2 = points[i2];

        cv::Point2f dp = p2-p1;
        dp *= 1./norm(dp);
        double score = 0;

        if(dp.y/dp.x<=k_max && dp.y/dp.x>=k_min )
        {
            for(int i=0; i<n; i++)
            {
                cv::Point2f v = points[i]-p1;
                double d = v.y*dp.x - v.x*dp.y;
                //score += exp(-0.5*d*d/(sigma*sigma));
                if( fabs(d)<sigma )
                    score += 1;
            }
        }
        if(score > bestScore)
        {
            line = cv::Vec4f(dp.x, dp.y, p1.x, p1.y);
            bestScore = score;
        }
    }
}

cv::Mat DynamicObjectsAvoidance::GetImage() {
    return frame;
}

bool DynamicObjectsAvoidance::Step() {

    // auto t3 = std::chrono::steady_clock::now();
    
    // Drawing
    cv::Point text_position(15, 30);
    double font_size(0.5);
    cv::Scalar font_Color(0, 255, 255);
    int font_weight(1);

    if (events_->getHighestTime() > 0) {

        slice_ = events_->sliceTime(events_->getHighestTime()-((int) Config::Get<int>("accumTime")), events_->getHighestTime()-1);//.sliceBack(2000);
        auto len = slice_.size();
        // LOG(INFO)<<"event size = "<<len;
        // initAction = false;


        

        if (len > (int) Config::Get<int>("eventCount") && key == 32) {
            // initAction = false;
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
            cv::medianBlur(event_img, img_med, 3); // 3
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
            
            cv::Mat ts_color;

            if (x != 0 || y != 0){
                cv::cvtColor(img_med, ts_color, cv::COLOR_GRAY2BGR);
                cv::circle(ts_color, cv::Point(x, y), 5, cv::Scalar(0,0,255), 2, 8, 0);
                cv::Point2i curr_pos = cv::Point2i(x, y);
                cv::Point2f curr_posf = cv::Point2f(x, y);
                posVector.push_back(curr_posf);
                if (posVector.size() > 100) {
                    posVector.erase(posVector.begin(), posVector.begin() + posVector.size() - 100);
                }
                // LOG(INFO)<<"pos size = "<<posVector.size();
                cv::Vec4f line;
                fitLineRansac(posVector, line);

                double k = line[1] / line[0];
                double b = line[3] - k*line[2];

                cv::Point p1,p2,p3;
                // current
                p1.y = y;
                p1.x = x;
                // bottom
                p2.y = 239;
                p2.x = (p2.y-b) / k;
                // top
                p3.y = 1;
                p3.x = (p3.y-b) / k;
                // cv::line(ts_color,p1,p2,cv::Scalar(0,0,255),2);
                // cv::line(ts_color,p1,p3,cv::Scalar(0,0,255),2);

                cv::line(ts_color, pre_pos_, curr_pos, cv::Scalar(0, 255, 255), 1);

                cv::line(ts_color, cv::Point(160-(int) Config::Get<int>("robotRadius"), 240), cv::Point(160+(int) Config::Get<int>("robotRadius"), 240), cv::Scalar(0, 0, 255), 4);

                // Pixel-wise velocity
                xVel = pre_pos_.x - curr_pos.x;
                yVel = pre_pos_.y - curr_pos.y;
                // angle = atan (yVel/xVel) * 180 / PI;

                // Quadrant-wise pixel velocity
                if (xVel > 0) {
                    if (yVel > 0) { // 2nd quadrant
                        tmpDirection = 2;
                    } else { // 3rd quadrant
                        tmpDirection = 3;
                    }
                } else if (xVel < 0) { 
                    if (yVel > 0) { // 1st quadrant
                        tmpDirection = 1;
                    } else { // 4th quadrant
                        tmpDirection = 4;
                    }
                }  

                dirQueue.push(tmpDirection);

                // Extract major direction based on window
                if (dirQueue.size() == (int) Config::Get<int>("windowSize")) {
                    std::fill(dirCounts.begin(), dirCounts.end(), 0);
                    for (int i = 0; i < dirQueue.size(); i++) {
                        dirCounts[dirQueue.front()]++;
                        dirQueue.push(dirQueue.front());
                    }
                    for (int j=0; j<dirCounts.size(); j++) {
                        if (dirCounts[j] >= (int) Config::Get<int>("windowSize")/2 +1) {
                            majorDirection = j;
                        }
                    }

                    // There is a major direction and object is not stationary
                    if (majorDirection != 0 && (curr_pos.x-pre_pos_.x != 0 || curr_pos.y-pre_pos_.y != 0)) {
                        auto direction = cv::Point(int(((curr_pos.x-pre_pos_.x)/(sqrt(pow(curr_pos.x-pre_pos_.x, 2) 
                            + pow(curr_pos.y-pre_pos_.y, 2))))*40), int(((curr_pos.y-pre_pos_.y)/(sqrt(pow(curr_pos.x-pre_pos_.x, 2) 
                            + pow(curr_pos.y-pre_pos_.y, 2))))*40));
                        cv::arrowedLine(ts_color, curr_pos, curr_pos+direction, cv::Scalar(0, 255, 0), 2);
                        cv::arrowedLine(ts_color, curr_pos, p2, cv::Scalar(255, 0, 0), 2);
                        cv::arrowedLine(ts_color, curr_pos, p3, cv::Scalar(255, 0, 0), 2);
                        
                        // // Trajectory prediction
                        // if (int(direction.x) != 0 && int(direction.y) != 0) {
                        //     // Faling obstacle
                        //     if (yVel < 0){
                        //         double inclination = double(direction.y)/double(direction.x);
                        //         // Pedicted ground collision (x, 240) -> pixel frame
                        //         groundX = int((240-curr_pos.y)/inclination+curr_pos.x);
                        //         auto extendedDirection = cv::Point(groundX, 240); // y = -a(x-X)-Y
                        //         cv::arrowedLine(ts_color, curr_pos, extendedDirection, cv::Scalar(255, 255, 0), 1);    
                        //     // Rising obstacle
                        //     } 
                            // else if (yVel > 0){
                            //     double inclination = double(direction.y)/double(direction.x);
                            //     // Pedicted ground collision (x, 240) -> pixel frame
                            //     topX = int((-curr_pos.y/inclination)+curr_pos.x);
                            //     groundX = int(-240/inclination+topX);
                            //     auto extendedTopDirection = cv::Point(topX, 0); // y = -a(x-X)-Y
                            //     auto extendedBottomDirection = cv::Point(groundX, 240); // y = -a(x-X)-Y
                            //     cv::line(ts_color, curr_pos, extendedTopDirection, cv::Scalar(255, 255, 0), 1);  
                            //     cv::arrowedLine(ts_color, extendedTopDirection, extendedBottomDirection, cv::Scalar(255, 255, 0), 1);    
                            // }

                            // Define collision w/ robot
                            // if (160 - (int) Config::Get<int>("robotRadius") < groundX && 160 + (int) Config::Get<int>("robotRadius") > groundX) {
                            //     collision = true;
                            // } else {
                            //     collision = false;
                            // }

                        if ((160 - (int) Config::Get<int>("robotRadius") < p2.x && 160 + (int) Config::Get<int>("robotRadius") > p2.x && p2.y == 239) || (160 - (int) Config::Get<int>("robotRadius") < p2.x && 160 + (int) Config::Get<int>("robotRadius") > p2.x && p2.y == 1)) {
                            collision = true;
                        } else {
                            collision = false;
                        }


                        // Compare collision & send final command
                        // if (collision && groundX > 160 && initAction == false) {
                        // if (collision && xVel >0 && initAction == false) {
                        if (collision && (p2.x < 160 || p3.x < 160) && initAction == false) {
                            // inited_ = true;
                            cmdDir = 0.7f; // robot left command
                            initAction = true;
                            printf("\nHit from right -> GO Left\n");
                            // frame = ts_color;
                            // cv::Mat GetImage(ts_color);
                            // printf("%f\n", cmdDir);
                            // putText(ts_color, "Collision! CMD Left", text_position, cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//

                        // } else if (collision && groundX < 160 && initAction == false) {
                        // } else if (collision && xVel < 0 && initAction == false) {                                
                        } else if (collision && (p2.x >= 160 || p3.x >= 160)&& initAction == false) {                                
                            // inited_ = true;
                            cmdDir = -0.7f; // robot right command
                            initAction = true;
                            printf("\nHIt from left -> GO Right\n");
                            // frame = ts_color;
                            // cv::Mat GetImage(ts_color);

                            // printf("%f\n", cmdDir);
                            // putText(ts_color, "Collision! CMD Right", text_position, cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//
                        } 
                            
                            // initAction = false;
                        // }

                    }

                    pre_pos_ = curr_pos;

                    cv::imshow("ts_color", ts_color);
                    cv::waitKey(1);         
                }    
            } else { // No events
                xVel = 0;
                yVel = 0;
                    // initAction = false;

                // angle = 0;
            }
            
            // Drawing for demo
            // if (rCount > Config::Get<int>("accumThreshold")) {
            //     putText(ts_color, "Left (" + std::to_string(int(xVel)) + ", " + std::to_string(int(yVel)) + ") " + std::to_string(angle), text_position, cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//
            //     drawCount++;
            //     if (drawCount > 20){
            //         rCount = 0;
            //         drawCount = 0;
            //     }
            // } else if (lCount > Config::Get<int>("accumThreshold")) {
            //     putText(ts_color, "Right (" + std::to_string(int(xVel)) + ", " + std::to_string(int(yVel))+ ") " + std::to_string(angle), text_position, cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//
            //     drawCount++;
            //     if (drawCount > 20){
            //         lCount = 0;
            //         drawCount = 0;
            //     }
            // } 

            // static uint64_t saving_cnt(0);
            
            // Save image
            // cv::imwrite("/home/zh/data/dyn_detection/"+std::to_string(saving_cnt)+".png", ts_color);
            // cv::imwrite("/home/hochul/Repository/DynamicObjectsAvoidance/data/output/"+std::to_string(saving_cnt)+".png", ts_color);

            // saving_cnt++;

            // // cv::namedWindow("ts img1", cv::WINDOW_NORMAL);
            // cv::namedWindow("Dynamic obstacle avoidance project", cv::WINDOW_NORMAL);
            // // cv::imshow("ts img1", event_img);

        }

        // If avoidance motion is triggered
        if (initAction == true){
            motionCnt++;
            // printf("motionCnt: %d", motionCnt);
            // printf("cnt: %d", motionCnt);
            // printf("Motion Count: %d\n", motionCnt);
            // End motion
            if (motionCnt > 1000) {
                initAction = false;
                motionCnt = 0;
                cmdDir = 0.0f;
                // printf("\nMotion ended\n");
            }
        }
        // initAction = false;

    }
    
    return true;
}

}  // namespace DynamicObjectsAvoidance
