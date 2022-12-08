#include "DynamicObjectsAvoidance/data_reader.h"
#include "DynamicObjectsAvoidance/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

namespace DynamicObjectsAvoidance {

DataReader::DataReader(const std::string& cfg_path) : cfg_path_(cfg_path) {
    std::cout << "construct datareader " << std::endl;
    if (Config::SetParameterFile(cfg_path) == false) {
        LOG(ERROR) << "Cannot find config parameter file";
    }
    mode_ = Config::Get<int>("mode");
    visualize_alignment_ = Config::Get<int>("visualize_alignment");
    pyramid_layers_ = Config::Get<int>("pyramid_layers");
    exposure_ = Config::Get<int>("exposure");
    pyramid_ratio_ = Config::Get<double>("pyramid_ratio");
    event_dataset_path_ = Config::Get<std::string>("event_dataset_path");
    rgbd_dataset_path_ = Config::Get<std::string>("rgbd_dataset_path");
    std::string timestamp_path = Config::Get<std::string>("timestamp_path");
    rgbd_timestamp_file_out_.open(timestamp_path, std::ios::app);
    rgbd_timestamp_file_out_.precision(6);
    rgbd_timestamp_file_out_.flags(std::ios::fixed);
    rgbd_timestamp_file_out_.fill('0');
    accumulator_ = std::make_unique<dv::Accumulator>(cv::Size(320, 240));
    accumulator_->setEventContribution(static_cast<float>(0.15));
    accumulator_->setMaxPotential(static_cast<float>(1.0));
    accumulator_->setMinPotential(static_cast<float>(0.0));
    accumulator_->setNeutralPotential(static_cast<float>(0.0));
    accumulator_->setDecayParam(static_cast<float>(1e+6));
    accumulator_->setRectifyPolarity(false);
    accumulator_->setSynchronousDecay(false);
    accumulator_->setDecayFunction(static_cast<dv::Accumulator::Decay>(dv::Accumulator::Decay::STEP));

    pixel_accumulator_ = std::make_unique<dv::PixelAccumulator>(cv::Size(320, 240));
    pixel_accumulator_->setIgnorePolarity(true);
    pixel_accumulator_->setContribution(static_cast<float>(1)); // 0.3
    pixel_accumulator_->setNeutralValue(static_cast<float>(0.0));
    pixel_accumulator_->setDecay(static_cast<float>(0.01)); // 0.3

    mEventBuffer_ = new dv::EventStore;
    int64_queue_ = new std::queue<int64_t>;
    time_surface_mine_ = std::make_unique<TimeSurface>(cfg_path,
                                    320, 240, 10);
}

DataReader::DataReader(const std::string& event_dataset_path, const std::string& rgbd_dataset_path, 
                        const std::string& cfg_path) : event_dataset_path_(event_dataset_path), 
                        rgbd_dataset_path_(rgbd_dataset_path), cfg_path_(cfg_path) {
    std::cout << "construct datareader " << std::endl;
    if (Config::SetParameterFile(cfg_path) == false) {
        LOG(ERROR) << "Cannot find config parameter file";
    }
    mode_ = Config::Get<int>("mode");
    visualize_alignment_ = Config::Get<int>("visualize_alignment");
    pyramid_layers_ = Config::Get<int>("pyramid_layers");
    pyramid_ratio_ = Config::Get<double>("pyramid_ratio");
    exposure_ = Config::Get<int>("exposure");
    std::string timestamp_path = Config::Get<std::string>("timestamp_path");
    rgbd_timestamp_file_out_.open(timestamp_path, std::ios::app);
    rgbd_timestamp_file_in_.open(timestamp_path, std::ios::app);
    accumulator_ = std::make_unique<dv::Accumulator>(cv::Size(320, 240));
    accumulator_->setEventContribution(static_cast<float>(0.15));
    accumulator_->setMaxPotential(static_cast<float>(1.0));
    accumulator_->setMinPotential(static_cast<float>(0.0));
    accumulator_->setNeutralPotential(static_cast<float>(0.0));
    accumulator_->setDecayParam(static_cast<float>(1e+6));
    accumulator_->setRectifyPolarity(false);
    accumulator_->setSynchronousDecay(false);
    accumulator_->setDecayFunction(static_cast<dv::Accumulator::Decay>(dv::Accumulator::Decay::STEP));

    pixel_accumulator_ = std::make_unique<dv::PixelAccumulator>(cv::Size(320, 240));
    pixel_accumulator_->setIgnorePolarity(true);
    pixel_accumulator_->setContribution(static_cast<float>(0.3));
    pixel_accumulator_->setNeutralValue(static_cast<float>(0.0));
    pixel_accumulator_->setDecay(static_cast<float>(0.3));

    mEventBuffer_ = new dv::EventStore;
    time_surface_mine_ = std::make_unique<TimeSurface>(cfg_path,
                                    320, 240, 10);
}

DataReader::~DataReader() {
    rgbd_timestamp_file_out_.close();
    mSpinThread_ = false;
}

void DataReader::Clock(int64_t start, int64_t end, int64_t timeIncrement) {
	double frequency = 1.0 / (static_cast<double>(timeIncrement) * 1e-6);
	while (mSpinThread_) {
        mFrameTimestampQueue_.push(start);
        mEventsTimestampQueue_.push(start);
        mImuTimestampQueue_.push(start);
        start += timeIncrement;
        std::this_thread::sleep_for(std::chrono::microseconds(930)); // TODO: need to fix this
		// EOF or reader is disconnected
		if (start >= end) {
			mSpinThread_ = false;
		}
	}
}

void DataReader::FrameReadingThread() {
    double alphaValue = 0.2;
    double betaValue = 1.0 - alphaValue;
    cv::Mat alignment_result;
    std::optional<dv::Frame> frame = std::nullopt;
    while (mSpinThread_) {
		mFrameTimestampQueue_.consume_all([&](const int64_t timestamp) {
            if (!frame.has_value()) {
				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex_); // lock mEventReader_
				frame = mEventReader_.getNextFrame();
			}
            while (frame.has_value() && timestamp >= (frame->timestamp+exposure_)) {
                cv::Mat img_undistort;
                if ((*frame).image.channels()>1) {
                    cv::cvtColor((*frame).image, img_undistort, cv::COLOR_BGR2GRAY); // convert from BGR to gray
                } else {
                    img_undistort = (*frame).image.clone();
                }
                // cv::imshow("frame.image", img_undistort);
                // cv::waitKey(0);
                cv::remap(img_undistort, img_undistort, event_camera_->remap_before_, event_camera_->remap_after_, cv::INTER_LINEAR);
                mRGBQueue_.push(dv::Frame(((*frame).timestamp+exposure_), img_undistort)); // set the half exposure as timestamp
                LOG(INFO)<<"get new frame, timestamp ========== "<<std::setprecision(16)<<((*frame).timestamp+exposure_)*1e-6;
                image_left = img_undistort.clone();
				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex_);
                frame = mEventReader_.getNextFrame();
            }
        });
    }
}

void DataReader::EventsReadingThread() {
    // const auto &live_capture = mEventReader_.getCameraCapturePtr();
    // if (live_capture) {
        std::optional<dv::Frame> frame = std::nullopt;
        std::optional<dv::EventStore> events = std::nullopt;
        cv::Mat img_undistort, img_pxl_acc_un, img_pxl_acc_un_blur;
        double alphaValue = 0.2;
        double betaValue = 1.0 - alphaValue;
        cv::Mat alignment_result;
        int64_t int_cnt = 0;
        while (mSpinThread_) {
            // LOG(INFO) << "mSpinThread_";
            // if (!frame.has_value()) {
			// 	frame = mEventReader_.getNextFrame();
			// }
            // if (frame.has_value()) {
            //     time_rgb_ = (*frame).timestamp / 1000000.0;
            //     cv::remap((*frame).image, img_undistort, cameras_[0]->remap_before_, cameras_[0]->remap_after_, cv::INTER_LINEAR);
            //     // mDataQueue_.push(dv::Frame((*frame).timestamp, img_undistort));
            //     mRGBBuffer_.push(dv::Frame((*frame).timestamp, img_undistort));
            //     frame = mEventReader_.getNextFrame();
            // }
            // mEventsTimestampQueue_.consume_all([&](const int64_t timestamp) {

                if (!events.has_value()) {
				    std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex_); // lock mEventReader_
                    events = mEventReader_.getNextEventBatch();
                }
                while (events.has_value()) {
                    time_event_ = (*events).getLowestTime();
                    dv::EventStore store;
                    if (mNoiseFilter_ != nullptr) {
                        mNoiseFilter_->accept(*events);
                        store = mNoiseFilter_->generateEvents();
                        // store = *events;
                    } else {
                        store = *events;
                    }
                    mEventBuffer_->add(store);
                    mEventBuffer_->retainDuration(mStoreTimeLimit_);
				    std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex_); // lock mEventReader_
                    events = mEventReader_.getNextEventBatch();
                    // LOG(INFO) << "Reading event";
                }
            // });
        }
    // }
}

void DataReader::ImuReadingThread() {
    std::optional<dv::Frame> frame = std::nullopt;
    std::optional<dv::EventStore> events = std::nullopt;
    std::optional<dv::cvector<dv::IMU>> imus = std::nullopt;
    cv::Mat img_undistort, img_pxl_acc_un, img_pxl_acc_un_blur;
    double alphaValue = 0.2;
    double betaValue = 1.0 - alphaValue;
    cv::Mat alignment_result;
    int64_t int_cnt = 0;
    while (mSpinThread_) {
        mImuTimestampQueue_.consume_all([&](const int64_t timestamp) {

            if (!imus.has_value()) {
                std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex_); // lock mEventReader_
                imus = mEventReader_.getNextImuBatch();
            }
            while (imus.has_value() && timestamp >= imus.value().back().timestamp) {
                double t(0.), gx(0.), gy(0.), gz(0.), ax(0.), ay(0.), az(0.);
                uint16_t imu_cnt(0);

                for (auto &imu : *imus) {
                    // if (imu_cnt == 0) LOG(INFO)<<"t = "<<imu.timestamp;
                    gx += imu.gyroscopeX;
                    gy += imu.gyroscopeY;
                    gz += imu.gyroscopeZ;
                    ax += imu.accelerometerX;
                    ay += imu.accelerometerY;
                    az += imu.accelerometerZ;
                    imu_cnt++;
                }
                static int has_data_cnt(0);
                if (!get_imu_bias_ && has_data_cnt < 1000) {
                    has_data_cnt++;
                    if (has_data_cnt > 10) {
                        init_gx_ += (gx/imu_cnt); init_gy_ += (gy/imu_cnt); init_gz_ += (gz/imu_cnt);
                        init_ax_ += (ax/imu_cnt); init_ay_ += (ay/imu_cnt); init_az_ += (az/imu_cnt);
                    }
                    if (has_data_cnt == 1000) {
                        init_gx_ /= (has_data_cnt-10); init_gy_ /= (has_data_cnt-10); init_gz_ /= (has_data_cnt-10); 
                        init_ax_ /= (has_data_cnt-10); init_ay_ /= (has_data_cnt-10); init_az_ /= (has_data_cnt-10); 
                        get_imu_bias_ = true;
                    }
                }
                
                if (has_data_cnt == 1000) {
                    gx /= imu_cnt; gy /= imu_cnt; gz /= imu_cnt;
                    gx -= init_gx_; gy -= init_gy_; gz -= init_gz_;
                    Eigen::Vector3d ang_vel(gx, gy, gz);
                    ax /= imu_cnt; ay /= imu_cnt; az /= imu_cnt;
                    ax -= init_ax_; ay -= init_ay_; az -= init_az_;
                    curr_imu_timestamp_ = imus.value().back().timestamp;
                    static Eigen::Vector3d lin_vel(0., 0., 0.);
                    Eigen::Vector3d acc(ax, ay, az);
                    if (last_imu_timestamp_ != 0) {
                        lin_vel += acc * (curr_imu_timestamp_ - last_imu_timestamp_)* 1e-6;
                    }
                    last_imu_timestamp_ = curr_imu_timestamp_;
                    linear_vel_ = lin_vel;
                    angle_vel_ = ang_vel;
                    auto ang = std::make_pair(curr_imu_timestamp_, ang_vel.norm());
                    auto vel = std::make_pair(curr_imu_timestamp_, lin_vel.norm());
                    mImuAngVelQueue_.push(ang);
                    mImuLinVelQueue_.push(vel);
                }

                std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex_); // lock mEventReader_
                imus = mEventReader_.getNextImuBatch();
            }
        });
    }
}

void DataReader::RGBDDataReadingThread() {
    using namespace std::chrono_literals;
    rs2_error* e = 0;
    int count = 0;
    double diff = 0.0, diff_avg = 0.0;
    // boost::format fmt("%s/image_depth_record/%06d.png"); // 10 us
    boost::format fmt("%s/image_depth_record/%s.png"); // 10 us
    while (mSpinThread_) {
        // for live mode
        if (mode_ == 3 || mode_ == 4) {
            frameset_ = pipe_.wait_for_frames(150); // wait for 15ms
            auto depth_frame = frameset_.get_depth_frame();
            auto color_frame = frameset_.get_color_frame();

            count++;
            if (count > 300) {
                diff += (time_event_/1000000 - depth_frame.get_timestamp()/1000.0);
            }
            if (count == 600) {
                diff_avg = diff / 300.0;
                diff = 0;
                count = 0;
            }

            cv::Mat depth_img_rs = Frame2Mat(depth_frame);
            cv::Mat depth_img = cv::Mat::zeros(event_camera_->height_, event_camera_->width_, CV_16UC1);
            ushort* output_data = reinterpret_cast<ushort*> (depth_img.data);
            const ushort* input_data = reinterpret_cast<ushort*> (depth_img_rs.data);
            SE3 extrinsic = rgbd_camera_->depth_to_event_;
            ImageAlignment(rgbd_camera_, event_camera_, input_data, output_data, extrinsic);
            // cv::imshow("depth", depth_img);
            // mDataQueue_.push(depth_img);
            mDepthBuffer_.push(dv::Frame(time_event_ - rs_exposure_*0.5, depth_img));
            mDepthQueue_.push(dv::Frame(time_event_ - rs_exposure_*0.5, depth_img));
            image_depth = depth_img.clone();
        // for the offline mode
        } else if (mode_ == 1 || mode_ == 2) {
            char rgbd_time_buffer[256];
            double rgbd_time;
            rgbd_timestamp_file_in_.getline(rgbd_time_buffer, 100);
            rgbd_time = std::atof(rgbd_time_buffer);
            cv::Mat depth_img = cv::imread((fmt % rgbd_dataset_path_ % rgbd_time_buffer).str(), cv::IMREAD_UNCHANGED);
            current_image_index_++;
            mDepthBuffer_.push(dv::Frame(int64_t(rgbd_time*1000000.0), depth_img));
            mDepthQueue_.push(dv::Frame(int64_t(rgbd_time*1000000.0), depth_img));
    		std::this_thread::sleep_for(9ms);
                        
            // // test depth blur by using events as ground truth
    		// // std::this_thread::sleep_for(15ms);
            // static uint64_t ed_cnt(0);
            // double alphaValue = 0.5;
            // double betaValue = 1.0 - alphaValue;
            // cv::Mat alignment_result;
            // auto t0 = std::chrono::steady_clock::now();
            // int64_t time = int64_t(rgbd_time*1000000.0);
            // auto slice = mEventBuffer_->sliceTime(time-3000, time+3000);//.sliceBack(number_of_events_);
            // LOG(INFO)<<"mEventBuffer_"<<mEventBuffer_->getHighestTime() << "  "<<mEventBuffer_->getLowestTime();
            // LOG(INFO)<<"time = "<<time;
            // LOG(INFO)<<"Total count = "<<slice.getTotalLength();
            // pixel_accumulator_->accept(slice);
            // dv::Frame pxl_acc = pixel_accumulator_->generateFrame();
            // cv::Mat img_pxl_acc_un;
            // cv::remap(pxl_acc.image, img_pxl_acc_un, GetEventCamera()->remap_before_, GetEventCamera()->remap_after_, cv::INTER_LINEAR);
            // cv::imshow("img_pxl_acc_un", img_pxl_acc_un);
            // cv::waitKey(1);
            // // cv::blur(img_pxl_acc_un, img_pxl_acc_un_blur, cv::Size(7,7));
            // auto t1 = std::chrono::steady_clock::now();
            // auto time_used1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
            // LOG(INFO) << "pixel_accumulator_ cost time: " << time_used1.count() << " seconds.";
            // depth_img.convertTo(depth_img,CV_8UC1, 1.0/255.0);
            // LOG(INFO)<<"depthtype = "<<depth_img.type()<<"  rgbtype="<<img_pxl_acc_un.type();
            // LOG(INFO)<<depth_img.cols<<" "<<depth_img.rows<<" "<<img_pxl_acc_un.cols<<" "<<img_pxl_acc_un.rows;
            // // cv::cvtColor(image_rgb_event_un, image_rgb_event_un, cv::COLOR_BGR2GRAY);
            // if (depth_img.cols!=346||depth_img.rows!=260||img_pxl_acc_un.cols!=346||img_pxl_acc_un.rows!=260) {
            //     LOG(INFO)<<"dimension error";
            // } else {
            //     cv::addWeighted(img_pxl_acc_un, alphaValue, depth_img, betaValue, 0.0, alignment_result);
            //     LOG(INFO)<<"2";
            //     cv::imwrite("/home/zh/data/img/event_depth/"+std::to_string(ed_cnt)+".png", alignment_result);
            //     LOG(INFO)<<"/home/zh/data/img/event_depth/"+std::to_string(ed_cnt)+".png";
            //     ++ed_cnt;
            // }
            
            // // cv::namedWindow("alignment", cv::WINDOW_NORMAL);
            // // cv::imshow("alignment", alignment_result);
            // // cv::waitKey(1);
        }
    }
}

bool DataReader::InitRGBD() {
    // read camera intrinsics and extrinsics
    double color_fx = Config::Get<double>("rgbdcameracolor.fx");
    double color_fy = Config::Get<double>("rgbdcameracolor.fy");
    double color_cx = Config::Get<double>("rgbdcameracolor.cx");
    double color_cy = Config::Get<double>("rgbdcameracolor.cy");
    double color_k0 = Config::Get<double>("rgbdcameracolor.coeff0");
    double color_k1 = Config::Get<double>("rgbdcameracolor.coeff1");
    double color_k2 = Config::Get<double>("rgbdcameracolor.coeff2");
    double color_k3 = Config::Get<double>("rgbdcameracolor.coeff3");
    double color_k4 = Config::Get<double>("rgbdcameracolor.coeff4");

    double depth_fx = Config::Get<double>("rgbdcameradepth.fx");
    double depth_fy = Config::Get<double>("rgbdcameradepth.fy");
    double depth_cx = Config::Get<double>("rgbdcameradepth.cx");
    double depth_cy = Config::Get<double>("rgbdcameradepth.cy");

    double rgbddepth2color_qx = Config::Get<double>("rgbddepth2color.qx");
    double rgbddepth2color_qy = Config::Get<double>("rgbddepth2color.qy");
    double rgbddepth2color_qz = Config::Get<double>("rgbddepth2color.qz");
    double rgbddepth2color_qw = Config::Get<double>("rgbddepth2color.qw");
    double rgbddepth2color_tx = Config::Get<double>("rgbddepth2color.x");
    double rgbddepth2color_ty = Config::Get<double>("rgbddepth2color.y");
    double rgbddepth2color_tz = Config::Get<double>("rgbddepth2color.z");

    depth_scale_ = Config::Get<double>("rgbdcamera.depth_scale");
    // extrinsic matrix from event camera rgb image to depth rgb image
    double qx = Config::Get<double>("rgbdcolor2event_qx");
    double qy = Config::Get<double>("rgbdcolor2event_qy");
    double qz = Config::Get<double>("rgbdcolor2event_qz");
    double qw = Config::Get<double>("rgbdcolor2event_qw");
    double tx = Config::Get<double>("rgbdcolor2event_tx");
    double ty = Config::Get<double>("rgbdcolor2event_ty");
    double tz = Config::Get<double>("rgbdcolor2event_tz");

    // intrinsic parameters and distortion parameters
    float K_color[4], K_depth[4];
    float* D_color = new float[5];
    float* D_depth = new float[5];

    int width, height;

    if (mode_ == 3 || mode_ == 4) {
        if (!serial_.empty()) {
            cfg_.enable_device(serial_);
        }
        cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 90);
        cfg_.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_BGR8, 90);
        pipe_.start(cfg_);

        frameset_ = pipe_.wait_for_frames(1500); 
        rs2::frame depth = frameset_.get_depth_frame();
        rs2::frame color = frameset_.get_color_frame();
        rs2::stream_profile depth_profile = depth.get_profile();
        rs2::stream_profile color_profile = color.get_profile();
        // auto event_camera_rs = color_profile.clone(color_profile.stream_type(), 0, color_profile.format()); // event profile
        depth_intrin_ = depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
        color_intrin_ = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
        depth2color_rs_ex_ = depth_profile.get_extrinsics_to(color_profile);
        rs_exposure_ = (depth.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE));    
        LOG(INFO) << "exposure = " << rs_exposure_;
        LOG(INFO) << "intrinsic = " << color_intrin_.fx << " " << color_intrin_.fy << " " << color_intrin_.ppx << " " << color_intrin_.ppy;
        LOG(INFO) << "intrinsic = " << depth_intrin_.fx << " " << depth_intrin_.fy << " " << depth_intrin_.ppx << " " << depth_intrin_.ppy;

        K_color[0] = color_intrin_.fx; K_color[1] = color_intrin_.fy; K_color[2] = color_intrin_.ppx; K_color[3] = color_intrin_.ppy;
        K_depth[0] = depth_intrin_.fx; K_depth[1] = depth_intrin_.fy; K_depth[2] = depth_intrin_.ppx; K_depth[3] = depth_intrin_.ppy;
        memcpy(D_color, color_intrin_.coeffs, sizeof(color_intrin_.coeffs));
        memcpy(D_depth, depth_intrin_.coeffs, sizeof(color_intrin_.coeffs));
        width = depth_intrin_.width;
        height = depth_intrin_.height;

        // extrinsic matrix from depth camera depth image to rgb image
        Eigen::Matrix3d rot;
        rot << depth2color_rs_ex_.rotation[0], depth2color_rs_ex_.rotation[1], depth2color_rs_ex_.rotation[2],
                depth2color_rs_ex_.rotation[3], depth2color_rs_ex_.rotation[4], depth2color_rs_ex_.rotation[5], 
                depth2color_rs_ex_.rotation[6], depth2color_rs_ex_.rotation[7], depth2color_rs_ex_.rotation[8];
        Eigen::Vector3d tran;
        tran << depth2color_rs_ex_.translation[0], depth2color_rs_ex_.translation[1], depth2color_rs_ex_.translation[2];
        Eigen::Quaterniond depth_to_rgb_q(rot);
        depth2color_rs_ex_se3_ = SE3(depth_to_rgb_q, tran);
    } else if (mode_ == 1 || mode_ == 2) {
        K_color[0] = color_fx; K_color[1] = color_fy; K_color[2] = color_cx; K_color[3] = color_cy;
        K_depth[0] = depth_fx; K_depth[1] = depth_fy; K_depth[2] = depth_cx; K_depth[3] = depth_cy;
        D_color[0] = color_k0; D_color[1] = color_k1; D_color[2] = color_k2; D_color[3] = color_k4;
        D_depth[0] = 0.0; D_depth[1] = 0.0; D_depth[2] = 0.0; D_depth[3] = 0.0; D_depth[4] = 0.0;
        rs_exposure_ = 9946; // Manually get the realsense camera exposure (ms)
        // Note the order of the arguments: the real w coefficient first, while internally the coefficients 
        // are stored in the following order: [x, y, z, w]
        Eigen::Quaterniond  depth_to_rgb_q(rgbddepth2color_qw, rgbddepth2color_qx, rgbddepth2color_qy, rgbddepth2color_qz);
        Eigen::Vector3d tran(rgbddepth2color_tx, rgbddepth2color_ty, rgbddepth2color_tz);
        depth2color_rs_ex_se3_ = SE3(depth_to_rgb_q, tran);
        // just read first image to get width and height
        // cv::Mat depth_img = cv::imread(rgbd_dataset_path_+"/image_depth_record/000000.png", cv::IMREAD_UNCHANGED);
        width = 640;
        height = 360;
    }
    LOG(INFO) << "Depth dist" << D_depth[0] << " "<< D_depth[1] << " "<< D_depth[2] << " "<< D_depth[3] << " "<< D_depth[4];
    Camera::Ptr new_camera(new Camera(K_color, K_depth, D_color, D_depth, width, height,
                                        pyramid_layers_, pyramid_ratio_, SE3(SO3(), Vec3::Zero()), depth_scale_));

    Eigen::Quaterniond rgbdcolor_to_event_q = Eigen::Quaterniond(qw, qx, qy, qz);
    Eigen::Vector3d rgbdcolor_to_event_t = Eigen::Vector3d(tx, ty, tz);
    // can not use Sophus::SE3 // need to use Sophus::SE3d
    SE3 rgbdcolor_to_event(rgbdcolor_to_event_q, rgbdcolor_to_event_t);

    // extrinsic matrix from depth camera depth image to event image
    SE3 depth_to_event = rgbdcolor_to_event * depth2color_rs_ex_se3_;
    // LOG(INFO) << "depth2color_rs_ex_ Q = " << depth_to_rgb_q.x() << " " << depth_to_rgb_q.y() << " " << depth_to_rgb_q.z() << " " << depth_to_rgb_q.w();
    // LOG(INFO) << "tran = " << tran.transpose();
    LOG(INFO) << "depth2color_rs_ex_se3_ R= \n" << depth2color_rs_ex_se3_.rotationMatrix();
    LOG(INFO) << "depth2color_rs_ex_se3_ t= \n" << depth2color_rs_ex_se3_.translation();
    LOG(INFO) << "rgbdcolor_to_event R= \n" << rgbdcolor_to_event.rotationMatrix();
    LOG(INFO) << "rgbdcolor_to_event t= \n" << rgbdcolor_to_event.translation();
    LOG(INFO) << "depth_to_event R= \n" << depth_to_event.rotationMatrix();
    LOG(INFO) << "depth_to_event t= \n" << depth_to_event.translation();

    new_camera->width_ = width;
    new_camera->height_ = height;
    new_camera->SetDepthtoColor(depth2color_rs_ex_se3_);
    new_camera->SetDepthtoEvent(depth_to_event);
    rgbd_camera_ = std::move(new_camera);

    mRGBDThread_ = std::thread(&DataReader::RGBDDataReadingThread, this);

    return true;
}

bool DataReader::InitEventCamera() {
    // read event camera intrinsics and extrinsics
    double fx = Config::Get<double>("eventcamera.fx");
    double fy = Config::Get<double>("eventcamera.fy");
    double cx = Config::Get<double>("eventcamera.cx");
    double cy = Config::Get<double>("eventcamera.cy");
    double k0 = Config::Get<double>("eventcamera.coeff0");
    double k1 = Config::Get<double>("eventcamera.coeff1");
    double k2 = Config::Get<double>("eventcamera.coeff2");
    double k3 = Config::Get<double>("eventcamera.coeff3");
    double k4 = Config::Get<double>("eventcamera.coeff4");
    int width = Config::Get<int>("eventcamera.width");
    int height = Config::Get<int>("eventcamera.height");
    Camera::Ptr new_camera(new Camera(fx, fy, cx, cy, k0, k1, k2, k3, k4, width, height, pyramid_layers_, pyramid_ratio_,
                                SE3(SO3(), Vec3::Zero()))); // t.norm() is base line
    event_camera_ = std::move(new_camera);

    current_image_index_ = 0;

    int64_t background_activity_time = Config::Get<int>("background_activity_time");

    if (mode_ == 3 || mode_ == 5) { // event_rgbd_live mode
        mEventReader_ = EventReader(""); // empty string will opens first discovered camera of any type.
    } else {
        // std::string camera_name = "DAVIS346_00000545";
        mEventReader_ = EventReader(event_dataset_path_, "");
        LOG(INFO)<<"event_dataset_path_ = "<<event_dataset_path_;
    }

    auto &cameraCapPtr = mEventReader_.getCameraCapturePtr();
    auto &cameraRecPtr = mEventReader_.getMonoCameraRecordingPtr();
	auto times = mEventReader_.getTimeRange();

    if (cameraCapPtr != nullptr) {
		if (cameraCapPtr->isFrameStreamAvailable()) {
            LOG(INFO)<<"Read from live capturing.";
			// DAVIS camera
			cameraCapPtr->setDavisFrameInterval(dv::Duration(40000));
			cameraCapPtr->deviceConfigSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, 1000);
            cameraCapPtr->setDavisColorMode(static_cast<dv::io::CameraCapture::DavisColorMode>(0));
            cameraCapPtr->setDavisReadoutMode(static_cast<dv::io::CameraCapture::DavisReadoutMode>(0));
            if (Config::Get<int>("enable_autoexposure")) {
                cameraCapPtr->enableDavisAutoExposure();
                LOG(INFO)<<"enable auto exposure";
            } else {
                cameraCapPtr->setDavisExposureDuration(dv::Duration(Config::Get<int>("exposure")));
                LOG(INFO)<<"set exposure "<<Config::Get<int>("exposure");
            }
            // Create the filter and return
            if (mNoiseFilter_ == nullptr) {
                mNoiseFilter_ = std::make_unique<dv::noise::BackgroundActivityNoiseFilter<>>(
                    mEventReader_.getEventResolution().value(), dv::Duration(background_activity_time));
            }
            // Noise filter is instantiated, just update the period
		    mNoiseFilter_->setBackgroundActivityDuration(dv::Duration(background_activity_time));
		}
	} else if (cameraRecPtr != nullptr) {
        if (cameraRecPtr->isFrameStreamAvailable()) {
            LOG(INFO)<<"Read from recorded file.";
            // Create the filter and return
            if (mNoiseFilter_ == nullptr) {
                mNoiseFilter_ = std::make_unique<dv::noise::BackgroundActivityNoiseFilter<>>(
                    mEventReader_.getEventResolution().value(), dv::Duration(background_activity_time));
            }
            // Noise filter is instantiated, just update the period
		    mNoiseFilter_->setBackgroundActivityDuration(dv::Duration(background_activity_time));
		}
    }
    // mClock_ = std::thread(&DataReader::Clock, this, times->first, times->second, Config::Get<int>("time_increment"));
    mEventsReadingThread_ = std::thread(&DataReader::EventsReadingThread, this);
    mFrameReadingThread_ = std::thread(&DataReader::FrameReadingThread, this);
    mImuReadingThread_ = std::thread(&DataReader::ImuReadingThread, this);
    return true;
}

// Convert rs2::frame to cv::Mat
cv::Mat DataReader::Frame2Mat(const rs2::frame& f) {
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16) // current depth image format
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat DataReader::DepthFrame2Meters(const rs2::depth_frame & f) {
    cv::Mat dm = Frame2Mat(f);
    dm.convertTo( dm, CV_64F );
    dm = dm * f.get_units();
    return dm;
}

/**
 * Get depth image that is already projected to event camera frame
 * Note: both input output image are ushort type
 */
bool DataReader::ImageAlignment(const Camera::Ptr cam_in, const Camera::Ptr cam_out, const ushort* input_data, ushort* output_data, SE3& extrinsic) {
    int h_in = cam_in->height_, w_in = cam_in->width_;
    int h_out = cam_out->height_, w_out = cam_out->width_;
    // #pragma omp parallel for schedule(dynamic)
    #pragma omp parallel for num_threads(16)
    for (int y = 0; y < h_in; ++y) {
        int input_pixel_index = y * w_in;
        // auto output_pixels_row = output_pixels + y * channel;
        for (int x = 0; x < w_in; ++x, ++input_pixel_index) {
            // Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
            if (ushort data = input_data[input_pixel_index]) {
                // Map the top-left corner of the depth pixel onto the other image
                // Vec2 pt(x-0.5f, y-0.5f);
                Vec2 pt(x, y);
                Vec3 pt_c = cam_in->pixel2camera(pt, data*depth_scale_);
                Vec3 pt_new = cam_in->camera2camera(pt_c, extrinsic);
                Vec2 pxl_new = cam_out->camera2pixel(pt_new);
                const int other_x0 = static_cast<int>(pxl_new[0] + 0.5f);
                const int other_y0 = static_cast<int>(pxl_new[1] + 0.5f);
                if (other_x0 < 0 || other_y0 < 0 || other_x0 >= w_out || other_y0 >= h_out)
                    continue;
                output_data[other_y0 * w_out + other_x0] = input_data[input_pixel_index];

                // // Map the bottom-right corner of the depth pixel onto the other image
                // Vec2 pt2(x+0.5f, y+0.5f);
                // Vec3 pt_c2 = cam_in->pixel2cameraDistortion(pt2, data*0.001);
                // Vec3 pt_new2 = cam_in->camera2camera(pt_c2, depth2color_rs_ex_se3_);
                // Vec2 pxl_new2 = cam_out->camera2pixelDistortion(pt_new2);
                // const int other_x1 = static_cast<int>(pxl_new2[0] + 0.5f);
                // const int other_y1 = static_cast<int>(pxl_new2[1] + 0.5f);
                // if (other_x0 < 0 || other_y0 < 0 || other_x1 >= color_intrin_.width || other_y1 >= color_intrin_.height)
                //     continue;
                // // Transfer between the depth pixels and the pixels inside the rectangle on the other image
                // for (int pxl_y = other_y0; pxl_y <= other_y1; ++pxl_y) {
                //     for (int pxl_x = other_x0; pxl_x <= other_x1; ++pxl_x) {

                //         output_data[pxl_y * width + pxl_x] = output_data[pxl_y * width + pxl_x]?
                //             std::min((int)(output_data[pxl_y * width + pxl_x]), (int)input_data[input_pixel_index]) :
                //             input_data[input_pixel_index];
                //     }
                // }
            } else {
                Vec2 pt(x, y);
                Vec3 pt_c = cam_in->pixel2camera(pt, data*depth_scale_);
                Vec3 pt_new = cam_in->camera2camera(pt_c, extrinsic);
                Vec2 pxl_new = cam_out->camera2pixel(pt_new);
                const int other_x0 = static_cast<int>(pxl_new[0] + 0.5f);
                const int other_y0 = static_cast<int>(pxl_new[1] + 0.5f);
                if (other_x0 < 0 || other_y0 < 0 || other_x0 >= w_out || other_y0 >= h_out)
                    continue;
                output_data[other_y0 * w_out + other_x0] = 65535;
            }
        }
    }
    return true;
}

void DataReader::writeImg(std::string addr, cv::Mat* img) {
    cv::imwrite(addr, *img);
}

// cam_in needs to be depth camera
// extrin_select: 0 project to rgb frame, 1 project to event frame;
bool DataReader::GetAlignedDepthData(Camera::Ptr cam_in, Camera::Ptr cam_out, cv::Mat& rgb_img, cv::Mat& depth_img, int extrin_select) {
    frameset_ = pipe_.wait_for_frames(1500);
    // Undistort RGB image
    cv::remap(rgb_img, rgb_img, cam_out->remap_before_, cam_out->remap_after_, cv::INTER_LINEAR);

    auto depth_frame = frameset_.get_depth_frame();
    cv::Mat depth_img_rs = Frame2Mat(depth_frame);
    depth_img = cv::Mat::zeros(cam_out->height_, cam_out->width_, CV_16UC1);
    ushort* output_data = reinterpret_cast<ushort*> (depth_img.data);
    const ushort* input_data = reinterpret_cast<ushort*> (depth_img_rs.data);
    auto t0 = std::chrono::steady_clock::now();
    SE3 extrinsic;
    if (extrin_select == 0) {
        extrinsic = cam_in->depth_to_color_;
    } else if (extrin_select == 1) {
        extrinsic = cam_in->depth_to_event_;
    }
    ImageAlignment(cam_in, cam_out, input_data, output_data, extrinsic);
    auto t1 = std::chrono::steady_clock::now();
    auto time_used1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    LOG(INFO) << "Img align cost time: " << time_used1.count() << " seconds.";
    if (rgb_img.data == nullptr || depth_img.data == nullptr) {
        LOG(WARNING) << "cannot find images at index ";
        return false;
    }
    return true;
}

Frame::Ptr DataReader::NextFrame() {
    // LOG(INFO)<<"Before getting data ----- RGB data queue has "<<mRGBQueue_.size()<<" data. Depth data queue has "<<mDepthQueue_.size()<<" data.";
    // cv::waitKey(100);
    boost::format fmt("%s/image_%d/%06d.png"); // 10 us
    std::optional<dv::EventStore> event_data;
    std::optional<dv::Frame> frame_data;
    std::optional<dv::IMU> imu_data;
    int64_t rgb_timestamp, depth_timestamp;
    // read images
    switch (mode_)
    {
    case 0: { // stereo mode
        image_left =
            cv::imread((fmt % rgbd_dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
        image_right =
            cv::imread((fmt % rgbd_dataset_path_ % 1 % current_image_index_).str(),
                    cv::IMREAD_GRAYSCALE);
        if (image_left.data == nullptr || image_right.data == nullptr) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }
    } break;
    case 1: { // rgbd mode
        image_left =
            cv::imread((fmt % rgbd_dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
        image_depth =
            cv::imread((fmt % rgbd_dataset_path_ % 1 % current_image_index_).str(), cv::IMREAD_UNCHANGED);
        if (image_left.data == nullptr || image_depth.data == nullptr) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }
    } break;
    case 2: { // event_rgbd mode
        bool get_data = false;
        if (!mRGBQueue_.empty() && !mDepthQueue_.empty()) {
            rgb_timestamp = mRGBQueue_.front().timestamp;
            depth_timestamp = mDepthQueue_.front().timestamp;
            get_data = true;
            // LOG(INFO)<<"rgb_timestamp is "<<rgb_timestamp;
        } else {
            // LOG(WARNING)<<"RGB image queue is empty!!!!!!!!!!";
            // auto new_frame = Frame::CreateFrame();
            data_ready_ = false;
            // return new_frame;
        }
        if (abs(rgb_timestamp-depth_timestamp)>12000 && get_data) {
            if (rgb_timestamp < depth_timestamp) {
                // LOG(INFO)<<"Data not ready: timestamp between depth and rgb is "<<depth_timestamp - rgb_timestamp;
                mRGBQueue_.pop();
                // auto new_frame = new Frame();
                data_ready_ = false;
                Frame::Ptr new_frame(new Frame);
                new_frame->angle_vel_ = angle_vel_;
                new_frame->linear_vel_ = linear_vel_;
                new_frame->timestamp_ = rgb_timestamp_;
                return new_frame;
            } else {
                
                while (rgb_timestamp >= depth_timestamp) {
                    mDepthQueue_.pop();
                    rgb_timestamp = mRGBQueue_.front().timestamp;
                    depth_timestamp = mDepthQueue_.front().timestamp;
                }
                static int skip(0);
                if (skip < 3) {
                    data_ready_ = false;
                    skip++;
                } else {
                    data_ready_ = true;
                }
                image_left = mRGBQueue_.front().image;
                image_depth = mDepthQueue_.front().image;
                rgb_timestamp_ = mRGBQueue_.front().timestamp;
                depth_timestamp_ = mDepthQueue_.front().timestamp;
                mDepthQueue_.pop();
                mRGBQueue_.pop();
            }
        } else if (abs(rgb_timestamp-depth_timestamp)<=12000 && get_data) {
            // LOG(INFO)<<"Data ready: timestamp between depth and rgb:  "<<std::setprecision(16)<<
            // depth_timestamp*1e-6 <<" "<< rgb_timestamp*1e-6<<" "<<depth_timestamp*1e-6 - rgb_timestamp*1e-6;
            LOG(INFO)<<"Data ready from sit 1: timestamp between depth and rgb:  "<<std::setprecision(16)<<depth_timestamp*1e-6 - rgb_timestamp*1e-6;
            LOG(INFO)<<"RGB data queue has "<<mRGBQueue_.size()<<" data. Depth data queue has "<<mDepthQueue_.size()<<" data.";
            image_left = mRGBQueue_.front().image;
            image_depth = mDepthQueue_.front().image;
            rgb_timestamp_ = mRGBQueue_.front().timestamp;
            depth_timestamp_ = mDepthQueue_.front().timestamp;
            mDepthQueue_.pop();
            mRGBQueue_.pop();
            // cv::waitKey(0);
        } else {
            data_ready_ = false;
            // LOG(INFO)<<"2 "<<rgb_timestamp_;
            Frame::Ptr new_frame(new Frame);
            new_frame->angle_vel_ = angle_vel_;
            new_frame->linear_vel_ = linear_vel_;
            new_frame->timestamp_ = rgb_timestamp_;
            return new_frame;
        }
    } break;
    case 3: { // event_rgbd_live mode
        mDepthBuffer_.consume_one([&](const auto &data) {
            if (const dv::Frame *depth = std::get_if<dv::Frame>(&data); depth != nullptr) {
                image_depth = (*depth).image;
            }
        });
        mRGBBuffer_.consume_one([&](const auto &data) {
            if (const dv::Frame *gray = std::get_if<dv::Frame>(&data); gray != nullptr) {
                image_left = (*gray).image;
            }
        });
    } break;
    case 4: { // rgbd_live mode
        auto t1 = std::chrono::steady_clock::now();
        get_rbgd_ = GetAlignedDepthData(rgbd_camera_, rgbd_camera_, image_left, image_depth, 0);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        cv::imshow("image_left", image_left);
        cv::imshow("image_depth", image_depth);
        cv::waitKey(1);
        LOG(INFO) << "Get RGBD data cost time: " << time_used.count() << " seconds.";
    } break;
    case 5: { //  mode
    
    } break;
    default:
        LOG(ERROR) << "Unknown mode!!!";
        break;
    }

    auto new_frame = Frame::CreateFrame();

    switch (mode_)
    {
    case 0: // stereo mode
        new_frame->right_img_ = image_right; // image_right_resized;
        break;
    case 1: // rgbd mode
    case 4: // rgbd_live mode
        new_frame->depth_img_ = image_depth;
        break;
    case 2: // event_rgbd mode
        new_frame->depth_img_ = image_depth;
        new_frame->event_data_ = event_data;
        new_frame->timestamp_ = rgb_timestamp_;
        new_frame->depth_timestamp_ = depth_timestamp_;
        LOG(INFO)<<"new_frame time = "<<std::setprecision(16)<<new_frame->timestamp_*1e-6;
        break;
    case 3: // event_rgbd_live mode
        // TODO add event
        new_frame->depth_img_ = image_depth;
        break;
    default:
        LOG(ERROR) << "Unknown mode!!!";
        break;
    }


    new_frame->left_img_ = image_left; // image_left_resized; 7 us
    data_ready_ = CheckDataStatus(new_frame);
    if (data_ready_) BuildImagePyramid(new_frame);

    current_image_index_++;
    return new_frame;
}

bool DataReader::CheckDataStatus(Frame::Ptr new_frame) {
    if (new_frame->left_img_.data == nullptr || new_frame->depth_img_.data == nullptr) {
            return false;
    } else {
        return true;
    }
}

void DataReader::BuildImagePyramid(Frame::Ptr new_frame) {
    // create pyramids
    for (int i = 0; i < pyramid_layers_; i++) {
        if (i == 0) {
            new_frame->left_img_pyr_.push_back(new_frame->left_img_);
        } else {
            cv::Mat img_pyr;
            cv::resize(new_frame->left_img_pyr_[i - 1], img_pyr,
                       cv::Size(new_frame->left_img_pyr_[i - 1].cols * pyramid_ratio_, 
                                new_frame->left_img_pyr_[i - 1].rows * pyramid_ratio_));
            new_frame->left_img_pyr_.push_back(img_pyr);
        }
    }
}


}  // namespace DynamicObjectsAvoidance