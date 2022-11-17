#pragma once
// #ifndef DynamicObjectsAvoidance_DATASET_H
// #define DynamicObjectsAvoidance_DATASET_H
#include "DynamicObjectsAvoidance/camera.h"
#include "DynamicObjectsAvoidance/common_include.h"
#include "DynamicObjectsAvoidance/frame.h"
#include "DynamicObjectsAvoidance/config.h"
#include "DynamicObjectsAvoidance/event_reader.h"
#include "DynamicObjectsAvoidance/time_surface.h"

// // dv-processing headers
// #include <dv-processing/core/frame.hpp>
// #include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
// #include <dv-processing/io/writer.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>
// DAVIS driver
#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>

#include <csignal>
#include <atomic>
// #include <signal.h>
#include <opencv2/highgui.hpp>

// Real sense camera headers
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include "/home/zh/code/sensor_driver/librealsense-2.50.0/src/api.h"
#include "omp.h"
#include <boost/thread/pthread/recursive_mutex.hpp>

namespace DynamicObjectsAvoidance {

using Data = std::variant<dv::EventStore, dv::Frame, cv::Mat>;
using DataQueue = boost::lockfree::spsc_queue<Data, boost::lockfree::capacity<100>>;
using TimestampQueue = boost::lockfree::spsc_queue<int64_t, boost::lockfree::capacity<1000>>;

/**
 * Data reading
 * For dataset mode: set dataset path & config file path
 * For live mode: set config file path only
 * Need to call Init* function before calling NextFrame()
 */
class DataReader {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<DataReader> Ptr;
	DataReader() = default;
    DataReader(const std::string& cfg_path);
    DataReader(const std::string& event_dataset_path, const std::string& rgbd_dataset_path, 
                        const std::string& cfg_path);
    ~DataReader();

    void EventsReadingThread();
    void FrameReadingThread();
    void ImuReadingThread();
    void RGBDDataReadingThread();
	void Clock(int64_t start, int64_t end, int64_t timeIncrement);

    // Initialization, return true if success
    bool InitStereo();
    bool InitRGBD();
    bool InitEventCamera();
    cv::Mat Frame2Mat(const rs2::frame& f);
    cv::Mat DepthFrame2Meters(const rs2::depth_frame& f);
    void writeImg(std::string addr, cv::Mat* img);

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();
    // extrin_select: 0 project to rgb frame, 1 project to event frame;
    bool GetAlignedDepthData(Camera::Ptr cam_in, Camera::Ptr cam_out, cv::Mat& rgb_img, cv::Mat& depth_img, int extrin_select);
    bool ImageAlignment(const Camera::Ptr cam_in, const Camera::Ptr cam_out, const ushort* input_data, ushort* output_data, SE3& extrinsic);
    void BuildImagePyramid(Frame::Ptr new_frame);
    bool CheckDataStatus(Frame::Ptr new_frame);
    bool data_ready_ = false;
    // get camera
    // Camera::Ptr GetCamera(int camera_id) const {
    //     return cameras_.at(camera_id);
    // }
    Camera::Ptr GetEventCamera() const {
        return event_camera_;
    }
    Camera::Ptr GetRGBDCamera() const {
        return rgbd_camera_;
    }
    dv::EventStore* GetEventStore () {
        return mEventBuffer_;
    }
    std::queue<std::pair<int64_t, double>>* GetImuLinQueue() {
        return &mImuLinVelQueue_;
    }
    
    std::queue<std::pair<int64_t, double>>* GetImuAngQueue() {
        return &mImuAngVelQueue_;
    }

   private:
    std::string cfg_path_;
    std::string event_dataset_path_;
    std::string rgbd_dataset_path_;
    std::string event_data_file_name_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
    Camera::Ptr event_camera_;
    Camera::Ptr rgbd_camera_;
    int mode_; // stereo, rgbd
    bool get_depth_, get_rbgd_, get_event_;
    int visualize_alignment_;
    int64_t curr_imu_timestamp_ = 0;
    int64_t last_imu_timestamp_ = 0;
    int64_t rgb_timestamp_;
    int64_t depth_timestamp_;
    int64_t rgb_timestamp_print_;
    std::ofstream rgbd_timestamp_file_out_;
    std::ifstream rgbd_timestamp_file_in_;
    // static dv::io::MonoCameraRecording event_reader_;
    bool check_event_stream_ = false;
    int pyramid_layers_;
    double pyramid_ratio_;
    int exposure_;

    bool get_imu_bias_ = false;
    double init_gx_, init_gy_, init_gz_, init_ax_, init_ay_, init_az_;
    Eigen::Vector3d linear_vel_, angle_vel_;

    // Below are realsense camera related parameters and functions
    rs2::frameset frameset_;
    std::string serial_;
    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::frame depth_;

    // rs2_intrinsics event_in_;
    rs2_intrinsics color_intrin_;
    rs2_intrinsics depth_intrin_;
    rs2_extrinsics depth2event_rs_ex_;
    rs2_extrinsics depth2color_rs_ex_;
    SE3 depth2color_rs_ex_se3_;
    float depth_scale_;

    dv::Duration mStoreTimeLimit_ = dv::Duration(10000000);
    EventReader mEventReader_;
	DataQueue mDataQueue_;
    dv::EventStore* mEventBuffer_ = nullptr;
    // boost::lockfree::spsc_queue<dv::EventStore, boost::lockfree::capacity<100>> mEventBuffer_;
    boost::lockfree::spsc_queue<std::variant<dv::Frame>, boost::lockfree::capacity<500>> mDepthBuffer_;
    boost::lockfree::spsc_queue<std::variant<dv::Frame>, boost::lockfree::capacity<1>> mRGBBuffer_;
    std::queue<dv::Frame> mDepthQueue_;
    std::queue<dv::Frame> mRGBQueue_;
    std::queue<std::pair<int64_t, double>> mImuLinVelQueue_;
    // std::queue<std::pair<int64_t, double>> mImuLinVelQueue2_;
    std::queue<std::pair<int64_t, double>> mImuAngVelQueue_;
    std::queue<int64_t>* int64_queue_;

	std::atomic<bool> mSpinThread_ = true;
	std::unique_ptr<dv::noise::BackgroundActivityNoiseFilter<>> mNoiseFilter_ = nullptr;
    // std::thread mReaderThread_;
    std::thread mFrameReadingThread_;
    std::thread mEventsReadingThread_;
    std::thread mImuReadingThread_;
    std::thread mRGBDThread_;
    std::thread mDisplayThread_;
	std::thread mClock_;

	std::unique_ptr<dv::Accumulator> accumulator_ = nullptr;
	std::unique_ptr<dv::PixelAccumulator> pixel_accumulator_ = nullptr;
    double time_depth_, time_event_, time_rgb_, time_diff_;
    long long rs_exposure_;

    cv::Mat image_left, image_right, image_depth, image_event; // 4 us
    TimestampQueue mFrameTimestampQueue_;
    TimestampQueue mEventsTimestampQueue_;
    TimestampQueue mImuTimestampQueue_;
    boost::recursive_mutex mReaderMutex_;
    std::unique_ptr<TimeSurface> time_surface_mine_ = nullptr;

};
}  // namespace DynamicObjectsAvoidance

// #endif