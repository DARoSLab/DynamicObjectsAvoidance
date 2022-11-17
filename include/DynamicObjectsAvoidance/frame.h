#pragma once

// #ifndef DynamicObjectsAvoidance_FRAME_H
// #define DynamicObjectsAvoidance_FRAME_H

#include <dv-processing/core/core.hpp>
#include "DynamicObjectsAvoidance/camera.h"
#include "DynamicObjectsAvoidance/common_include.h"

namespace DynamicObjectsAvoidance {

// forward declare
struct MapPoint;
struct Feature;

/**
 * Frame
 * asign seperate id for each frame and keyframe
 */
struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;          // id of this frame
    unsigned long keyframe_id_ = 0; // id of key frame
    bool is_keyframe_ = false;      // whther is key frame
    bool is_removed_ = false;       // whether be removed
    int64_t timestamp_;             // timestamp for frame (rgb image)
    int64_t depth_timestamp_;             // timestamp for depth image
    SE3 pose_;                      // Tcw format
    SE3 pose_tmp_;
    SE3 pose_tmp_last_;
    bool has_new_tmp_pose_ = false;
    bool has_last_tmp_pose_ = false;
    std::mutex pose_mutex_;         // Pose data lock
    cv::Mat left_img_, right_img_, depth_img_, event_img_;   // stereo images
    dv::EventStore* events_;
    std::vector<cv::Mat> left_img_pyr_; // image pyramid
    std::vector<cv::Mat> event_img_pyr_;
    std::optional<dv::EventStore> event_data_;
    std::optional<dv::Frame> frame_data_;
    std::optional<dv::IMU> imu_data_;
    Eigen::Vector3d linear_vel_, angle_vel_;

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

   public:  // data members
    Frame() {}

    Frame(long id, int64_t timestamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }
    SE3 TmpPose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_tmp_;
    }

    SE3 LastTmpPose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_tmp_last_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }
    void SetTmpPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_tmp_ = pose;
    }
    void SetLastTmpPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_tmp_last_ = pose;
    }
    /// set keyframe and asign keyframe id
    void SetKeyFrame();

    /// create frame and asign id
    static std::shared_ptr<Frame> CreateFrame();

    double findDepth (int y, int x);
    double findDepthOnRGBDImage(int y, int x);

};

}  // namespace DynamicObjectsAvoidance

// #endif  // DynamicObjectsAvoidance_FRAME_H
