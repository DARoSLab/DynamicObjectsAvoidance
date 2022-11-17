#pragma once

// #ifndef DynamicObjectsAvoidance_CAMERA_H
// #define DynamicObjectsAvoidance_CAMERA_H

#include "DynamicObjectsAvoidance/common_include.h"
#include <opencv2/opencv.hpp>

namespace DynamicObjectsAvoidance {

// Pinhole stereo camera model
class Camera {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           fx_depth_ = 0, fy_depth_ = 0, cx_depth_ = 0, cy_depth_ = 0,
           baseline_ = 0, depth_scale_ = 30;  // Camera intrinsics
    double coeffs_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double coeffs_depth_[5] = {0.0, 0.0, 0.0, 0.0};
    double* fx_pyr_ = nullptr;
    double* fy_pyr_ = nullptr;
    double* cx_pyr_ = nullptr;
    double* cy_pyr_ = nullptr;
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics
    SE3 depth_to_event_;
    SE3 depth_to_color_;
    int pyramid_layers_;
    double pyramid_ratio_;
    int width_, height_;
    cv::Mat remap_before_, remap_after_;

    Camera();

    // stereo without distortion
    Camera(double fx, double fy, double cx, double cy, int pyramid_layers, double pyramid_ratio, double baseline, const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), pyramid_layers_(pyramid_layers), pyramid_ratio_(pyramid_ratio),
        baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();

        fx_pyr_ = new double[pyramid_layers];
        fy_pyr_ = new double[pyramid_layers];
        cx_pyr_ = new double[pyramid_layers];
        cy_pyr_ = new double[pyramid_layers];
        
        fx_pyr_[0] = fx;
        fy_pyr_[0] = fy;
        cx_pyr_[0] = cx;
        cy_pyr_[0] = cy;

        for (int level = 1; level < pyramid_layers; level++) {
            // scale fx, fy, cx, cy in different pyramid levels
            fx_pyr_[level] = fx_pyr_[level-1] * pyramid_ratio;
            fy_pyr_[level] = fy_pyr_[level-1] * pyramid_ratio;
            cx_pyr_[level] = cx_pyr_[level-1] * pyramid_ratio;
            cy_pyr_[level] = cy_pyr_[level-1] * pyramid_ratio;
        }
    }

    // stereo with distortion
    Camera(double fx, double fy, double cx, double cy, double k0, double k1, double k2, double k3, 
            int pyramid_layers, double pyramid_ratio, double baseline, const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), pyramid_layers_(pyramid_layers), pyramid_ratio_(pyramid_ratio),
        baseline_(baseline), pose_(pose) {
        coeffs_[0] = k0;
        coeffs_[1] = k1;
        coeffs_[2] = k2;
        coeffs_[3] = k3;
        pose_inv_ = pose_.inverse();

        fx_pyr_ = new double[pyramid_layers];
        fy_pyr_ = new double[pyramid_layers];
        cx_pyr_ = new double[pyramid_layers];
        cy_pyr_ = new double[pyramid_layers];
        
        fx_pyr_[0] = fx;
        fy_pyr_[0] = fy;
        cx_pyr_[0] = cx;
        cy_pyr_[0] = cy;

        for (int level = 1; level < pyramid_layers; level++) {
            // scale fx, fy, cx, cy in different pyramid levels
            fx_pyr_[level] = fx_pyr_[level-1] * pyramid_ratio;
            fy_pyr_[level] = fy_pyr_[level-1] * pyramid_ratio;
            cx_pyr_[level] = cx_pyr_[level-1] * pyramid_ratio;
            cy_pyr_[level] = cy_pyr_[level-1] * pyramid_ratio;
        }
    }

    // RGBD camera has both rgb camera and depth camera, so it need two intrinsic matrice and distortion parameters
    // Note that: Realsense depth camera does not have distortion
    Camera(float* K_color, const float* K_depth, const float* D_color, const float* D_depth, const int width, const int height,
            int pyramid_layers, double pyramid_ratio, const SE3 &pose, double depth_scale)
        : pyramid_layers_(pyramid_layers), pyramid_ratio_(pyramid_ratio), width_(width), height_(height),
        pose_(pose), depth_scale_(depth_scale) {
        coeffs_[0] = D_color[0];
        coeffs_[1] = D_color[1];
        coeffs_[2] = D_color[2];
        coeffs_[3] = D_color[3];
        coeffs_[4] = D_color[4];
        coeffs_depth_[0] = D_depth[0];
        coeffs_depth_[1] = D_depth[1];
        coeffs_depth_[2] = D_depth[2];
        coeffs_depth_[3] = D_depth[3];
        coeffs_depth_[4] = D_depth[4];
        pose_inv_ = pose_.inverse();

        // only Realsense color camera has distortion
        cv::Mat K_old = (cv::Mat_<double> ( 3,3 ) << K_color[0], 0.0, K_color[2], 0.0, K_color[1], K_color[3], 0.0, 0.0, 1.0);
        cv::Mat D_old = (cv::Mat_<double> ( 5,1 ) << D_color[0], D_color[1], D_color[2], D_color[3], 0);
        const int alpha = 0; // remove black edge
        cv::Size imageSize(width, height);
        cv::Mat K_new = getOptimalNewCameraMatrix(K_old, D_old, imageSize, alpha, imageSize, 0); // get new intrinsic parameters
        // cv::initUndistortRectifyMap(K_old, D_old, cv::Mat(), K_new, imageSize, CV_16SC2, remap_before_, remap_after_);
        cv::initUndistortRectifyMap(K_old, D_old, cv::Mat(), K_new, imageSize, CV_32FC1, remap_before_, remap_after_);
        K_color[0] = K_new.at<double>(0, 0);
        K_color[1] = K_new.at<double>(1, 1);
        K_color[2] = K_new.at<double>(0, 2);
        K_color[3] = K_new.at<double>(1, 2);
        fx_ = K_color[0]; fy_ = K_color[1]; cx_ = K_color[2]; cy_ = K_color[3];

        coeffs_[0] = 0;
        coeffs_[1] = 0;
        coeffs_[2] = 0;
        coeffs_[3] = 0;
        coeffs_[4] = 0;

        //  fx_(K_color[0]), fy_(K_color[1]), cx_(K_color[2]), cy_(K_color[3])

        fx_pyr_ = new double[pyramid_layers];
        fy_pyr_ = new double[pyramid_layers];
        cx_pyr_ = new double[pyramid_layers];
        cy_pyr_ = new double[pyramid_layers];
        
        fx_pyr_[0] = K_color[0];
        fy_pyr_[0] = K_color[1];
        cx_pyr_[0] = K_color[2];
        cy_pyr_[0] = K_color[3];

        for (int level = 1; level < pyramid_layers; level++) {
            // scale fx, fy, cx, cy in different pyramid levels
            fx_pyr_[level] = fx_pyr_[level-1] * pyramid_ratio;
            fy_pyr_[level] = fy_pyr_[level-1] * pyramid_ratio;
            cx_pyr_[level] = cx_pyr_[level-1] * pyramid_ratio;
            cy_pyr_[level] = cy_pyr_[level-1] * pyramid_ratio;
        }
    }

    // Event camera
    Camera(double fx, double fy, double cx, double cy, double k0, double k1, double k2, double k3, double k4, int width, int height,
            int pyramid_layers, double pyramid_ratio, const SE3 &pose)
        : width_(width), height_(height), pyramid_layers_(pyramid_layers), pyramid_ratio_(pyramid_ratio), pose_(pose) {
        coeffs_[0] = k0;
        coeffs_[1] = k1;
        coeffs_[2] = k2;
        coeffs_[3] = k3;
        coeffs_[4] = k4;
        pose_inv_ = pose_.inverse();

        cv::Mat K_old = (cv::Mat_<double> ( 3,3 ) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        cv::Mat D_old = (cv::Mat_<double> ( 5,1 ) << k0, k1, k2, k3, k4);
        const int alpha = 0; // remove black edge
        // LOG(INFO) << "fx = " << fx << " " << fy << " " << cx << " " << cy;
        cv::Size imageSize(width, height);
        cv::Mat K_new = getOptimalNewCameraMatrix(K_old, D_old, imageSize, alpha, imageSize, 0);
        // cv::initUndistortRectifyMap(K_old, D_old, cv::Mat(), K_new, imageSize, CV_16SC2, remap_before_, remap_after_);
        cv::initUndistortRectifyMap(K_old, D_old, cv::Mat(), K_new, imageSize, CV_32FC1, remap_before_, remap_after_);
        fx_ = K_new.at<double>(0, 0);
        fy_ = K_new.at<double>(1, 1);
        cx_ = K_new.at<double>(0, 2);
        cy_ = K_new.at<double>(1, 2);
        // fx_ = fx;
        // fy_ = fy;
        // cx_ = cx;
        // cy_ = cy;
        // LOG(INFO) << "fx_ = " << fx_ << " " << fy_ << " " << cx_ << " " << cy_;

        fx_pyr_ = new double[pyramid_layers];
        fy_pyr_ = new double[pyramid_layers];
        cx_pyr_ = new double[pyramid_layers];
        cy_pyr_ = new double[pyramid_layers];
        
        fx_pyr_[0] = fx_;
        fy_pyr_[0] = fy_;
        cx_pyr_[0] = cx_;
        cy_pyr_[0] = cy_;

        for (int level = 1; level < pyramid_layers; level++) {
            // scale fx, fy, cx, cy in different pyramid levels
            fx_pyr_[level] = fx_pyr_[level-1] * pyramid_ratio;
            fy_pyr_[level] = fy_pyr_[level-1] * pyramid_ratio;
            cx_pyr_[level] = cx_pyr_[level-1] * pyramid_ratio;
            cy_pyr_[level] = cy_pyr_[level-1] * pyramid_ratio;
            LOG(INFO) << "fx_pyr_[level] = " << fx_pyr_[level];
        }
    }

    SE3 pose() const { return pose_; }

    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    const cv::Mat K_cv() {
        return ( cv::Mat_<double> ( 3,3 ) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0 );
    }
    const cv::Mat D_cv() {
        return (cv::Mat_<double> ( 5,1 ) << coeffs_[0], coeffs_[1], coeffs_[2], coeffs_[3], 0);
    }

    const SE3 GetDepthtoEvent() {
        return depth_to_event_;
    }

    const SE3 GetDepthtoColor() {
        return depth_to_color_;
    }

    void SetDepthtoEvent(SE3 ex_pose) {
        depth_to_event_ = ex_pose;
    }

    void SetDepthtoColor(SE3 ex_pose) {
        depth_to_color_ = ex_pose;
    }

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);
    Vec2 camera2pixelDistortion(const Vec3 &p_c);
    
    Vec2 camera2pixel(const Vec3 &p_c, const int &lvl);

    Vec3 camera2camera(const Vec3 &p_c, const SE3 &T_c_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
    Vec3 pixel2cameraDistortion(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
};

}  // namespace DynamicObjectsAvoidance
// #endif  // DynamicObjectsAvoidance_CAMERA_H
