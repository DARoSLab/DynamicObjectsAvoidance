#pragma once
// #ifndef DynamicObjectsAvoidance_dynamic_objects_detection.h_H
// #define DynamicObjectsAvoidance_dynamic_objects_detection.h_H

#include "DynamicObjectsAvoidance/common_include.h"
#include "DynamicObjectsAvoidance/data_reader.h"
#include "DynamicObjectsAvoidance/config.h"
#include "DynamicObjectsAvoidance/camera.h"
#include "DynamicObjectsAvoidance/time_surface.h"

// // dv-processing headers
#include <dv-processing/core/frame.hpp>
#include <dv-processing/io/camera_capture.hpp>

// #include "DynamicObjectsAvoidance/unitree_legged_sdk.h"

// ctrl header
// #include "DynamicObjectsAvoidance/run_control.h"

#define PI 3.14159265


namespace DynamicObjectsAvoidance {

/**
 * VO interface
 */
class DynamicObjectsAvoidance {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<DynamicObjectsAvoidance> Ptr;

    /// constructor with config file
    DynamicObjectsAvoidance(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * init system control
     */
    // void InitCtrl();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    void SetEventStore (dv::EventStore* events) {
        events_ = events;
    }

    double GetXVel() {
        return xVel;
    }
    double GetYVel() {
        return yVel;
    }

   protected:
    bool inited_ = false;
    std::string config_file_path_;
    dv::EventStore* events_ = nullptr;

    // dataset
    DataReader::Ptr data_reader_ = nullptr;
    int store_rgbd_;
    int rCount = 0;
    int lCount = 0;
    double xVel = 0;
    double yVel = 0;
    double angle;
    int drawCount = 0;


    // Custom summer;

    std::unique_ptr<TimeSurface> time_surface_ = nullptr;
    std::unique_ptr<TimeSurfaceAccumulator> time_surface_accumulator_ = nullptr;
    dv::EventStore slice_;

    cv::Point2i pre_pos_;

};
}  // namespace DynamicObjectsAvoidance

// #endif  // DynamicObjectsAvoidance_dynamic_objects_detection.h_H



// using namespace UNITREE_LEGGED_SDK;

// namespace Avoid_behavior {
//   constexpr int FACE_RIGHT = 0;
//   constexpr int FACE_LEFT = 1;
//   constexpr int FACE_UP = 2;
//   constexpr int FACE_DOWN = 3;
//   constexpr int BODY_UP = 4;
//   constexpr int BODY_DOWN = 5;
//   constexpr int INITIAL = 6;
// };

// class Custom
// {
// public:
//     Custom(uint8_t level): 
//       safe(LeggedType::Go1), 
//        udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)) // WIFI  
//       //  udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)) // WIRED
//     {
//         udp.InitCmdData(cmd);
//     }
//     void UDPRecv();
//     void UDPSend();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     HighCmd cmd = {0};
//     HighState state = {0};
//     int motiontime = 0;
//     float dt = 0.002;  // 0.001~0.01
//     int avoid_mode = 0;
// private:
//     // double obsXvel = xVel;
//     // double obsYvel = yVel;
//     // double obsAngle = angle;
// };

