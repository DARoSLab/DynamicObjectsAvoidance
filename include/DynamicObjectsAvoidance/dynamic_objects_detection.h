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

   private:
    bool inited_ = false;
    std::string config_file_path_;
    dv::EventStore* events_ = nullptr;

    // dataset
    DataReader::Ptr data_reader_ = nullptr;
    int store_rgbd_;
    int rCount = 0;
    int lCount = 0;
    int xVel = 0;
    int yVel = 0;
    int drawCount = 0;

    std::unique_ptr<TimeSurface> time_surface_ = nullptr;
    std::unique_ptr<TimeSurfaceAccumulator> time_surface_accumulator_ = nullptr;
    dv::EventStore slice_;



    cv::Point2i pre_pos_;

};
}  // namespace DynamicObjectsAvoidance

// #endif  // DynamicObjectsAvoidance_dynamic_objects_detection.h_H
