#pragma once
// #ifndef DynamicObjectsAvoidance_dynamic_objects_detection.h_H
// #define DynamicObjectsAvoidance_dynamic_objects_detection.h_H

#include "DynamicObjectsAvoidance/common_include.h"
#include "DynamicObjectsAvoidance/data_reader.h"

// // dv-processing headers
// #include <dv-processing/core/frame.hpp>
// #include <dv-processing/io/camera_capture.hpp>

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

   private:
    bool inited_ = false;
    std::string config_file_path_;

    // dataset
    DataReader::Ptr data_reader_ = nullptr;
    int store_rgbd_;
};
}  // namespace DynamicObjectsAvoidance

// #endif  // DynamicObjectsAvoidance_dynamic_objects_detection.h_H
