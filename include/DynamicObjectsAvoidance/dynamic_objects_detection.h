#pragma once

#include "DynamicObjectsAvoidance/common_include.h"
#include "DynamicObjectsAvoidance/data_reader.h"
#include "DynamicObjectsAvoidance/config.h"
#include "DynamicObjectsAvoidance/camera.h"
#include "DynamicObjectsAvoidance/time_surface.h"
#include "DynamicObjectsAvoidance/directionQueue.h"

// // dv-processing headers
#include <dv-processing/core/frame.hpp>
#include <dv-processing/io/camera_capture.hpp>


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
        double GetCmdDir(){
            return cmdDir; 
        }
        bool GetInitAct(){
            return initAction; 
        }
        cv::Mat GetImage();
        

    protected:
        bool inited_ = false;
        int motionCnt = 0;
        bool collision = false;
        std::string config_file_path_;
        dv::EventStore* events_ = nullptr;

        // dataset
        DataReader::Ptr data_reader_ = nullptr;
        int store_rgbd_;

        double xVel = 0;
        double yVel = 0;
        double cmdDir = .0f;
        bool initAction = false;

        int tmpDirection;
        int majorDirection;
        int groundX;
        int topX;
        cv::Mat frame;
        // int key = cv::waitKey(100);
        
        
        FixedQueue<int, 7> dirQueue; // Queue to store the last 5 directions
        std::vector<int> dirCounts{std::vector<int>(5, 0)}; // Store the number of times for each direction (default 0)
        
        std::unique_ptr<TimeSurface> time_surface_ = nullptr;
        std::unique_ptr<TimeSurfaceAccumulator> time_surface_accumulator_ = nullptr;
        dv::EventStore slice_;

        cv::Point2i pre_pos_;
        std::vector<cv::Point2f>posVector;
        int key;
        std::thread imshowthread_;
        void ImshowThread();

    };
}  // namespace DynamicObjectsAvoidance
