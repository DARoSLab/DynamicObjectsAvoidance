#include <gflags/gflags.h>
// #include <DynamicObjectsAvoidance/common_include.h>
#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"
#include "DynamicObjectsAvoidance/run_control.h"

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char **argv) {
    // printf("OpenCV: %s", cv::getBuildInformation().c_str());
    google::ParseCommandLineFlags(&argc, &argv, true);

    DynamicObjectsAvoidance::DynamicObjectsAvoidance::Ptr DOA(
        new DynamicObjectsAvoidance::DynamicObjectsAvoidance(FLAGS_config_file));
    assert(DOA->Init() == true);

    std::cout << "[HIGH-LEVEL] Communication" << std::endl
            << "WARNING: Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    
    // Add for safety check
    // std::cin.ignore();

    Custom custom(HIGHLEVEL, DOA);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    DOA->Run();

    LOG(INFO)<<"Hello Dynamic Objects Avoidance";

    return 0;
}
