#include <gflags/gflags.h>
#include <DynamicObjectsAvoidance/common_include.h>
// #include "DynamicObjectsAvoidance/visual_odometry.h"

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    // DynamicObjectsAvoidance::VisualOdometry::Ptr vo(
    //     new DynamicObjectsAvoidance::VisualOdometry(FLAGS_config_file));
    // assert(vo->Init() == true);
    // vo->Run();

    LOG(INFO)<<"Hello Dynamic Objects Avoidance";

    return 0;
}
