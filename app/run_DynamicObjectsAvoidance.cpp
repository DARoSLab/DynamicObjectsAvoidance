#include <gflags/gflags.h>
// #include <DynamicObjectsAvoidance/common_include.h>
#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    DynamicObjectsAvoidance::DynamicObjectsAvoidance::Ptr DOA(
        new DynamicObjectsAvoidance::DynamicObjectsAvoidance(FLAGS_config_file));
    // assert(DOA->Init() == true);
    // DOA->Run();

    LOG(INFO)<<"Hello Dynamic Objects Avoidance";

    return 0;
}
