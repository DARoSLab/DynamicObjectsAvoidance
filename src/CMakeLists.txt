find_package(PkgConfig REQUIRED)
pkg_check_modules(libzstd REQUIRED IMPORTED_TARGET libzstd)

add_library(DynamicObjectsAvoidance
        config.cpp
        camera.cpp
        event_reader.cpp
        data_reader.cpp
        frame.cpp
        time_surface.cpp
        run_control.cpp
        dynamic_objects_detection.cpp)

target_link_libraries(DynamicObjectsAvoidance
        ${THIRD_PARTY_LIBS} PkgConfig::libzstd)
