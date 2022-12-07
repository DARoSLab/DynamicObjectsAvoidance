add_library(DynamicObjectsAvoidance
        config.cpp
        camera.cpp
        event_reader.cpp
        data_reader.cpp
        frame.cpp
        time_surface.cpp
        dynamic_objects_detection.cpp)

target_link_libraries(DynamicObjectsAvoidance
        ${THIRD_PARTY_LIBS})
