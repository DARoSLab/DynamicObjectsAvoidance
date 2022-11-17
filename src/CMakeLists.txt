add_library(DynamicObjectsAvoidance
        event_reader.cpp
        data_reader.cpp
        time_surface.cpp)

target_link_libraries(DynamicObjectsAvoidance
        ${THIRD_PARTY_LIBS})
