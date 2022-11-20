# run DynamicObjectsAvoidance
add_executable(run_dynamic_objects_avoidance run_DynamicObjectsAvoidance.cpp)
target_link_libraries(run_dynamic_objects_avoidance DynamicObjectsAvoidance ${THIRD_PARTY_LIBS})

# Run control (testing)
add_executable(walkTest run_control.cpp)
target_link_libraries(walkTest DynamicObjectsAvoidance ${THIRD_PARTY_LIBS})