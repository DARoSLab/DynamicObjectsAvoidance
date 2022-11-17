# SET(TEST_SOURCES test_speed)
# include_directories("/home/zh/code/dv/dv-processing/tests/external/cli11/include", "/usr/include/")

# FOREACH (test_src ${TEST_SOURCES})
#     ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
#     TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} DynamicObjectsAvoidance)
#     ADD_TEST(${test_src} ${test_src})
# ENDFOREACH (test_src)
