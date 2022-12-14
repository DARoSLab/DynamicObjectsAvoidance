cmake_minimum_required(VERSION 2.8)
project(DynamicObjectsAvoidance)

cmake_policy(SET CMP0063 NEW)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3 -fopenmp -pthread")

set(Boost_NO_WARN_NEW_VERSIONS 1)
# set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl") # already linked below

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-w) # do not show warning

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
# find_package(OpenCV 4.4 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
# set(Sophus_LIBRARIES libSophus.so)
include_directories("/usr/local/include/Sophus")
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# fmt
find_package(fmt REQUIRED)

# # dv-processing
# find_package(dv-processing 1.5.1 REQUIRED)
find_dependency(dv-processing 1.5.1 REQUIRED)
include_directories("/opt/inivation/boost/include")
include_directories("/usr/include")

# libcaer
find_package(libcaer REQUIRED)

# realsense
find_package(realsense2 2.50.0)
# include_directories(${realsense2_INCLUDE_DIR})
# message("test")
# message(${realsense2_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        fmt::fmt
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        # ${DV_LIBRARIES} # should replace this with dv::processing
        dv::processing
        ${realsense2_LIBRARY}
        libcaer::caer
        /usr/lib/x86_64-linux-gnu/libdl.so
        ${PROJECT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64.so
        lcm
        )

# Control (tested)
# set(THIRD_PARTY_LIBS -pthread /home/hochul/Repository/DynamicObjectsAvoidance/lib/libunitree_legged_sdk_amd64.so lcm)

enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)
