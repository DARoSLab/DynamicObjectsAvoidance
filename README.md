# DynamicObjectsAvoidance

## Click video!!
[![Video](https://img.youtube.com/vi/YjxPUyzaNW8/0.jpg)](https://www.youtube.com/watch?v=YjxPUyzaNW8)

## Dependencies
The following packages need to be installed:

* dv-processing 
```
# Package in Ubuntu 18.04
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt update
sudo apt install dv-processing

# Python bindings
sudo apt install dv-processing-python

# Dependencies in Ubuntu 18.04
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt-get update
sudo apt-get install git gcc-10 g++-10 cmake boost-inivation libopencv-dev libeigen3-dev libcaer-dev libfmt-dev liblz4-dev libzstd-dev
```

* Sophus

```
# Install sophus from source and extract it : https://github.com/strasdat/Sophus.git
# (rename sophus-master to sophus) and move it to /usr/local/include
sudo cmake ..
sudo make
```

* Glog
```
sudo apt install libgoogle-glog-dev 
sudo apt-get install libgflags-dev
sudo apt install libgoogle-glog-dev
sudo apt-get install protobuf-compiler libprotobuf-dev
```

* GTest
```
sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
```

* g20

Install g2o and build from source: https://github.com/RainerKuemmerle/g2o


## Todo
- [ ] realsense api.h head file
- [ ] Remove realsense

## Version Requriment
OpenCV >= 4.2.0 (tested 4.4.0)
gcc compiler version >= 10.0 (tested 10.3.0)

## Issues
```bash
- gtest related compile error
sudo apt-get install cmake libgtest-dev
cd /usr/src/gtest
sudo mkdir build
sudo cmake ..
sudo make
 
# copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
sudo cp *.a /usr/lib
```

## Build in conda environment & suppress warnings
```
PISM_INSTALL_PREFIX=~/pism PATH=$NOCONDA_PATH cmake .. -Wno-dev
```
