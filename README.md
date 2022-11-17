# DynamicObjectsAvoidance


## Todo
realsense api.h head file

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
