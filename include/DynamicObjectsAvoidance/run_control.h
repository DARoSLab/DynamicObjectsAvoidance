/*
    Executes avoiding behavior {face right, face left, face down, body low} via high-level control
*/
#include "DynamicObjectsAvoidance/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include "DynamicObjectsAvoidance/dynamic_objects_detection.h"


using namespace UNITREE_LEGGED_SDK;

namespace Avoid_behavior {
  constexpr int INITIAL = 0;
  constexpr int FACE_RIGHT = 1;
  constexpr int FACE_LEFT = 2;
  constexpr int FACE_UP = 3;
  constexpr int FACE_DOWN = 4;
  constexpr int BODY_UP = 5;
  constexpr int BODY_DOWN = 6;
  constexpr int WALK_RIGHT = 7;
};

class Custom
{
public:
    Custom(uint8_t level, DynamicObjectsAvoidance::DynamicObjectsAvoidance::Ptr doa): 
      safe(LeggedType::Go1), 
       udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)), // WIFI  
      //  udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)) // WIRED
      event(doa)
      
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    DynamicObjectsAvoidance::DynamicObjectsAvoidance::Ptr event;

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};

    // const int windowSize = 5;
    int motiontime = 0;
    int targetTime = 0;
    float dt = 0.002;  // 0.001~0.01
    int avoid_mode;
    bool firstMotion = false;
    
protected:
    // int obsXvel;
    // int obsYvel;
    double robotDir = 0; // y velocity

};
