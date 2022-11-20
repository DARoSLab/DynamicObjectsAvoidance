/*
    Executes avoiding behavior {face right, face left, face down, body low} via high-level control
*/
#include "DynamicObjectsAvoidance/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

namespace Avoid_behavior {
  constexpr int FACE_RIGHT = 0;
  constexpr int FACE_LEFT = 1;
  constexpr int FACE_UP = 2;
  constexpr int FACE_DOWN = 3;
  constexpr int BODY_UP = 4;
  constexpr int BODY_DOWN = 5;
  constexpr int INITIAL = 6;
};

class Custom
{
public:
    Custom(uint8_t level): 
      safe(LeggedType::Go1), 
       udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)) // WIFI  
    //    udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)) // WIRED
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;  // 0.001~0.01
    int avoid_mode = 0;
};
