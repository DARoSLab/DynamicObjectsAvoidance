#include "DynamicObjectsAvoidance/run_control.h"


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

// Run avoiding behavior
void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("[%d]  IMU STATE: %f\n", motiontime, state.imu.quaternion[2]);

    // Initialize
    cmd.mode = 0;    
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    // Avoid based on time 
    if(motiontime > 1000 && motiontime < 2000){
        avoid_mode = Avoid_behavior::FACE_UP;
    } else if (motiontime > 3000 && motiontime < 4000){
        avoid_mode = Avoid_behavior::FACE_DOWN;
    } else if (motiontime > 5000 && motiontime < 6000){
        avoid_mode = Avoid_behavior::FACE_RIGHT;
    } else if (motiontime > 7000 && motiontime < 8000){
        avoid_mode = Avoid_behavior::FACE_LEFT;
    } else if (motiontime > 9000 && motiontime < 10000){
        avoid_mode = Avoid_behavior::BODY_DOWN;
    } else {
        avoid_mode = Avoid_behavior::INITIAL;
    }

    // Pitch control
    if (avoid_mode == Avoid_behavior::FACE_UP){
        cmd.mode = 1;
        cmd.euler[1] = -0.2;
    }

    if (avoid_mode == Avoid_behavior::FACE_DOWN){
        cmd.mode = 1;
        cmd.euler[1] = 0.2;
    }

    // Yaw control
    if (avoid_mode == Avoid_behavior::FACE_RIGHT){
        cmd.mode = 1;
        cmd.euler[2] = -0.2;
    }

    if (avoid_mode == Avoid_behavior::FACE_LEFT){
        cmd.mode = 1;
        cmd.euler[2] = 0.2;
    }

    // Body pos control
    if (avoid_mode == Avoid_behavior::BODY_DOWN){
        cmd.mode = 1;
        cmd.bodyHeight = -0.2;
    }

    if (avoid_mode == Avoid_behavior::INITIAL){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0;
    }


    // // Body low
    // if(motiontime > 6000 && motiontime < 7000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = -0.2;
    // }

    // // Body high
    // if(motiontime > 7000 && motiontime < 8000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = 0.1;
    // }

    // // Body initial
    // if(motiontime > 8000 && motiontime < 9000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = 0.0;
    // }


    // MOTOR TORQUE
    // // Motor initial
    // if(motiontime > 9000 && motiontime < 11000){
    //     cmd.mode = 5;
    // }

    // // Motor torque
    // if(motiontime > 11000 && motiontime < 13000){
    //     cmd.mode = 6;
    // }

    // // Initialize
    // if(motiontime > 13000 && motiontime < 14000){
    //     cmd.mode = 0;
    // }

    // WALKING
    // if(motiontime > 14000 && motiontime < 18000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 2;
    //     cmd.velocity[0] = 0.4f; // -1  ~ +1
    //     cmd.yawSpeed = 2;
    //     cmd.footRaiseHeight = 0.1;
    //     // printf("walk\n");
    // }
    // if(motiontime > 18000 && motiontime < 20000){
    //     cmd.mode = 0;
    //     cmd.velocity[0] = 0;
    // }
    // if(motiontime > 20000 && motiontime < 24000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 1;
    //     cmd.velocity[0] = 0.2f; // -1  ~ +1
    //     cmd.bodyHeight = 0.1;
    // }
    
    // if(motiontime>24000 ){
    //     cmd.mode = 1;
    // }

    udp.SetSend(cmd);
}


int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
