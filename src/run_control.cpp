#include "DynamicObjectsAvoidance/run_control.h"

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    
    // Get pixel velocity from event
    // obsXvel = int(event->GetXVel());
    // obsYvel = int(event->GetYVel());

    // Get direction command
    robotDir = event->GetCmdDir(); 

    // Get window velocity direction
    // if (obsXvel < 0 && obsYvel > 0) {
    //     windowVelDir = 0;
    // } else if (obsXvel > 0 && obsYvel > 0) {
    //     windowVelDir = 1;
    // } else if (obsXvel > 0 && obsYvel < 0) {
    //     windowVelDir = 2;
    // } else if (obsXvel < 0 && obsYvel < 0) {
    //     windowVelDir = 3;
    // }

    // Debug prints
    // printf("[%d]  EVENT (X,Y) VEL: (%d, %d)\n", motiontime, obsXvel, obsYvel);
    // printf("[%d]\n", avoid_mode);
    // printf("EVENT (X,Y) VEL: (%d, %d) WINDOW VEL DIR: %d", obsXvel, obsYvel, windowVelDir);
    // printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);
    // printf("%f %f %f\n", state.motorState[3].q, state.motorState[4].q, state.motorState[5].q);

    // Initialize every ctrl step
    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
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

    // Need this for now to operate
    /*
    if(motiontime > 1000 && motiontime < 2000){
        cmd.mode = 5;
    } else if(motiontime > 3000 && motiontime < 4000){
        cmd.mode = 6;
    } else if(motiontime > 4000 && motiontime < 5000){
        cmd.mode = 0;
    } else if (motiontime> 5000){
        if (obsXvel > 50 && obsXvel < 100 && firstMotion == false) {
            avoid_mode = Avoid_behavior::FACE_RIGHT;
            targetTime = motiontime + 1000;
        } else if (obsXvel < -50 && obsXvel > -100 && firstMotion == false) {
            avoid_mode = Avoid_behavior::FACE_LEFT;
            targetTime = motiontime + 1000; 
        }
        
        // Avoid by walking sideways
        if (avoid_mode == Avoid_behavior::FACE_RIGHT) {
            printf("[%d] TURNING RIGHT\n", motiontime);
            // cmd.mode = 1;
            // cmd.euler[2] = 0.2;
            // firstMotion = true;
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[1] = 0.2f; // -1  ~ +1
            cmd.bodyHeight = 0.1;
        } 

        if (avoid_mode == Avoid_behavior::FACE_LEFT) {
            printf("[%d] TURNING LEFT\n", motiontime);
            // cmd.mode = 1;
            // cmd.euler[2] = 0.2;
            // firstMotion = true;
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[1] = -0.2f; // -1  ~ +1
            cmd.bodyHeight = 0.1;
        } 

        if (targetTime == motiontime){
            avoid_mode = Avoid_behavior::INITIAL;
            cmd.mode = 0;
            firstMotion = false;
            targetTime = 0;
        }
        
    }
    */

    udp.SetSend(cmd);
}