#include "DynamicObjectsAvoidance/control.h"

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::Avoid(double robotDir) 
{
    // Get y velocity from event
    // robotDir = event->GetCmdDir(); 
    cmd.mode = 2;
    cmd.gaitType = 1;
    // Avoid by walking sideways
    cmd.velocity[1] = robotDir; // -1  ~ +1
    cmd.bodyHeight = 0.1;
    // printf("Moving: %f\n", robotDir);
}


void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    // Check if action is initialized
    firstMotion = event->GetInitAct();

    // Debug prints
    // printf("[%d]  EVENT (X,Y) VEL: (%d, %d)\n", motiontime, obsXvel, obsYvel);
    // printf("[%d]\n", avoid_mode);
    // printf("EVENT (X,Y) VEL: (%d, %d) WINDOW VEL DIR: %d", obsXvel, obsYvel, windowVelDir);
    // printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);
    // printf("%f %f %f\n", state.motorState[3].q, state.motorState[4].q, state.motorState[5].q);

    // Initialize every ctrl step
    cmd.mode = 0; // 0:idle, default stand; 1:forced stand; 2:walk continuously
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
    if(motiontime > 1000 && motiontime < 2000){
        cmd.mode = 5;
    } else if(motiontime > 3000 && motiontime < 4000){
        cmd.mode = 6;
    } else if(motiontime > 4000 && motiontime < 5000){
        cmd.mode = 0;
    } else if (motiontime > 5000){
        
        // Start moving
        if (firstMotion == true && targetTime == 0 && targetTime2 == 0){
            targetTime = motiontime + 1000;
            targetTime2 = targetTime + 2000;
            robotDir = event->GetCmdDir(); 
            // printf("First motion control: %f", robotDir);
            firstMotion = false;
        } 
        // printf("Motiontime: %d\n", motiontime);
        // printf("Target time: %d\n", targetTime);
        // printf("Target time2: %d\n", targetTime2);
        
        if (motiontime < targetTime){
            // printf("1\n");
            // printf("actual motion control: %f", robotDir);
            Avoid(robotDir);
            // cv::imshow("ts_color", ts_color);
            // cv::waitKey(1);   
        } else if (motiontime >= targetTime && motiontime < targetTime2) {
            // frame = event->GetImage();

            targetTime = 0;
            // printf("2\n");

            // comment out Shifan
            // cv::namedWindow("Dynamic obstacle avoidance", cv::WINDOW_NORMAL);
            // cv::imshow("Dynamic obstacle avoidance", frame);
            // cv::waitKey(1);   
            Avoid(0.0f);
        } else if (motiontime >= targetTime2) {
            targetTime2 = 0;
            // printf("3\n");
            // printf("Stopped\n");
        }
    }

    udp.SetSend(cmd);
}