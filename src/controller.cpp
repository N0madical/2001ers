#include "robot.h"

// const float route[4][3] = {{0.818,0,-PI/2},{0.818,-0.396,-PI},{0,-0.396,(3*-PI)/2},{0,0,0}};
// const float route[4][3] = {{2.05,0.409,-PI/2},{2.05,-0.417,-PI},{0,0,0}};
const float route[5][3] = {{0.398,1.228,-45},{-0.417,2.05,170},{0.389,0.408,-20},{0,2.05,-180},{0,0,0}};
const int points = 5;

int point = 0;

/**
 * Handles moving robot along a set path
 */
void Robot::ControllerLoop(void) {
    #ifdef __CONTROLLER_DEBUG__
        TeleplotPrint("State", robotState);
        TeleplotPrint("Point",point);
    #endif
    if(robotState == ROBOT_IDLE) {
        if(point <= points - 1) {
            Pose destPose;  

            destPose.x = route[point][1];
            destPose.y = route[point][0];
            destPose.theta = route[point][2] * DEG_TO_RAD;

            SetDestination(destPose);
            point++;
        } else {
            robotState = ROBOT_DONE;
            Serial.println("-> Loop Done");
        }
    } else if (robotState != ROBOT_DONE) {
        RobotLoop();
    }
}

/**
 * Sets a destination for the robot controller
 */
void Robot::SetDestination(const Pose& dest)
{
    //Turn on LED to signifiy active
    chassis.LightOn();

    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}