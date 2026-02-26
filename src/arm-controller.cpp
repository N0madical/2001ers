#include "robot.h"

float mainAngle = 0;
float secondaryAngle = 0;
float goToPos1 = PI/2;
float goToPos2 = PI/2;

void Robot::setArmPos(int x, int y) {
    //Set up where 90 degrees on both servos forms an L

    // Set arm length
    const int L = 175;

    // I got it from google, search up "2-link planar arm inverse kinematics"
    // Modified to fit our robot
    uint32_t r = pow(y,2) + pow(x,2);
    float phi = acosf((r-(2*pow(L,2)))/(2*pow(L,2)));
    float theta1 = atan2f(x,y)-atan2f(L*sinf(phi),L+L*cosf(phi));
    float theta2 = theta1 + phi;
    Pose setAngles;
    mainAngle = PI/2 - theta1, secondaryAngle = theta2;
    TeleplotPrintTuple("ArmPosOut", mainAngle, secondaryAngle);
}

void Robot::armLoop(void) {
    if(robotState == ARM_ACTIVE) {
        if((abs(mainAngle - goToPos1) >= 0.001) || (abs(secondaryAngle - goToPos2) >= 0.001)) {
            goToPos1 += (goToPos1 - mainAngle) < 0 ? 0.001 : -0.001;
            goToPos2 += (goToPos2 - secondaryAngle) < 0 ? 0.001 : -0.001;
            TeleplotPrintTuple("X diff", goToPos1, mainAngle);
            TeleplotPrintTuple("Y diff", goToPos2, secondaryAngle);
        } else {
            robotState = ROBOT_IDLE;
        }

        SetServoAngle1(goToPos1*RAD_TO_DEG);
        SetServoAngle2(goToPos2*RAD_TO_DEG);
    }
}