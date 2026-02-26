#include "robot.h"

// const float route[4][3] = {{0.818,0,-PI/2},{0.818,-0.396,-PI},{0,-0.396,(3*-PI)/2},{0,0,0}};
// const float route[4][3] = {{2.05,0.409,-PI/2},{2.05,-0.417,-PI},{0,0,0}};
// const float route[5][3] = {{0.398,1.228,-45},{-0.417,2.05,170},{0.389,0.408,-20},{0,2.05,-180},{0,0,0}};
const float route[6][3] = {{180,175,512},{200,-10,512},{280,-10,512},{280,50,512},{200,75,512},{175,175,512}};
const int points = 6;

int point = 0;

int incomingByte = 0;
int setPos1 = 0;
int setPos2 = 0;
int inputPos = 0;
Pose destPose;

/**
 * Handles moving robot along a set path
 */
void Robot::ControllerLoop(void) {
    #ifdef __CONTROLLER_DEBUG__
        TeleplotPrint("State", robotState);
        TeleplotPrint("Point",point);
    #endif
    #ifdef __SERIAL_CONTROL__
        SerialInputLoop();
    #else
        if(robotState == ROBOT_IDLE) {
            if(point <= points - 1) {
                if(route[point][2] > 360) {
                    Serial.println("Moving arm");
                    setArmPos(route[point][0], route[point][1]);
                    robotState = ARM_ACTIVE;
                    point++;
                } else {
                    Pose destPose;  

                    destPose.x = route[point][1];
                    destPose.y = route[point][0];
                    destPose.theta = route[point][2] * DEG_TO_RAD;

                    SetDestination(destPose);
                    point++;
                }
            } else {
                robotState = ROBOT_DONE;
                Serial.println("-> Loop Done");
            }
        } else if (robotState != ROBOT_DONE) {
            RobotLoop();
        }
    #endif
    armLoop();
}

void Robot::SerialInputLoop() {
    if (Serial.available() > 0) {
        incomingByte = int(Serial.read())-48;
        switch (inputPos) {
            case 0:
                setPos1 = incomingByte*100;
                inputPos++;
                break;
            case 1:
                setPos1 += incomingByte*10;
                inputPos++;
                break;
            case 2:
                setPos1 += incomingByte;
                inputPos++;
                TeleplotPrint("Set point X:", setPos1);
                break;
            case 3:
                setPos2 = incomingByte*100;
                inputPos++;
                break;
            case 4:
                setPos2 += incomingByte*10;
                inputPos++;
                break;
            case 5:
                setPos2 += incomingByte;
                inputPos++;
                TeleplotPrint("Set point Y:", setPos2);
                setArmPos(setPos1, setPos2);
                robotState = ARM_ACTIVE;
                TeleplotPrintTuple("Set pose:", destPose.x*RAD_TO_DEG, destPose.y*RAD_TO_DEG);
                break;
            case 6:
                inputPos = 0;
                Serial.println("-- Reset --");
                break;
        }
    };
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