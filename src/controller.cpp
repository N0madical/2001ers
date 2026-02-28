#include "robot.h"

// const float route[5][4] = {{0.398,1.228,-45,0},{-0.417,2.05,170,0},{0.389,0.408,-20,0},{0,2.05,-180,0},{0,0,0,0}};
// const float route[8][4] = {{180,175,512,1},{200,-10,512,1},{280,-10,512,1},{280,50,512,1},{200,75,512,1},{175,175,512,1},{0.3,0.2,-45,0},{0,0.4,180,0}};
const float route[20][4] = {
    {200,30,0,1},
    {200,-2,0,1},
    {270,-2,0,1},
    {270,50,0,1},
    {175,50,0,1},
    {175,200,0,1},
    {225,192,0,1},
    {225,175,0,1},
    {150,175,0,1},
    {125,212,0,1},
    {210,212,0,1},
    {210,245,0,1},
    {150,245,0,1},
    {150,290,0,1},
    {210,270,0,1},
    {210,260,0,1},
    {100,265,0,1},
    {125,125,0,1},
    {0.15,0.2,-45,0},
    {-0.02,0.42,180,0}};
const int points = 20;

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
        armLoop();
    #else
        if(robotState == ROBOT_IDLE) {
            if(point <= points - 1) {
                if(route[point][3] == 1) {
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
            armLoop();
        }
    #endif
}

void Robot::SerialInputLoop() {
    if (Serial.available() > 0) {
        robotState = ARM_ACTIVE;
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