#include "robot.h"


void Robot::SetServoAngle1(const int angle)
{
    servoMainRot = angle;
    servo1.write(angle + servoMainOffset);
}

void Robot::SetServoAngle2(const int angle) {
    servoSecondRot = angle;
    servo2.write(angle + servoSecondOffset);
}
