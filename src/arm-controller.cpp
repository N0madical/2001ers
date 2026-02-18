#include "robot.h"

Pose Robot::armPosToServo(int pos[2]) {

}

Pose Robot::getArmPos(void) {
    int armPos1[2] = {0.175 * cosf(servoMainRot), 0.175 * sinf(servoMainRot)};
    Pose armPos;
    armPos.x = armPos1[1] + cosf(servoSecondRot);
    armPos.y = armPos1[2] + sinf(servoSecondRot);

    return armPos;
}