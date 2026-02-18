#include "robot.h"

int setVelocity = 0;

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/
void Robot::RobotLoop(void)
{
     /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);

        ControllerLoop();

        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
        } else if (robotState == ROBOT_SET_HEADING) {
            SetHeading();
        }
    }
}
