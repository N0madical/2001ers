/**
 * Navigation routines
*/

#include "robot.h"

/** 
 * Uses recent odometry to calculate current position
*/
void Robot::UpdatePose(const Twist& twist)
{
    //twist.v = forward speed (m/s), twist.omega = angular speed (deg/s)

    // Calculate half-angle
    const float thetaMid = currPose.theta + (0.5f * twist.omega);

    // Perform odometry. Positive omega is clockwise. Moving on x-y (2D) plane
    currPose.x += twist.v * cosf(thetaMid);
    currPose.y += twist.v * sinf(thetaMid);
    currPose.theta += twist.omega;

    //wrap to a circle
    currPose.theta = fmod((3*PI) + currPose.theta, 2*PI) - PI;

    #ifdef __NAV_DEBUG__
        TeleplotPrint("Twist u", twist.u);
        TeleplotPrint("Twist v", twist.v);
        TeleplotPrint("x", currPose.x);
        TeleplotPrint("y", currPose.y);
        TeleplotPrint("theta", currPose.theta);
    #endif
}

/** Returns distance to destination as a Twist */
Twist Robot::GetDestDistance(void) 
{
    Twist distance;

    // distance.omega = fmod(PI + destPose.theta - currPose.theta, 2*PI) - PI; //Absolute direction to destination theta
    distance.omega = atan2((destPose.y - currPose.y), (destPose.x - currPose.x)) - currPose.theta; //Returns shortest angle to destination point between π and -π
    distance.omega = fmod((3*PI) + distance.omega, 2*PI) - PI;
    distance.v = sqrtf(pow((destPose.x - currPose.x), 2) + pow((destPose.y - currPose.y), 2)); //Returns distance to destination using Pythagoreas' Theorem

    return distance;
}

/** Handles nav routines to drive to a point */
void Robot::DriveToPoint(void)
{
    Twist distance = GetDestDistance();

    int runSpeed = 150;

    // Find the different efforts and speeds needed to achieve a destination
    // float kTheta = distance.v > 0.01 ? 2000 * (1 - (1/(distance.v*100))) : 0;
    float effortOmega = distance.omega * 80; // sqrtf(2000*abs(distance.omega));
    float effort = (runSpeed + (1/(distance.v*100))) * pow(cos(distance.omega),25);

    #ifdef __NAV_DEBUG__
            TeleplotPrint("Effort", effort);
            TeleplotPrint("Delta V", distance.v);
            TeleplotPrint("Delta Theta", distance.omega);
            TeleplotPrint("Effort Theta", effortOmega);
    #endif

    // Drive the motors to move to the destination
    chassis.SetMotorEfforts(int(effort + effortOmega), int(effort - effortOmega));
    
    if(distance.v <= 0.01 /*meters*/) {
        robotState = ROBOT_SET_HEADING;
    }
}

/** Handles nav routines to rotate to a heading */
void Robot::SetHeading() {
    float effort = (fmod((3*PI) + (destPose.theta - currPose.theta), 2*PI) - PI) * 200;
    effort = effort >= 0 ? min(effort, 200) : max(effort, -200);

    #ifdef __NAV_DEBUG__
            TeleplotPrint("Effort", effort);
    #endif

    chassis.SetMotorEfforts(int(effort), int(-effort));

    if(abs(effort) < 25) {
        HandleDestination();
    }
}

/** Stop the robot and tell the controller it is ready for next operation */
void Robot::HandleDestination(void)
{
    EnterIdleState();
    chassis.LightOff();
}