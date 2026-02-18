#pragma once

#include "chassis.h"

class Robot
{
    protected:
        /**
         * robotState is used to track the current task of the robot. You will add new states as 
         * the term progresses.
         */
        enum ROBOT_STATE 
        {
            ROBOT_IDLE,
            ROBOT_DRIVE_TO_POINT,
            ROBOT_SET_HEADING,
            ROBOT_DONE,
        };
        ROBOT_STATE robotState = ROBOT_IDLE;

        /* Define the chassis*/
        Chassis chassis;

        // For managing key presses
        String keyString;

        /**
         * For tracking current pose and the destination.
         */
        Pose currPose;
        Pose destPose;
        float omegaI;
        
    public:
        Robot(void) {keyString.reserve(10);}
        void InitializeRobot(void);

        // /* Controller methods */
        void ControllerLoop(void);

    protected:
        /* State changes */    
        void EnterIdleState(void);

        // /* Navigation methods.*/
        void UpdatePose(const Twist& u);
        void SetDestination(const Pose& destination);
        void DriveToPoint(void);
        void SetHeading(void);
        bool CheckReachedDestination(void);
        Twist GetDestDistance(void);
        void HandleDestination(void);

        void RobotLoop(void);
};
