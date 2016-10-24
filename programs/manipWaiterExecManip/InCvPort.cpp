// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCvPort.hpp"

namespace teo
{

/************************************************************************/
void InCvPort::setFollow(int value)
{
    follow = value;
}

/************************************************************************/


void InCvPort::onRead(Bottle& b) {

//------------------------SET right ARM INITIAL POSITION------------------------
    if (a==0)
    {
        iEncoders->getAxes(&numRobotJoints);
        CD_INFO("numRobotJoints: %d.\n",numRobotJoints);

        iPositionControl->setPositionMode();
        printf("begin MOVE TO START POSITION\n");
        double initpos[7] = {30,0,0,90,0,-30,0};
        iPositionControl->positionMove(initpos);
        bool done = false;
        while( ! done )
        {
            yarp::os::Time::delay(0.5);
            iPositionControl->checkMotionDone(&done);
            printf(".");
            fflush(stdout);
        }
        printf("end MOVE TO START POSITION\n");
        iVelocityControl->setVelocityMode();
        a=1;
    }


    //-------------------READING INPUT MESSAGES FROM VISION SENSOR--------------------
    //double x = b.get(0).asDouble(); //Data pxXpos
    //double y = b.get(1).asDouble(); //Data pxYpos
    double angle = b.get(2).asDouble(); //Angle

    //------------------------CONTROL------------------------

    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> x;

    //-- Perform forward kinematics to obtain cartesian position
    if ( ! iCartesianSolver->fwdKin(currentQ,x) )
    {
        CD_ERROR("fwdKin failed.\n");
        return;
    }

    std::vector<double> xdotd(6, 0.0);

    //-- 0.526938 0.346914 0.312769 -1.0 0.000015 -0.000015 90.003044

/*    if ( x[0] > 0.526938+0.001 )
        xdotd[0] = -0.01;

    if ( x[0] < 0.526938-0.001 )
        xdotd[0] = 0.01;

    if ( x[2] > 0.312769+0.001 )
        xdotd[2] = -0.01;

    if ( x[2] < 0.312769-0.001 )
        xdotd[2] = 0.01;
*/
    if ( (angle >= 70) && (angle < 88) )  //Correction 01. Move arm Y right.
    {
        if( x[1] > -0.45 )
            xdotd[1] = -0.05; // [1] corresponds to Y axis
//        if( x[1] > 0.25 )
//           xdotd[1] = -0.05; // [1] corresponds to Y axis
    }
    else if( (angle > 92) && (angle <= 110) )  //Correction 02. Move arm Y right.
    {
        if( x[1] < -0.25 )
            xdotd[1] = 0.05; // [1] corresponds to Y axis
//        if( x[1] < 0.45 )
//            xdotd[1] = 0.05; // [1] corresponds to Y axis
    }
    else //if(z>=88 && z<=92)
    {
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;
    if (! iCartesianSolver->diffInvKin(currentQ,xdotd,commandQdot) )
    {
        CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        return;
    }

    for(int i=0;i<commandQdot.size();i++)
    {
        if( fabs(commandQdot[i]) > DEFAULT_QDOT_LIMIT)
        {
            CD_ERROR("diffInvKin too dangerous, STOP!!!.\n");
            for(int i=0;i<commandQdot.size();i++)
                commandQdot[i] = 0;
        }
    }

    CD_DEBUG_NO_HEADER("[MOVV] ");
    for(int i=0;i<6;i++)
        CD_DEBUG_NO_HEADER("%f ",xdotd[i]);
    CD_DEBUG_NO_HEADER("-> ");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",commandQdot[i]);
    CD_DEBUG_NO_HEADER("[deg/s]\n");

    commandQdot[0] = 0.1;
    if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }

    return;
}

/************************************************************************/

}  // namespace teo

