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

//------------------------SET LEFT ARM INITIAL POSITION------------------------
    if (a==0)
    {
        iEncoders->getAxes(&numRobotJoints);
        CD_INFO("numRobotJoints: %d.\n",numRobotJoints);

        printf("begin MOVE TO START POSITION\n");
        double initpos[7] = {-30,0,0,-90,0,30,0};
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

    std::vector<double> xdotd;

    if(angle>=70 && angle<88)  //Correction 01. Move arm Y left.
    {
        //if((coordY+0.05)<=0.55)
        //{
            /*outputCartesian.addString("movv");
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0.05);
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );*/
            //Time::delay(0.1);
        /*}
        else if((coordY+0.05)>0.55)
        {
            printf("BOTTLE FALL right!! \n");
        }*/

    }
    else if(angle>92 && angle<=110)  //Correction 02. Move arm Y right.
    {
        //printf("value coordYleft: %f", coordY);
        //if((coordY-0.05)>=0.25)
        //{
            /*outputCartesian.addString("movv");
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( -0.05);
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );*/
            //Time::delay(1);
        /*}
        else if((coordY-0.05)<0.05){
            printf("BOTTLE FALL left!! \n");
        }*/
    }
    else //if(z>=88 && z<=92)
    {
        /*printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        outputCartesian.addString("movv");
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );*/
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

    if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }

}

/************************************************************************/

}  // namespace teo

