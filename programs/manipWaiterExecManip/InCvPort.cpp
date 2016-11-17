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

    std::string strategy = DEFAULT_STRATEGY;

    if(strategy == "velocity")
        strategyVelocity(b);
    else if(strategy == "positionDirect")
        strategyPositionDirect(b);
    else
        CD_ERROR("Unknown strategy!!!\n");

}

/************************************************************************/

void InCvPort::strategyPositionDirect(Bottle& b)
{
    if (a==0)
    {
        preprogrammedInitTrajectory();
        //iPositionDirect->setPositionDirectMode();
        iPositionControl->setPositionMode();
        a=1;
    }

    //-------------------READING INPUT MESSAGES FROM VISION SENSOR--------------------
    //double x = b.get(0).asDouble(); //Data pxXpos
    //double y = b.get(1).asDouble(); //Data pxYpos
    double angle = b.get(2).asDouble(); //Angle

    //------------------------CONTROL------------------------

    //-- 0.526938 0.346914 0.312769 -1.0 0.000015 -0.000015 90.003044

    //-- Obtain current joint position

    //----- Obtain current joint position ---------------
    std::vector<double> currentQ(numRobotJoints);
    std::vector<double> desireQ(numRobotJoints);

    if ( ! iEncoders->getEncoders( currentQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }
    //---------------------------------------------------


    //------ Obtain current cartesian position ----------
    std::vector<double> currentX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
    }
    //---------------------------------------------------


    std::vector<double> desireX;

    desireX = currentX; // 6 = 6

    if ( (angle >= 70) && (angle < 88) )    {   //Correction 01. Move arm Y right.
        if( currentX[1] >= 0.25 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            desireX[1] = currentX[1] - 0.02;
            pepinito = 1;
        }
        else if( currentX[1] < 0.25 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;
            desireX[1] = 0.346914;
            pepinito = 3;


            //if( ! iPositionControl->positionMove( beforeQ.data() ))            {
            //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
            //}
            // beforeQ = currentQ; ???? por confirmar
            //return;
        }
    }
    else if( (angle > 92) && (angle <= 110) )    {   //Correction 02. Move arm Y left.
        if( currentX[1] <= 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] + 0.02;
             pepinito = 1;
        }
        else if( currentX[1] > 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             return;
             desireX[1] = 0.346914;
             pepinito = 3;


             //if( ! iPositionControl->positionMove( beforeQ.data() ))                    {
             //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
             //}
             // beforeQ = currentQ; ???? por confirmar
             //return;
        }
    }
    else{      //if(z>=88 && z<=92)
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        return;
        desireX[1] = 0.346914;
        pepinito = 3;


        //if( ! iPositionControl->positionMove( beforeQ.data() ))        {
        //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
        //}
        // beforeQ = currentQ; ???? por confirmar
        //return;
    }
    desireX[0] = 0.526938;
    desireX[2] = 0.312769;
    desireX[3] = -1;
    desireX[4] = 0;
    desireX[5] = 0;
    desireX[6] = 90;

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) ){
        CD_ERROR("invKin failed.\n");
    }

    CD_DEBUG_NO_HEADER("[currentX]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentX[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[desireX]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    //--------------------------------------------------------------
/*
    CD_DEBUG_NO_HEADER("[beforeQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[currentQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[desireQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireQ[i]);
    CD_DEBUG_NO_HEADER("\n");
*/
    switch (pepinito) {
    case 1:
        if( ! iPositionControl->positionMove( desireQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    case 2:
        if( ! iPositionControl->positionMove( currentQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    case 3:
        currentQ=std::vector<double>(initpos);
        /*
        iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, 0);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -90);
        iPositionControl->positionMove(4, 0);
        iPositionControl->positionMove(5, 30);
        iPositionControl->positionMove(6, 0);
        */
        if( ! iPositionControl->positionMove( currentQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    default:
        break;
    }

    //beforeQ = currentQ;

    return;


/*
    //----- Obtain current joint position ---------------
    std::vector<double> currentQ(numRobotJoints);
    std::vector<double> desireQ(numRobotJoints);

    if ( ! iEncoders->getEncoders( currentQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }
    //---------------------------------------------------


    //------ Obtain current cartesian position ----------
    std::vector<double> currentX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
    }
    //---------------------------------------------------


    std::vector<double> desireX;

    desireX = currentX; // 6 = 6

    if ( (angle >= 70) && (angle < 88) )    {   //Correction 01. Move arm Y right.
        if(( currentX[1] - 0.02 ) >= 0.25 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            desireX[1] = currentX[1] - 0.02;
        }
        else if(( currentX[1] - 0.02 ) < 0.25 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            currentQ = beforeQ;


            //if( ! iPositionControl->positionMove( beforeQ.data() ))            {
            //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
            //}
            // beforeQ = currentQ; ???? por confirmar
            //return;
        }
    }
    else if( (angle > 92) && (angle <= 110) )    {   //Correction 02. Move arm Y left.
        if(( currentX[1] + 0.02 ) <= 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] + 0.02;
        }
        else if(( currentX[1] + 0.02 ) > 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             currentQ = beforeQ;


             //if( ! iPositionControl->positionMove( beforeQ.data() ))                    {
             //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
             //}
             // beforeQ = currentQ; ???? por confirmar
             //return;
        }
    }
    else{      //if(z>=88 && z<=92)
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        currentQ = beforeQ;


        //if( ! iPositionControl->positionMove( beforeQ.data() ))        {
        //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
        //}
        // beforeQ = currentQ; ???? por confirmar
        //return;
    }

    currentX[3] = -1;
    currentX[4] = 0;
    currentX[5] = 0;
    currentX[6] = 90;

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) ){
        CD_ERROR("invKin failed.\n");
    }

    CD_DEBUG_NO_HEADER("[beforeQ]");
    for(int i=0;i<6;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[currentQ]");
    for(int i=0;i<6;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[desireQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireQ[i]);
    CD_DEBUG_NO_HEADER("\n");

    //if( ! iPositionDirect->setPositions( desireQ.data() ))
    if( ! iPositionControl->positionMove( desireQ.data() ))
    {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");
    }
    beforeQ = currentQ;

    return;
    */
/*
 * //-----------print example---------------------
 *
    CD_DEBUG_NO_HEADER("[MOVL] [%f] ",movementTime);
    for(int i=0;i<6;i++)
        CD_DEBUG_NO_HEADER("%f ",commandXdot[i]);
    CD_DEBUG_NO_HEADER("-> ");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",commandQdot[i]);
    CD_DEBUG_NO_HEADER("[deg/s]\n");
*/




}

/************************************************************************/

void InCvPort::strategyVelocity(Bottle& b)
{
    if (a==0)
    {
        preprogrammedInitTrajectory();
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

bool InCvPort::preprogrammedInitTrajectory()
{
    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);

    iPositionControl->setPositionMode();
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

    //----- designate initial position ---------------
    std::vector<double> beforeQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( beforeQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return false;
    }
    yarp::os::Time::delay(1);
    return true;
}

/************************************************************************/

}  // namespace teo

