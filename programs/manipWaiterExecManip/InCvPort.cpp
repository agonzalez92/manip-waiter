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
   if (a==0){
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

//------------------------CORRECTION OUTPUTS------------------------

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

//    if (outputCartesian.size() > 0)
  //      pOutPort->write(outputCartesian);

}

/************************************************************************/

}  // namespace teo

