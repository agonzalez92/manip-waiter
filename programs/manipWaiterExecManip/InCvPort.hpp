// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_CV_PORT_HPP__
#define __IN_CV_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include <fstream>
#include <stdio.h>

//instrucciones para el followme
#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

//insatrucciones para el waiterbot
#define VOCAB_HELLO_TEO VOCAB4('e','l','o','t')
#define VOCAB_GO_TEO VOCAB4('g','t','e','o')
#define VOCAB_WATER_PLEASE VOCAB4('w','p','l','e')
#define VOCAB_STOP_TEO VOCAB4('s','t','e','o')

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup followMeExecutionCore
 *
 * @brief Input port of computer vision data.
 *
 */
class InCvPort : public BufferedPort<Bottle> {
    public:

        InCvPort()
        {
            follow = 0;
            a = 0;
            coordY = 0.347;
        }

        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }
        void setIVelocityControl(yarp::dev::IVelocityControl *iVelocityControl) {
            this->iVelocityControl = iVelocityControl;
        }

        void setFollow(int value);

    private:

        int follow;
        int a;
        int c;
        int i;
        double coordY;

        /** Callback on incoming Bottle. **/
        virtual void onRead(Bottle& b);

        yarp::dev::IEncoders *iEncoders;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IVelocityControl *iVelocityControl;

};

}  // namespace teo

#endif  // __IN_CV_PORT_HPP__
