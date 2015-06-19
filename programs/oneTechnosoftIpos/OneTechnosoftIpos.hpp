// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_MANIPULATION__
#define __LAUNCH_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <stdlib.h>

#include "ICanBusSharer.h"
#include "ColorDebug.hpp"

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup oneTechnosoftIpos
 *
 * @brief Launches one CAN bus drivers and one TechnosoftIpos.
 * 
 */
class OneTechnosoftIpos : public RFModule {

    public:
        OneTechnosoftIpos();
        bool configure(ResourceFinder &rf);

    protected:

        virtual double getPeriod() {return 1.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        /** A CAN device. */
        yarp::dev::PolyDriver canBusDevice;
        CanBusHico* iCanBus;

        struct can_msg buffer;

        yarp::dev::PolyDriver dd;

        teo::ICanBusSharer *iCanBusSharer;
        yarp::dev::IPositionControlRaw *pos;
        yarp::dev::IEncodersRaw *enc;
        yarp::dev::IVelocityControlRaw *vel;
        yarp::dev::IControlModeRaw *ctrl;

};

}  // namespace teo

#endif  // __LAUNCH_MANIPULATION__

