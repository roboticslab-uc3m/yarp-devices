// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __roboticslab_SIM_EXAMPLE_HPP__
#define __roboticslab_SIM_EXAMPLE_HPP__

#include <yarp/os/Network.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace roboticslab
{

class ExampleRemoteControlBoard
{
public:
    int run(int argc, char **argv);

private:
    yarp::os::Network yarp;
    yarp::dev::PolyDriver dd;
    yarp::dev::IPositionControl * pos;
    yarp::dev::IEncoders * enc;
    yarp::dev::IVelocityControl * vel;
};

}  // namespace roboticslab

#endif // __roboticslab_SIM_EXAMPLE_HPP__
