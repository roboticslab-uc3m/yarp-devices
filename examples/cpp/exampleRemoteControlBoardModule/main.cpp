// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleRemoteControlBoardModule exampleRemoteControlBoardModule
 *
 * @brief This example connects to a remote control board device (e.g. @ref CanBusBroker) as a module.
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2016 Universidad Carlos III de Madrid;
 *
 * Authors: raulfdzbis, jgvictores
 *
 * Contribs: Paul Fitzpatrick and Giacomo Spigler (YARP dev/motortest.cpp example)
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd $roboticslab_ROOT/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./testroboticslabSimRightArm
\endverbatim
 *
 */

#include "ExampleRemoteControlBoard.hpp"

int main(int argc, char **argv)
{
    roboticslab::ExampleRemoteControlBoard mod;
    return mod.run(argc,argv);
}
