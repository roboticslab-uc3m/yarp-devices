// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_examples_cpp
 * \defgroup exampleRemoteControlboard exampleRemoteControlboard
 *
 * @brief This example connects to a running \ref teoSim program.
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
cd $TEO_ROOT/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./testTeoSimRightArm
\endverbatim
 *
 */

#include "ExampleRemoteControlboard.hpp"

int main(int argc, char **argv)
{
    teo::ExampleRemoteControlboard mod;
    return mod.run(argc,argv);
}

