// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getAxes(int * ax)
{
    *ax = 1;
    return true;
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::setPosition(int j, double ref)
{
    yCTrace(TXT, "%d %f", j, ref);

    if (j != 0) return false;

    char cmdByte[1];

    if (ref == 0.0)
    {
        cmdByte[0] = 'a';
    }
    else if (ref == 1.0)
    {
        cmdByte[0] = 'b';
    }
    else
    {
        return false;
    }

    if (!iSerialDevice->send(cmdByte, 1))
    {
        return false;
    }

    lastTarget = ref;
    return true;
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::setPositions(const double * refs)
{
    return setPosition(0, refs[0]);
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::setPositions(int n_joint, const int * joints, const double * refs)
{
    return setPosition(joints[0], refs[0]);
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::getRefPosition(int joint, double * ref)
{
    yCTrace(TXT, "%d", joint);
    *ref = lastTarget;
    return true;
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::getRefPositions(double * refs)
{
    return getRefPosition(0, &refs[0]);
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::getRefPositions(int n_joint, const int * joints, double * refs)
{
    return getRefPosition(joints[0], &refs[0]);
}

// ----------------------------------------------------------------------------------------
