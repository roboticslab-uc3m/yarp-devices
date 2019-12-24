// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <ColorDebug.h>

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
    CD_DEBUG("(%d, %f)\n", j, ref);

    if (j != 0) return false;

    unsigned char cmdByte;

    if (ref == 0.0)
    {
        cmdByte = 'a';
    }
    else if (ref == 1.0)
    {
        cmdByte = 'b';
    }
    else
    {
        return false;
    }

    if (serialport_writebyte(fd, cmdByte) == -1)
    {
        return false;
    }

    lastTarget = ref;
    return true;
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::setPositions(const double * refs)
{
    CD_DEBUG("\n");
    return setPosition(0, refs[0]);
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::setPositions(int n_joint, const int * joints, const double * refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    return setPosition(joints[0], refs[0]);
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::getRefPosition(int joint, double * ref)
{
    CD_DEBUG("(%d)\n", joint);
    *ref = lastTarget;
    return true;
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::getRefPositions(double * refs)
{
    CD_DEBUG("\n");
    return getRefPosition(0, &refs[0]);
}

// ----------------------------------------------------------------------------------------

bool TextilesHand::getRefPositions(int n_joint, const int * joints, double * refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    return getRefPosition(joints[0], &refs[0]);
}

// ----------------------------------------------------------------------------------------
