// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <algorithm>

#include <ColorDebug.h>

// ############################ IEncoders Related ############################

bool roboticslab::DextraControlboardUSB::resetEncoder(int j)
{
    CD_DEBUG("(%d)\n",j);
    return setEncoder(j, 0.0);
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::resetEncoders()
{
    CD_DEBUG("\n");
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setEncoder(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    setSetpoint(j, val);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setEncoders(const double *vals)
{
    CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, setpoints);
    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoder(int j, double *v)
{
    //CD_DEBUG("%d\n", j);  //-- Too verbose in stream.
    CHECK_JOINT(j);
    *v = getSetpoint(j);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoders(double *encs)
{
    //CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(setpoints, setpoints + Synapse::DATA_POINTS, encs);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoderSpeed(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (DextraControlboardUSB).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoderSpeeds(double *spds)
{
    //CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoderAcceleration(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (DextraControlboardUSB).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoderAccelerations(double *accs)
{
    //CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------
