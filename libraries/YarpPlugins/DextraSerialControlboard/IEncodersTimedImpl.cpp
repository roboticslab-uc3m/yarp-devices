// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <algorithm>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// ------------------ IEncoders Related -----------------------------------------

bool roboticslab::DextraSerialControlboard::resetEncoder(int j)
{
    CD_DEBUG("(%d)\n",j);
    return setEncoder(j, 0.0);
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::resetEncoders()
{
    CD_DEBUG("\n");
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setEncoder(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    setSetpoint(j, val);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setEncoders(const double *vals)
{
    CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, setpoints);
    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoder(int j, double *v)
{
    //CD_DEBUG("%d\n", j);  //-- Too verbose in stream.
    CHECK_JOINT(j);
    *v = getSetpoint(j);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoders(double *encs)
{
    //CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(setpoints, setpoints + Synapse::DATA_POINTS, encs);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoderSpeed(int j, double *sp)
{
    //CD_DEBUG("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoderSpeeds(double *spds)
{
    //CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderSpeed(j, &spds[j]);
    }

    return ok;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoderAcceleration(int j, double *accs)
{
    //CD_DEBUG("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoderAccelerations(double *accs)
{
    //CD_DEBUG("\n");
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderAcceleration(j, &accs[j]);
    }

    return ok;
}

// ------------------ IEncodersTimed Related -----------------------------------------

bool roboticslab::DextraSerialControlboard::getEncodersTimed(double *encs, double *time)
{
    CD_DEBUG("\n");
    *time = yarp::os::Time::now();
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getEncoderTimed(int j, double *enc, double *time)
{
    //CD_DEBUG("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *time = yarp::os::Time::now();
    return getEncoder(j, enc);
}

// -----------------------------------------------------------------------------
