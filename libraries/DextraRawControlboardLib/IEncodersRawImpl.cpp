// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <algorithm>

#include <yarp/conf/version.h>

#include <yarp/os/Log.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------------------------------------------------------------------

bool DextraRawControlboard::resetEncoderRaw(int j)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(DEXTRA, id(), "%d", j);
#else
    yCTrace(DEXTRA, "%d", j);
#endif
    return setEncoderRaw(j, 0.0);
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::resetEncodersRaw()
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(DEXTRA, id(), "");
#else
    yCTrace(DEXTRA, "");
#endif
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::setEncoderRaw(int j, double val)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(DEXTRA, id(), "%d %f", j, val);
#else
    yCTrace(DEXTRA, "%d %f", j, val);
#endif
    CHECK_JOINT(j);
    setSetpoint(j, val);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::setEncodersRaw(const double * vals)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(DEXTRA, id(), "");
#else
    yCTrace(DEXTRA, "");
#endif
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, std::begin(setpoints));
    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderRaw(int j, double * v)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(DEXTRA, id(), "%d", j);
#else
    yCTrace(DEXTRA, "%d", j);
#endif
    CHECK_JOINT(j);
    *v = getSetpoint(j);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncodersRaw(double *encs)
{
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(std::cbegin(setpoints), std::cend(setpoints), encs);
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderSpeedRaw(int j, double * sp)
{
    CHECK_JOINT(j);
    return false;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderSpeedsRaw(double * spds)
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderSpeedRaw(j, &spds[j]);
    }

    return ok;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderAccelerationRaw(int j, double * accs)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderAccelerationsRaw(double * accs)
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderAccelerationRaw(j, &accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderTimedRaw(int j, double * enc, double * time)
{
    CHECK_JOINT(j);
    *time = yarp::os::Time::now();
    return getEncoderRaw(j, enc);
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncodersTimedRaw(double * encs, double * time)
{
    *time = yarp::os::Time::now();
    return getEncodersRaw(encs);
}

// -----------------------------------------------------------------------------
