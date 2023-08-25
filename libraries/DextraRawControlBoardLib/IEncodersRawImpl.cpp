// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <algorithm>

#include <yarp/os/Log.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::resetEncoderRaw(int j)
{
    yCITrace(DEXTRA, id(), "%d", j);
    return setEncoderRaw(j, 0.0);
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::resetEncodersRaw()
{
    yCITrace(DEXTRA, id(), "");
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::setEncoderRaw(int j, double val)
{
    yCITrace(DEXTRA, id(), "%d %f", j, val);
    CHECK_JOINT(j);
    setSetpoint(j, val);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::setEncodersRaw(const double * vals)
{
    yCITrace(DEXTRA, id(), "");
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, std::begin(setpoints));
    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getEncoderRaw(int j, double * v)
{
    yCITrace(DEXTRA, id(), "%d", j);
    CHECK_JOINT(j);
    *v = getSetpoint(j);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::getEncodersRaw(double *encs)
{
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(std::cbegin(setpoints), std::cend(setpoints), encs);
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getEncoderSpeedRaw(int j, double * sp)
{
    CHECK_JOINT(j);
    return false;
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::getEncoderSpeedsRaw(double * spds)
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderSpeedRaw(j, &spds[j]);
    }

    return ok;
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::getEncoderAccelerationRaw(int j, double * accs)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getEncoderAccelerationsRaw(double * accs)
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderAccelerationRaw(j, &accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getEncoderTimedRaw(int j, double * enc, double * time)
{
    CHECK_JOINT(j);
    *time = yarp::os::Time::now();
    return getEncoderRaw(j, enc);
}

// ------------------------------------------------------------------------------

bool DextraRawControlBoard::getEncodersTimedRaw(double * encs, double * time)
{
    *time = yarp::os::Time::now();
    return getEncodersRaw(encs);
}

// -----------------------------------------------------------------------------
