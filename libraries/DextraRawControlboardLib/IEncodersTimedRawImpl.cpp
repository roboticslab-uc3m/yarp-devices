// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <algorithm>

#include <yarp/os/Time.h>

#include <ColorDebug.h>
#include "../DextraRawControlboardLib/DextraRawControlboard.hpp"

// ------------------ IEncoders Related -----------------------------------------

bool roboticslab::DextraRawControlboard::resetEncoderRaw(int j)
{
    CD_DEBUG("(%d)\n",j);
    return setEncoderRaw(j, 0.0);
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::resetEncodersRaw()
{
    CD_DEBUG("\n");
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setEncoderRaw(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    setSetpoint(j, val);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setEncodersRaw(const double *vals)
{
    CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, setpoints);
    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncoderRaw(int j, double *v)
{
    //CD_DEBUG("%d\n", j);  //-- Too verbose in stream.
    CHECK_JOINT(j);
    *v = getSetpoint(j);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncodersRaw(double *encs)
{
    //CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(setpoints, setpoints + Synapse::DATA_POINTS, encs);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_DEBUG("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncoderSpeedsRaw(double *spds)
{
    //CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderSpeedRaw(j, &spds[j]);
    }

    return ok;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncoderAccelerationRaw(int j, double *accs)
{
    //CD_DEBUG("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncoderAccelerationsRaw(double *accs)
{
    //CD_DEBUG("\n");
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderAccelerationRaw(j, &accs[j]);
    }

    return ok;
}

// ------------------ IEncodersTimed Related -----------------------------------------

bool roboticslab::DextraRawControlboard::getEncodersTimedRaw(double *encs, double *time)
{
    CD_DEBUG("\n");
    *time = yarp::os::Time::now();
    return getEncodersRaw(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getEncoderTimedRaw(int j, double *enc, double *time)
{
    //CD_DEBUG("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *time = yarp::os::Time::now();
    return getEncoderRaw(j, enc);
}

// -----------------------------------------------------------------------------
