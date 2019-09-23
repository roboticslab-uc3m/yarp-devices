// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAxes(int *axes)
{
    CD_DEBUG("\n");
    *axes = deviceMapper.getControlledAxes();
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetEncoder(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IEncodersTimedRaw * p = deviceMapper.getDevice(j, &localAxis).iEncodersTimedRaw;
    return p ? p->resetEncoderRaw(localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetEncoders()
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IEncodersTimedRaw * p = rawDevices[i].iEncodersTimedRaw;
        ok &= p ? p->resetEncodersRaw() : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setEncoder(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, val, &yarp::dev::IEncodersRaw::setEncoderRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setEncoders(const double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(vals, &yarp::dev::IEncodersRaw::setEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoder(int j, double * v)
{
    //CD_DEBUG("%d\n", j); //-- Too verbose in stream.
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, v, &yarp::dev::IEncodersRaw::getEncoderRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoders(double * encs)
{
    //CD_DEBUG("\n"); //-- Too verbose in stream.
    return deviceMapper.fullJointMapping(encs, &yarp::dev::IEncodersRaw::getEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderSpeed(int j, double * sp)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, sp, &yarp::dev::IEncodersRaw::getEncoderSpeedRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderSpeeds(double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(spds, &yarp::dev::IEncodersRaw::getEncoderSpeedsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderAcceleration(int j, double * spds)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, spds, &yarp::dev::IEncodersRaw::getEncoderAccelerationRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderAccelerations(double * accs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(accs, &yarp::dev::IEncodersRaw::getEncoderAccelerationsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderTimed(int j, double * enc, double * stamp)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IEncodersTimedRaw * p = deviceMapper.getDevice(j, &localAxis).iEncodersTimedRaw;
    return p ? p->getEncoderTimedRaw(localAxis, enc, stamp) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncodersTimed(double * encs, double * stamps)
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IEncodersTimedRaw * p = rawDevices[i].iEncodersTimedRaw;
        ok &= p ? p->getEncodersTimedRaw(encs + localAxisOffsets[i], stamps + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------
