// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNumberOfMotorEncoders(int * num)
{
    CD_DEBUG("\n");
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetMotorEncoder(int m)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);

    int localAxis;
    yarp::dev::IMotorEncodersRaw * p = deviceMapper.getDevice(m, &localAxis).iMotorEncodersRaw;
    return p ? p->resetMotorEncoderRaw(localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetMotorEncoders()
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IMotorEncodersRaw * p = rawDevices[i].iMotorEncodersRaw;
        ok &= p ? p->resetMotorEncodersRaw() : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoderCountsPerRevolution(int m, double cpr)
{
    CD_DEBUG("(%d, %f)\n", m, cpr);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, cpr, &yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderCountsPerRevolution(int m, double * cpr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, cpr, &yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoder(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoders(const double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(vals, &yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoder(int m, double * v)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, v, &yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoders(double * encs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(encs, &yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderTimed(int m, double * enc, double * stamp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);

    int localAxis;
    yarp::dev::IMotorEncodersRaw * p = deviceMapper.getDevice(m, &localAxis).iMotorEncodersRaw;
    return p ? p->getMotorEncoderTimedRaw(localAxis, enc, stamp) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncodersTimed(double * encs, double * stamps)
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IMotorEncodersRaw * p = rawDevices[i].iMotorEncodersRaw;
        ok &= p ? p->getMotorEncodersTimedRaw(encs + localAxisOffsets[i], stamps + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderSpeed(int m, double * sp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, sp, &yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderSpeeds(double *spds)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(spds, &yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderAcceleration(int m, double * acc)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, acc, &yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderAccelerations(double * accs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(accs, &yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw);
}

// -----------------------------------------------------------------------------
