// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

#include <sys/ioctl.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

int Jr3::read(yarp::sig::Vector &out)
{
    bool ok = true;
    yarp::sig::Vector out0, out1, out2, out3;
    double timestamp;

    ok = ok & getSixAxisForceTorqueSensorMeasure(0, out0, timestamp);
    ok = ok & getSixAxisForceTorqueSensorMeasure(1, out1, timestamp);
    ok = ok & getSixAxisForceTorqueSensorMeasure(2, out2, timestamp);
    ok = ok & getSixAxisForceTorqueSensorMeasure(3, out3, timestamp);

    if (!ok)
    {
        return yarp::dev::IAnalogSensor::AS_ERROR;
    }

    out.resize(getChannels());

    out.setSubvector(0, out0);
    out.setSubvector(6, out1);
    out.setSubvector(12, out2);
    out.setSubvector(18, out3);

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int Jr3::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int Jr3::getChannels()
{
    return getNrOfSixAxisForceTorqueSensors() * 6;
}

// -----------------------------------------------------------------------------

int Jr3::calibrateSensor()
{
    constexpr auto ok = yarp::dev::IAnalogSensor::AS_OK;

    auto ok0 = calibrateChannel(0);
    auto ok1 = calibrateChannel(1);
    auto ok2 = calibrateChannel(2);
    auto ok3 = calibrateChannel(3);

    if (ok0 != ok || ok1 != ok || ok2 != ok || ok3 != ok)
    {
        return yarp::dev::IAnalogSensor::AS_ERROR;
    }

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int Jr3::calibrateSensor(const yarp::sig::Vector& value)
{
    yCError(JR3) << "calibrateSensor(const yarp::sig::Vector& value) not implemented";
    return yarp::dev::IAnalogSensor::AS_ERROR;
}

// -----------------------------------------------------------------------------

int Jr3::calibrateChannel(int ch)
{
    int ok;

    switch(ch)
    {
    case 0:
        ok = ::ioctl(fd, IOCTL0_JR3_ZEROOFFS);
        break;
    case 1:
        ok = ::ioctl(fd, IOCTL1_JR3_ZEROOFFS);
        break;
    case 2:
        ok = ::ioctl(fd, IOCTL2_JR3_ZEROOFFS);
        break;
    case 3:
        ok = ::ioctl(fd, IOCTL3_JR3_ZEROOFFS);
        break;
    default:
        yCError(JR3) << "Illegal channel" << ch;
        ok = -1;
        break;
    }

    if (ok == -1)
    {
        yCError(JR3) << "ioctl() on calibrate channel" << ch << "failed";
        return yarp::dev::IAnalogSensor::AS_ERROR;
    }

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int Jr3::calibrateChannel(int ch, double value)
{
    yCError(JR3) << "calibrateChannel(int ch, double value) not implemented";
    return yarp::dev::IAnalogSensor::AS_ERROR;
}

// -----------------------------------------------------------------------------
