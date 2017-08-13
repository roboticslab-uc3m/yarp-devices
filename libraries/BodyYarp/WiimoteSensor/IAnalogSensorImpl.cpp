// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cerrno>

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::read(yarp::sig::Vector &out)
{
    if (poll(fds, fds_num, -1) < 0 && errno != EINTR)
    {
        CD_ERROR("Cannot poll fds: %d.\n", -errno);
        return yarp::dev::IAnalogSensor::AS_ERROR;
    }

    int ret = xwii_iface_dispatch(iface, &event, sizeof(event));

    if (ret != 0)
    {
        if (ret != -EAGAIN)
        {
            CD_ERROR("Read failed with err: %d.\n", ret);
            return yarp::dev::IAnalogSensor::AS_ERROR;
        }
    }

    switch (event.type)
    {
    case XWII_EVENT_KEY:
        CD_INFO("Keypress event: code %d, state %d\n", event.v.key.code, event.v.key.state);
        break;
    case XWII_EVENT_ACCEL:
        CD_INFO("Accel event: [x] %d, [y] %d, [z] %d\n", event.v.abs[0].x, event.v.abs[0].y, event.v.abs[0].z);
        break;
    }

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::getChannels()
{
    return 4;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::calibrateSensor()
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::calibrateChannel(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::calibrateChannel(int ch, double value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------
