// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include "ColorDebug.h"

namespace
{
    bool yawActive = false;
}

// -----------------------------------------------------------------------------

int roboticslab::WiimoteSensor::read(yarp::sig::Vector &out)
{
    WiimoteEventData eventData = dispatcherThread.getEventData();

    double normX = ((double) eventData.accelX - calibZeroX) / (calibOneX - calibZeroX);
    double normY = ((double) eventData.accelY - calibZeroY) / (calibOneY - calibZeroY);
    double normZ = ((double) eventData.accelZ - calibZeroZ) / (calibOneZ - calibZeroZ);

    if (eventData.button1)
    {
        yawActive = false;
    }
    else if (eventData.button2)
    {
        yawActive = true;
    }

    out.resize(5);  //-- [roll, pitch, A, B, yawActive]

    out[0] = normX;
    out[1] = normY;
    out[2] = eventData.buttonA ? 1.0 : 0.0;
    out[3] = eventData.buttonB ? 1.0 : 0.0;
    out[4] = yawActive ? 1.0 : 0.0;

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
    return 5;
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
