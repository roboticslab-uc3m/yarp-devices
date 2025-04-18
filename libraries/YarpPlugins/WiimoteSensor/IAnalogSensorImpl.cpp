// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

// -----------------------------------------------------------------------------

int WiimoteSensor::read(yarp::sig::Vector &out)
{
    WiimoteEventData eventData = dispatcherThread.getEventData();

    double normX = ((double) eventData.accelX - m_calibZeroX) / (m_calibOneX - m_calibZeroX);
    double normY = ((double) eventData.accelY - m_calibZeroY) / (m_calibOneY - m_calibZeroY);
    double normZ = ((double) eventData.accelZ - m_calibZeroZ) / (m_calibOneZ - m_calibZeroZ);

    if (eventData.button1)
    {
        yawActive = false;
    }
    else if (eventData.button2)
    {
        yawActive = true;
    }

    // [roll, pitch, A, B, yawActive]
    out = {
        normX,
        normY,
        eventData.buttonA ? 1.0 : 0.0,
        eventData.buttonB ? 1.0 : 0.0,
        yawActive ? 1.0 : 0.0
    };

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int WiimoteSensor::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int WiimoteSensor::getChannels()
{
    return 8;
}

// -----------------------------------------------------------------------------

int WiimoteSensor::calibrateSensor()
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int WiimoteSensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int WiimoteSensor::calibrateChannel(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int WiimoteSensor::calibrateChannel(int ch, double value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------
