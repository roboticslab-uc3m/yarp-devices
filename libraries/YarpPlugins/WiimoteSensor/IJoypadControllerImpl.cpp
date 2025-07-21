// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool WiimoteSensor::getAxisCount(unsigned int & axis_count)
{
    axis_count = 3;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getButtonCount(unsigned int & button_count)
{
    button_count = 4;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getTrackballCount(unsigned int & trackball_count)
{
    trackball_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getHatCount(unsigned int & hat_count)
{
    hat_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getTouchSurfaceCount(unsigned int & touch_count)
{
    touch_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getStickCount(unsigned int & stick_count)
{
    stick_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getStickDoF(unsigned int stick_id, unsigned int & DoF)
{
    DoF = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getButton(unsigned int button_id, float & value)
{
    if (button_id >= 4)
    {
        yCError(WII) << "Invalid button ID:" << button_id;
        return false;
    }

    const auto eventData = dispatcherThread.getEventData();

    switch (button_id)
    {
    case 0:
        value = eventData.buttonA ? 1.0f : 0.0f;
    case 1:
        value = eventData.buttonB ? 1.0f : 0.0f;
    case 2:
        value = eventData.button1 ? 1.0f : 0.0f;
    case 3:
        value = eventData.button2 ? 1.0f : 0.0f;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getTrackball(unsigned int trackball_id, yarp::sig::Vector & value)
{
    return false;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getHat(unsigned int hat_id, unsigned char & value)
{
    return false;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getAxis(unsigned int axis_id, double & value)
{
    if (axis_id >= 3)
    {
        yCError(WII) << "Invalid axis ID:" << axis_id;
        return false;
    }

    const auto eventData = dispatcherThread.getEventData();

    const int * accel;
    const int * calibOne;
    const int * calibZero;

    switch (axis_id)
    {
    case 0: // X-axis
        accel = &eventData.accelX;
        calibOne = &m_calibOneX;
        calibZero = &m_calibZeroX;
        break;
    case 1: // Y-axis
        accel = &eventData.accelY;
        calibOne = &m_calibOneY;
        calibZero = &m_calibZeroY;
        break;
    case 2: // Z-axis
        accel = &eventData.accelZ;
        calibOne = &m_calibOneZ;
        calibZero = &m_calibZeroZ;
        break;
    }

    value = static_cast<double>(*accel - *calibZero) / static_cast<double>(*calibOne - *calibZero);
    return true;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getStick(unsigned int stick_id, yarp::sig::Vector & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
{
    return false;
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::getTouch(unsigned int touch_id, yarp::sig::Vector & value)
{
    return false;
}

// -----------------------------------------------------------------------------
