// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LeapMotionSensor.hpp"

#include <cmath>

#include <yarp/sig/Vector.h>

#include "ColorDebug.hpp"

namespace
{
    int32_t currentHandId = Leap::Hand::invalid().id();
    yarp::sig::Vector lastValidData(6);
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::read(yarp::sig::Vector &out)
{
    Leap::Frame frame = controller->frame();
    Leap::Hand hand = controller->frame().hand(currentHandId);

    if (currentHandId == Leap::Hand::invalid().id())
    {
        CD_INFO("Invalid id.\n");
        hand = frame.hands()[0];
        currentHandId = hand.id();
    }
    else
    {
        CD_INFO("Picking hand with id: %d.\n", currentHandId);
        hand = frame.hand(currentHandId);
    }

    if (!hand.isValid())
    {
        out = lastValidData;
        currentHandId = Leap::Hand::invalid().id();
        return yarp::dev::IAnalogSensor::AS_OK;
    }

    Leap::Vector position = hand.palmPosition();
    Leap::Vector direction = hand.direction();
    Leap::Vector normal = hand.palmNormal();

    out.resize(6);

    // send translation coordinates in cm
    out[0] = -position.z * 0.1;
    out[1] = -position.x * 0.1;
    out[2] = position.y * 0.1;

    // ...and rotations in degrees
    out[3] = -normal.roll() * 180.0 / M_PI;
    out[4] = -direction.pitch() * 180.0 / M_PI;
    out[5] = -direction.yaw() * 180.0 / M_PI;

    lastValidData = out;

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::getChannels()
{
    return 6;
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::calibrateSensor()
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::calibrateChannel(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::LeapMotionSensor::calibrateChannel(int ch, double value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------
