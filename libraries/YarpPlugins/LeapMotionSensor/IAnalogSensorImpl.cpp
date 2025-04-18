// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LeapMotionSensor.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

int LeapMotionSensor::read(yarp::sig::Vector &out)
{
    Leap::Frame frame = controller->frame();
    Leap::Hand hand = controller->frame().hand(currentHandId);

    if (currentHandId == Leap::Hand::invalid().id())
    {
        yCInfo(LEAP) << "Invalid hand id";
        hand = frame.hands()[0];
        currentHandId = hand.id();
    }
    else
    {
        yCInfo(LEAP) << "Picking hand with id:" << currentHandId;
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

    // https://developer.leapmotion.com/documentation/v2/cpp/api/Leap.Hand.html
    // https://developer.leapmotion.com/documentation/v2/cpp/api/Leap.Vector.html

    lastValidData = out = {
        // send translation coordinates in mm
        position.x,
        position.y,
        position.z,
        // ...and rotations in radians (assume hand fingers pointing at -Z)
        direction.pitch(),
        -direction.yaw(),
        normal.roll(),
        // gestures
        hand.grabStrength() > 0.5 ? 1.0 : 0.0,
        hand.pinchStrength() > 0.5 ? 1.0 : 0.0
    };

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int LeapMotionSensor::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int LeapMotionSensor::getChannels()
{
    return NUM_CHANNELS;
}

// -----------------------------------------------------------------------------

int LeapMotionSensor::calibrateSensor()
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int LeapMotionSensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int LeapMotionSensor::calibrateChannel(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int LeapMotionSensor::calibrateChannel(int ch, double value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------
