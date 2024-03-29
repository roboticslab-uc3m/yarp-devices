// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LeapMotionSensor.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

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

    out.resize(getChannels());

    // https://developer.leapmotion.com/documentation/v2/cpp/api/Leap.Hand.html
    // https://developer.leapmotion.com/documentation/v2/cpp/api/Leap.Vector.html

    // send translation coordinates in mm
    out[0] = position.x;
    out[1] = position.y;
    out[2] = position.z;

    // ...and rotations in radians (assume hand fingers pointing at -Z)
    out[3] = direction.pitch();
    out[4] = -direction.yaw();
    out[5] = normal.roll();

    out[6] = hand.grabStrength() > 0.5 ? 1.0 : 0.0;
    out[7] = hand.pinchStrength() > 0.5 ? 1.0 : 0.0;

    lastValidData = out;

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
    return 8;
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
