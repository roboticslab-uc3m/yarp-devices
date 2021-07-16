// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LeapMotionSensor.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LeapMotionSensor::open(yarp::os::Searchable& config)
{
    controller = new Leap::Controller();

    Leap::Controller::PolicyFlag leapPolicies = Leap::Controller::POLICY_BACKGROUND_FRAMES;
    controller->setPolicy(leapPolicies);

    const int maxRetries = 10;
    int retries = 0;

    while (retries < maxRetries)
    {
        if (controller->isConnected() && controller->isPolicySet(leapPolicies))
        {
            break;
        }

        retries++;
        yCInfo(LEAP) << "Connection failed, retrying... " << retries;
        yarp::os::Time::delay(1);
    }

    if (retries == maxRetries)
    {
        yCError(LEAP) << "Unable to connect to Leap device, max retries exceeded";
        close();
        return false;
    }

    currentHandId = Leap::Hand::invalid().id();
    lastValidData.resize(6);

    yCInfo(LEAP) << "Leap Motion device started and running";

    return true;
}

// -----------------------------------------------------------------------------

bool LeapMotionSensor::close()
{
    if (controller)
    {
        delete controller;
        controller = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------
