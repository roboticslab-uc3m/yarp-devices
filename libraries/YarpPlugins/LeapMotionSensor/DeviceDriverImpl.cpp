// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LeapMotionSensor.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

// -----------------------------------------------------------------------------

bool roboticslab::LeapMotionSensor::open(yarp::os::Searchable& config)
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
        yInfo() << "Connection failed, retrying... " << retries;
        yarp::os::Time::delay(1);
    }

    if (retries == maxRetries)
    {
        yError() << "Unable to connect to Leap device, max retries exceeded";
        close();
        return false;
    }

    yInfo() << "Leap Motion device started and running";

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LeapMotionSensor::close()
{
    if (controller)
    {
        delete controller;
        controller = NULL;
    }

    return true;
}

// -----------------------------------------------------------------------------
