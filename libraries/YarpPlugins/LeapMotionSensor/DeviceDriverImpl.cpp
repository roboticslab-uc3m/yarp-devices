// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LeapMotionSensor.hpp"

#include <yarp/os/Time.h>

#include "ColorDebug.h"

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
        CD_INFO("Connection failed, retrying... %d\n", retries);
        yarp::os::Time::delay(1);
    }

    if (retries == maxRetries)
    {
        CD_ERROR("Unable to connect to Leap device, max retries exceeded.\n");
        close();
        return false;
    }

    CD_SUCCESS("Leap Motion device started and running.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LeapMotionSensor::close()
{
    delete controller;
    controller = NULL;
    return true;
}

// -----------------------------------------------------------------------------
