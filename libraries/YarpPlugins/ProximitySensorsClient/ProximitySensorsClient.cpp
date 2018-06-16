// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensorsClient.hpp"

#include <ColorDebug.h>

void roboticslab::ProximitySensorsClient::SensorReader::onRead(yarp::os::Bottle& b)
{
    if (b.size() == 0)
    {
        return;
    }

    sens->gripperMutex.lock();

    if (b.get(14).asDouble() > sens->thresholdGripper && b.get(14).asDouble() < 1000)
    {
        sens->gripper=true;
        CD_INFO("Target detected.\n");
    }
    else
    {
       sens->gripper = false;
    }

    sens->gripperMutex.unlock();

    double max = 0;

    for (int i = 0; i < b.size(); i++)
    {
        if (b.get(i).asDouble() > max)
        {
            max = b.get(i).asDouble();
        }
    }

    CD_INFO("Current maximum sensor value: %f.\n", max);

    sens->alertMutex.lock();

    if (max > sens->thresholdAlertHigh)
    {
        sens->alert = HIGH;
    }
    else if (max > sens->thresholdAlertLow)
    {
        sens->alert = LOW;
    }
    else
    {
        sens->alert = ZERO;
    }

    sens->alertMutex.unlock();
}
