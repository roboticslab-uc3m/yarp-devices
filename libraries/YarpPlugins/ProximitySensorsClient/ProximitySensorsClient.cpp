// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensorsClient.hpp"

#include <yarp/os/LogStream.h>

void roboticslab::ProximitySensorsClient::SensorReader::onRead(yarp::os::Bottle& b)
{
    if (b.size() == 0)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sens->gripperMutex);

        if (b.get(14).asFloat64() > sens->thresholdGripper && b.get(14).asFloat64() < 1000)
        {
            sens->gripper=true;
            yInfo() << "Target detected";
        }
        else
        {
           sens->gripper = false;
        }
    }

    double max = 0;

    for (int i = 0; i < b.size(); i++)
    {
        if (b.get(i).asFloat64() > max)
        {
            max = b.get(i).asFloat64();
        }
    }

    yInfo() << "Current maximum sensor value:" << max;

    {
        std::lock_guard<std::mutex> lock(sens->alertMutex);

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
    }
}
