// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensors.hpp"

#include <ColorDebug.hpp>

bool roboticslab::ProximitySensors::hasObstacle()
{
    alertMutex.lock();
    bool alert_copy = alert;
    alertMutex.unlock();
    return alert_copy;
}

bool roboticslab::ProximitySensors::hasTarget()
{
    gripperMutex.lock();
    bool gripper_copy = gripper;
    gripperMutex.unlock();
    return gripper_copy;
}

