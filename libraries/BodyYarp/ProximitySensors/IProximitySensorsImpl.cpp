// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensorsClient.hpp"

#include <ColorDebug.hpp>

roboticslab::IProximitySensors::alert_level roboticslab::ProximitySensorsClient::getAlertLevel()
{
    alertMutex.lock();
    alert_level alert_copy = alert;
    alertMutex.unlock();
    return alert_copy;
}

bool roboticslab::ProximitySensorsClient::hasTarget()
{
    gripperMutex.lock();
    bool gripper_copy = gripper;
    gripperMutex.unlock();
    return gripper_copy;
}

