// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensorsClient.hpp"

roboticslab::IProximitySensors::alert_level roboticslab::ProximitySensorsClient::getAlertLevel()
{
    std::lock_guard<std::mutex> lock(alertMutex);
    return alert;
}

bool roboticslab::ProximitySensorsClient::hasTarget()
{
    std::lock_guard<std::mutex> lock(alertMutex);
    return gripper;
}
