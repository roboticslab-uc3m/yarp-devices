// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensors.hpp"


#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::ProximitySensors::open(yarp::os::Searchable& config)
{
    CD_INFO("Starting ProximitySensors plugin.\n");
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    sr.setReference(this);
    sr.open("/sensor_reader");
    sr.useCallback();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::ProximitySensors::close()
{
    CD_INFO("Closing ProximitySensors plugin.\n");
    sr.disableCallback();
    sr.close();
    return true;
}

// -----------------------------------------------------------------------------

