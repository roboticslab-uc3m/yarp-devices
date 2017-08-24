// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensors.hpp"

#include <string>
#include <sstream>

#include <yarp/os/Value.h>

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::ProximitySensors::open(yarp::os::Searchable& config)
{
    CD_INFO("Starting ProximitySensors plugin.\n");
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    std::string local = config.check("local", yarp::os::Value(DEFAULT_LOCAL), "local port").asString();
    std::string remote = config.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();

    std::string pmType = config.check("portMonitorType", yarp::os::Value(DEFAULT_PORTMONITOR_TYPE), "port monitor type").asString();
    std::string pmContext = config.check("portMonitorContext", yarp::os::Value(DEFAULT_PORTMONITOR_CONTEXT), "port monitor context").asString();
    std::string pmFile = config.check("portMonitorFile", yarp::os::Value(DEFAULT_PORTMONITOR_FILE), "port monitor file").asString();

    sr.setReference(this);
    sr.open(local);
    sr.useCallback();

    std::ostringstream carrier;

    carrier << "tcp+recv.portmonitor+type." << pmType << "+context." << pmContext << "+file." << pmFile;

    CD_INFO("Using carrier: %s\n", carrier.str().c_str());

    yarp::os::Network::connect(remote, local, carrier.str());

    if (sr.getInputCount() == 0)
    {
        CD_ERROR("Unable to connect to remote port \"%s\".\n", remote.c_str());
        close();
        return false;
    }

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

