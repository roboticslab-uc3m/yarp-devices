// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <algorithm>
#include <vector>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    if (key == "linInterpPeriodMs")
    {
        val.addInt32(posdThread->getPeriod());
    }
    else
    {
        CD_ERROR("Unsupported key: %s.\n", key.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (key != "linInterpPeriodMs" && key != "linInterpBufferSize" && key != "linInterpMode")
    {
        CD_ERROR("Unsupported key: %s.\n", key.c_str());
        return false;
    }

    std::vector<int> modes(nodeDevices.size());

    if (!getControlModes(modes.data()))
    {
        CD_ERROR("Unable to retrieve control modes.\n");
        return false;
    }

    if (std::any_of(modes.begin(), modes.end(), [](int mode) { return mode == VOCAB_CM_POSITION_DIRECT; }))
    {
        CD_ERROR("CAN device currently in posd mode, cannot change config params right now.\n");
        return false;
    }

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();
        p->setRemoteVariableRaw(key, val);
    }

    if (key == "linInterpPeriodMs")
    {
        posdThread->setPeriod(val.get(0).asInt32() * 0.001);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle * listOfKeys)
{
    CD_DEBUG("\n");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterpPeriodMs");
    listOfKeys->addString("linInterpBufferSize");
    listOfKeys->addString("linInterpMode");

    return true;
}

// -----------------------------------------------------------------------------
