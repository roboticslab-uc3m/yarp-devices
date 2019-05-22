// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <string>
#include <ColorDebug.h>

// ---------------------------- IRemoteVariables Related ----------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (val.size() > nodes.size())
    {
        CD_ERROR("Bottle size exceeds number of nodes: %zd > %zd\n", val.size(), nodes.size());
        return false;
    }
    else if (val.size() != nodes.size())
    {
        CD_WARNING("Bottle size does not match number of nodes: %zd != %zd\n", val.size(), nodes.size());
    }

    bool ok = true;

    val.clear();

    for (int i = 0; i < val.size(); i++)
    {
        yarp::os::Bottle b;

        if (iRemoteVariablesRaw[i]->getRemoteVariableRaw(key, b))
        {
            val.addList() = b;
        }
        else
        {
            val.addList();
            ok = false;
        }
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (key == "ptModeMs")
    {
        for (unsigned int i = 0; i < motorIds.size(); i++)
        {
            if (!iRemoteVariablesRaw[motorIds[i]]->setRemoteVariableRaw(key, val))
            {
                CD_WARNING("Unable to set remote variable on node %d.\n", motorIds[i]);
                return false;
            }
        }
    }
    else if (key == "pvtPoints")
    {
        yarp::os::Bottle pvtPointsPerJoint;

        for (unsigned int i = 0; i < motorIds.size(); i++)
        {
            pvtPointsPerJoint.addList();
        }

        for (int i = 0; i < val.size(); i++)
        {
            yarp::os::Bottle * pvtPoint = val.get(i).asList();

            if (pvtPoint == 0)
            {
                CD_WARNING("Not a list: %s.\n", val.get(i).toString().c_str());
                return false;
            }

            double time = pvtPoint->find("t").asFloat64();

            if (time == 0.0)
            {
                CD_WARNING("Time (\"p\") property missing or equal to zero: %s.\n", pvtPoint->toString().c_str());
                return false;
            }

            yarp::os::Bottle * positions = pvtPoint->find("p").asList();

            if (positions == 0)
            {
                CD_WARNING("Position (\"p\") property missing or not a list: %s.\n", pvtPoint->toString().c_str());
                return false;
            }
            else if (positions->size() != motorIds.size())
            {
                CD_WARNING("Position (\"p\") property size does not match number of nodes: %zd != %zd.\n", positions->size(), motorIds.size());
                return false;
            }

            yarp::os::Bottle * velocities = pvtPoint->find("v").asList();

            if (velocities != 0 && velocities->size() != motorIds.size())
            {
                CD_WARNING("Velocities (\"v\") property size does not match number of nodes: %zd != %zd.\n", velocities->size(), motorIds.size());
                return false;
            }

            for (unsigned int i = 0; i < motorIds.size(); i++)
            {
                yarp::os::Bottle & pvt = pvtPointsPerJoint.get(i).asList()->addList();
                pvt.addInt32(time);
                pvt.addFloat64(positions->get(i).asFloat64());

                if (velocities != 0)
                {
                    pvt.addFloat64(velocities->get(i).asFloat64());
                }
            }
        }

        for (unsigned int i = 0; i < motorIds.size(); i++)
        {
            if (!iRemoteVariablesRaw[motorIds[i]]->setRemoteVariableRaw(key, *pvtPointsPerJoint.get(i).asList()))
            {
                CD_WARNING("Unable to set remote variable on node %d.\n", motorIds[i]);
                return false; // TODO: clear internally stored PVT points
            }
        }
    }
    else
    {
        CD_WARNING("Unrecognized key: %s.\n", key.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("ptModeMs");
    listOfKeys->addString("pvtPoints");
    return true;
}

// -----------------------------------------------------------------------------
