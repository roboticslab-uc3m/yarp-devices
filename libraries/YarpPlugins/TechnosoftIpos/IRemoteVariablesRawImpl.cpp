// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IRemoteVariablesRaw Related ----------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    if (key == "ptModeMs")
    {
        val.addInt32(pvtModeMs);
    }
    else if (key == "pvtPoints")
    {
        /*for (unsigned int i = 0; i < pvtQueue.size(); i++)
        {
            val.addList() = pvtQueue[i].toBottle();
        }*/
    }
    else
    {
        CD_WARNING("Unrecognized key: %s.\n", key.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    bool ret = true;

    if (key == "ptModeMs")
    {
        if (pvtModeMs >= 0)
        {
            CD_WARNING("Illegal state, unable to change variable when currently used.\n");
            return false;
        }

        pvtModeMs = val.get(0).asInt32();

        if (pvtModeMs < 0)
        {
            CD_WARNING("Invalid %s: %d.\n", key.c_str(), pvtModeMs);
            return false;
        }
    }
    else if (key == "pvtPoints")
    {
        for (int i = 0; i < val.size(); i++)
        {
            yarp::os::Bottle * pvtBottle = val.get(i).asList();

            if (pvtBottle == 0 || pvtBottle->size() != 2 || pvtBottle->size() != 3)
            {
                CD_WARNING("Illegal value: malformed bottle or wrong size: %s.\n", val.get(i).toString().c_str());
                ret = false;
            }

            //pvtQueue.push_back(PvtPoint::fromBottle(*pvtBottle, pvtBottle->size() == 3));
        }

        /*if (!pvtQueue.empty() && !fillPvtBuffer(PVT_BUFFER_MAX_SIZE))
        {
            CD_ERROR("Unable to send PVT messages.\n");
            return false;
        }*/
    }
    else
    {
        CD_WARNING("Unrecognized key: %s.\n", key.c_str());
        return false;
    }

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("ptModeMs");
    listOfKeys->addString("pvtPoints");
    return true;
}

// -----------------------------------------------------------------------------
