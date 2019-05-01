// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IRemoteVariablesRaw Related ----------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    if (key == "pvtPoints")
    {
        for (unsigned int i = 0; i < pvtQueue.size(); i++)
        {
            val.addList() = pvtQueue[i].toBottle();
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

bool roboticslab::TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    bool ret = true;

    if (key == "pvtPoints")
    {
        for (int i = 0; i < val.size(); i++)
        {
            yarp::os::Bottle * pvtBottle = val.get(i).asList();

            if (pvtBottle->isNull() || pvtBottle->size() != 3)
            {
                CD_WARNING("Illegal value: malformed bottle or wrong size: %s.\n", val.get(i).toString().c_str());
                ret = false;
            }

            pvtQueue.push_back(PvtPoint::fromBottle(*pvtBottle));
        }
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
    listOfKeys->addString("pvtPoints");
    return true;
}

// -----------------------------------------------------------------------------
