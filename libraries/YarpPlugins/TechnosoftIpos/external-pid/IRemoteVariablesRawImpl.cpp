// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
#else
bool TechnosoftIposExternal::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
#endif
{
    val.addString(key);

    if (key == "enableCsv")
    {
        val.addInt32(enableCsv ? 1 : 0);
    }
    else
    {
        yCIError(IPOS, id()) << "Unsupported key:" << key;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
#else
bool TechnosoftIposExternal::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
#endif
{
    if (key == "enableCsv")
    {
        auto requested = val.get(0).asBool();

        if (requested ^ enableCsv)
        {
            if (actualControlMode == VOCAB_CM_VELOCITY || requestedcontrolMode == VOCAB_CM_VELOCITY)
            {
                yCIError(IPOS, id()) << "Currently in vel mode, cannot change internal mode mapping right now";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
                return false;
#endif
            }

            enableCsv = requested;
            yCIInfo(IPOS, id()) << "CSV mode" << (requested ? "enabled" : "disabled");
        }
    }
    else
    {
        yCIError(IPOS, id()) << "Unsupported key:" << key;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
#else
bool TechnosoftIposExternal::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
#endif
{
    listOfKeys->clear();
    listOfKeys->addString("enableCsv");

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
