// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::open(yarp::os::Searchable & config)
{
    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt8();
    axisName = config.check("name", yarp::os::Value(""), "axis name").asString();
    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));

    if (config.check("forceTorqueSensor", "embedded force-torque sensor"))
    {
        yCIInfo(LCQ, id()) << "Using embedded force-torque sensor device";

        auto sensorName = config.find("forceTorqueSensor").asString();
        const auto & sensorGroup = robotConfig->findGroup(sensorName);

        if (sensorGroup.isNull())
        {
            yCIError(LCQ, id()) << "Missing force-torque sensor device group" << sensorName;
            return false;
        }

        yarp::os::Property sensorOptions;
        sensorOptions.fromString(sensorGroup.toString());
        sensorOptions.put("robotConfig", config.find("robotConfig"));
        sensorOptions.setMonitor(config.getMonitor(), sensorName.c_str());

        if (!sensorDevice.open(sensorOptions))
        {
            yCIError(LCQ, id()) << "Unable to open force-torque sensor device";
            return false;
        }

        if (!sensorDevice.view(sensor))
        {
            yCIError(LCQ, id()) << "Unable to view interfaces in force-torque sensor device";
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::close()
{
    if (sensorDevice.isValid())
    {
        return sensorDevice.close();
    }

    return true;
}

// -----------------------------------------------------------------------------
