// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

// ------------------- DeviceDriver related ------------------------------------

bool roboticslab::AmorControlboard::open(yarp::os::Searchable& config)
{
    yDebug() << "AmorControlboard config:" << config.toString();

    int major, minor, build;
    amor_get_library_version(&major, &minor, &build);

    yInfo("AMOR API library version %d.%d.%d", major, minor, build);
    yInfo() << "Trying to connect to AMOR...";

    handle = amor_connect((char *)DEFAULT_CAN_LIBRARY, DEFAULT_CAN_PORT);

    if (handle == AMOR_INVALID_HANDLE)
    {
        yError() << "Could not get AMOR handle:" << amor_error();
        return false;
    }

    yInfo() << "Acquired AMOR handle!";

    AMOR_JOINT_INFO jointInfo[AMOR_NUM_JOINTS];
    int jointStatus[AMOR_NUM_JOINTS];

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        if (amor_get_joint_info(handle, j, &jointInfo[j]) != AMOR_SUCCESS)
        {
            yError() << "amor_get_joint_info() failed for joint" << j << "with error:" << amor_error();
            amor_release(handle);
            return false;
        }

        if (amor_get_status(handle, j, &jointStatus[j]) != AMOR_SUCCESS)
        {
            yError() << "amor_get_status() failed for joint" << j << "with error:" << amor_error();
            amor_release(handle);
            return false;
        }
    }

    std::vector<double> positions;

    if (!getEncoders(positions.data()))
    {
        yError() << "getEncoders() failed";
        amor_release(handle);
        return false;
    }

    yInfo() << "Current positions (degrees):" << positions;

    // Set position mode.
    if (!positionMove(positions.data()))
    {
        yError() << "positionMove() failed";
        amor_release(handle);
        return false;
    }

    yarp::os::Value * cartesianControllerName;

    if (config.check("cartesianControllerName", cartesianControllerName, "cartesian controller port"))
    {
        yInfo() << "Using AMOR cartesian controller device";

        usingCartesianController = true;

        std::string subdevice = "AmorCartesianControl";
        yarp::os::Value vHandle(&handle, sizeof handle);
        yarp::os::Property cartesianControllerOptions;

        cartesianControllerOptions.fromString((config.toString()));
        cartesianControllerOptions.put("device", "CartesianControlServer");
        cartesianControllerOptions.put("subdevice", subdevice);
        cartesianControllerOptions.put("name", cartesianControllerName->asString());
        cartesianControllerOptions.put("handle", vHandle);
        cartesianControllerOptions.setMonitor(config.getMonitor(), subdevice.c_str());

        cartesianControllerDevice.open(cartesianControllerOptions);

        if (!cartesianControllerDevice.isValid())
        {
            yError() << "AMOR cartesian controller device not valid";
            amor_release(handle);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::close()
{
    if (usingCartesianController)
    {
        cartesianControllerDevice.close();
    }

    if (handle != AMOR_INVALID_HANDLE)
    {
        amor_emergency_stop(handle);
        amor_release(handle);

        handle = AMOR_INVALID_HANDLE;
    }

    return true;
}

// -----------------------------------------------------------------------------
