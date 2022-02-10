// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <vector>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_CAN_LIBRARY = "libeddriver.so";
constexpr auto DEFAULT_CAN_PORT = 0;

// ------------------- DeviceDriver related ------------------------------------

bool AmorControlboard::open(yarp::os::Searchable& config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(AMOR) << "Config:" << config.toString();
#endif

    int major, minor, build;
    amor_get_library_version(&major, &minor, &build);

    yCInfo(AMOR, "AMOR API library version %d.%d.%d", major, minor, build);
    yCInfo(AMOR) << "Trying to connect to AMOR...";

    handle = amor_connect((char *)DEFAULT_CAN_LIBRARY, DEFAULT_CAN_PORT);

    if (handle == AMOR_INVALID_HANDLE)
    {
        yCError(AMOR) << "Could not get AMOR handle:" << amor_error();
        return false;
    }

    yCInfo(AMOR) << "Acquired AMOR handle!";

    AMOR_JOINT_INFO jointInfo[AMOR_NUM_JOINTS];
    int jointStatus[AMOR_NUM_JOINTS];

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        if (amor_get_joint_info(handle, j, &jointInfo[j]) != AMOR_SUCCESS)
        {
            yCError(AMOR) << "amor_get_joint_info() failed for joint" << j << "with error:" << amor_error();
            return false;
        }

        if (amor_get_status(handle, j, &jointStatus[j]) != AMOR_SUCCESS)
        {
            yCError(AMOR) << "amor_get_status() failed for joint" << j << "with error:" << amor_error();
            return false;
        }
    }

    std::vector<double> positions;

    if (!getEncoders(positions.data()))
    {
        yCError(AMOR) << "getEncoders() failed";
        return false;
    }

    yCInfo(AMOR) << "Current positions (degrees):" << positions;

    // Set position mode.
    if (!positionMove(positions.data()))
    {
        yCError(AMOR) << "positionMove() failed";
        return false;
    }

    yarp::os::Value * cartesianControllerName;

    if (config.check("cartesianControllerName", cartesianControllerName, "cartesian controller port"))
    {
        yCInfo(AMOR) << "Using AMOR cartesian controller device";

        usingCartesianController = true;

        std::string subdevice = "AmorCartesianControl";
        yarp::os::Value vHandle(&handle, sizeof(handle));
        yarp::os::Value vHandleMutex(&handleMutex, sizeof(handleMutex));
        yarp::os::Property cartesianControllerOptions;

        cartesianControllerOptions.fromString((config.toString()));
        cartesianControllerOptions.put("device", "CartesianControlServer");
        cartesianControllerOptions.put("subdevice", subdevice);
        cartesianControllerOptions.put("name", cartesianControllerName->asString());
        cartesianControllerOptions.put("handle", vHandle);
        cartesianControllerOptions.put("handleMutex", vHandleMutex);
        cartesianControllerOptions.setMonitor(config.getMonitor(), subdevice.c_str());

        cartesianControllerDevice.open(cartesianControllerOptions);

        if (!cartesianControllerDevice.isValid())
        {
            yCError(AMOR) << "AMOR cartesian controller device not valid";
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::close()
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
