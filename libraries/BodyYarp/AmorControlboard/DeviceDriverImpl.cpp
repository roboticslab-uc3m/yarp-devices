// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- DeviceDriver related ------------------------------------

bool roboticslab::AmorControlboard::open(yarp::os::Searchable& config)
{
    int major, minor, build;
    amor_get_library_version(&major, &minor, &build);

    CD_INFO("AMOR API library version %d.%d.%d\n", major, minor, build);
    CD_INFO("Trying to connect to AMOR...\n");

    handle = amor_connect((char *)DEFAULT_CAN_LIBRARY, DEFAULT_CAN_PORT);

    if (handle == AMOR_INVALID_HANDLE)
    {
        CD_ERROR("Could not get AMOR handle (%s)\n", amor_error());
        return false;
    }

    CD_SUCCESS("Acquired AMOR handle!\n");

    AMOR_JOINT_INFO jointInfo[AMOR_NUM_JOINTS];
    int jointStatus[AMOR_NUM_JOINTS];

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        if (amor_get_joint_info(handle, j, &jointInfo[j]) != AMOR_SUCCESS)
        {
            CD_ERROR("%s (%d).\n", amor_error(), j);
            amor_release(handle);
            return false;
        }

        if (amor_get_status(handle, j, &jointStatus[j]) != AMOR_SUCCESS)
        {
            CD_ERROR("%s (%d).\n", amor_error(), j);
            amor_release(handle);
            return false;
        }
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        amor_release(handle);
        return false;
    }

    CD_INFO("Current positions (degrees): [");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        CD_INFO_NO_HEADER("%f", toDeg(positions[j]));

        if (j != AMOR_NUM_JOINTS - 1)
        {
            CD_INFO_NO_HEADER(" ");
        }
    }

    CD_INFO_NO_HEADER("]\n");

    // Set position mode.
    if (amor_set_positions(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        amor_release(handle);
        return false;
    }

    if (config.check("useAmorCartesianController"))
    {
        CD_INFO("Using AMOR cartesian controller device.\n");

        usingCartesianController = true;

        yarp::os::Value vHandle(handle, sizeof handle);
        yarp::os::Property cartesianControllerOptions;

        cartesianControllerOptions.put("device", "CartesianControlServer");
        cartesianControllerOptions.put("subdevice", "AmorCartesianControl");
        cartesianControllerOptions.put("handle", vHandle);

        cartesianControllerDevice.open(cartesianControllerOptions);

        if (!cartesianControllerDevice.isValid())
        {
            CD_ERROR("AMOR cartesian controller device not valid.\n");
            amor_release(handle);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::close()
{
    CD_INFO("Closing AmorControlboard...\n");

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
