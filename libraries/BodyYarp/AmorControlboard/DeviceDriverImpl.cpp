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

    if (config.check("useAmorCartesianController"))
    {
        CD_INFO("Using AMOR cartesian controller device.\n");

        yarp::os::Value vHandle(handle, sizeof handle);
        yarp::os::Property cartesianControllerOptions;

        cartesianControllerOptions.put("device", "AmorCartesianController");
        cartesianControllerOptions.put("handle", vHandle);

        yarp::dev::PolyDriver cartesianControllerDevice = yarp::dev::PolyDriver(cartesianControllerOptions);

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
    CD_INFO("Closing...\n");

    if (handle != AMOR_INVALID_HANDLE)
    {
        amor_emergency_stop(handle);
        amor_release(handle);
    }

    return true;
}

// -----------------------------------------------------------------------------
