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
    else
    {
        CD_SUCCESS("Acquired AMOR handle!\n");
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
