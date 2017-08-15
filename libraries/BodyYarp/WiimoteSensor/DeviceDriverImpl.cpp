// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>
#include <cstring>

#include <yarp/os/Value.h>

#include "ColorDebug.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::open(yarp::os::Searchable& config)
{
    int deviceId = config.check("deviceId", yarp::os::Value(DEFAULT_DEVICE), "Wiimote device number").asInt();

    char * syspath = getDevicePath(deviceId);

    if (syspath == NULL)
    {
        CD_ERROR("Cannot find device with number %d.\n", deviceId);
        return false;
    }

    int ret = xwii_iface_new(&iface, syspath);

    if (ret < 0)
    {
        CD_ERROR("Cannot create xwii_iface '%s': %d.\n", syspath, ret);
        std::free(syspath);
        return false;
    }

    CD_SUCCESS("Created xwii_iface '%s'.\n", syspath);
    std::free(syspath);

    ret = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL);

    if (ret < 0)
    {
        CD_ERROR("Cannot open interface, did you launch with sudo? (%d).\n", ret);
        close();
        return false;
    }

    std::memset(fds, 0, sizeof(fds));

    fds[0].fd = 0;
    fds[0].events = POLLIN;

    fds[1].fd = xwii_iface_get_fd(iface);
    fds[1].events = POLLIN;

    fds_num = 2;

    calibZeroX = config.check("calibZeroX", yarp::os::Value(DEFAULT_CALIB_ZERO_X), "normalization value for X axis (zero)").asInt();
    calibZeroY = config.check("calibZeroY", yarp::os::Value(DEFAULT_CALIB_ZERO_Y), "normalization value for Y axis (zero)").asInt();
    calibZeroZ = config.check("calibZeroZ", yarp::os::Value(DEFAULT_CALIB_ZERO_Z), "normalization value for Z axis (zero)").asInt();

    calibOneX = config.check("calibOneX", yarp::os::Value(DEFAULT_CALIB_ONE_X), "normalization value for X axis (one)").asInt();
    calibOneY = config.check("calibOneY", yarp::os::Value(DEFAULT_CALIB_ONE_Y), "normalization value for Y axis (one)").asInt();
    calibOneZ = config.check("calibOneZ", yarp::os::Value(DEFAULT_CALIB_ONE_Z), "normalization value for Z axis (one)").asInt();

    CD_INFO("Calibration (zero): x = %d, y = %d, z = %d\n", calibZeroX, calibZeroY, calibZeroZ);
    CD_INFO("Calibration (one): x = %d, y = %d, z = %d\n", calibOneX, calibOneY, calibOneZ);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::close()
{
    xwii_iface_unref(iface);
    return true;
}

// -----------------------------------------------------------------------------
