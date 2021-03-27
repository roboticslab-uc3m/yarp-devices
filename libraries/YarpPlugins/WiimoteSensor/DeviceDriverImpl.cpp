// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::open(yarp::os::Searchable& config)
{
    yDebug() << "WiimoteSensor config:" << config.toString();

    int deviceId = config.check("deviceId", yarp::os::Value(DEFAULT_DEVICE), "Wiimote device number").asInt32();

    char * syspath = getDevicePath(deviceId);

    if (syspath == NULL)
    {
        yError() << "Cannot find device with number" << deviceId;
        return false;
    }

    int ret = xwii_iface_new(&iface, syspath);

    if (ret < 0)
    {
        yError("Cannot create xwii_iface %s: %d", syspath, ret);
        std::free(syspath);
        return false;
    }

    yInfo() << "Created xwii_iface" << syspath;
    std::free(syspath);

    ret = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL);

    if (ret < 0)
    {
        yError() << "Cannot open interface, did you launch with sudo?" << ret;
        return false;
    }

    calibZeroX = config.check("calibZeroX", yarp::os::Value(DEFAULT_CALIB_ZERO_X), "normalization value for X axis (zero)").asInt32();
    calibZeroY = config.check("calibZeroY", yarp::os::Value(DEFAULT_CALIB_ZERO_Y), "normalization value for Y axis (zero)").asInt32();
    calibZeroZ = config.check("calibZeroZ", yarp::os::Value(DEFAULT_CALIB_ZERO_Z), "normalization value for Z axis (zero)").asInt32();

    calibOneX = config.check("calibOneX", yarp::os::Value(DEFAULT_CALIB_ONE_X), "normalization value for X axis (one)").asInt32();
    calibOneY = config.check("calibOneY", yarp::os::Value(DEFAULT_CALIB_ONE_Y), "normalization value for Y axis (one)").asInt32();
    calibOneZ = config.check("calibOneZ", yarp::os::Value(DEFAULT_CALIB_ONE_Z), "normalization value for Z axis (one)").asInt32();

    yInfo("Calibration (zero): x = %d, y = %d, z = %d", calibZeroX, calibZeroY, calibZeroZ);
    yInfo("Calibration (one): x = %d, y = %d, z = %d", calibOneX, calibOneY, calibOneZ);

    dispatcherThread.setInterfacePointer(iface);
    return dispatcherThread.start();
}

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::close()
{
    dispatcherThread.stop();

    if (iface)
    {
        xwii_iface_unref(iface);
        iface = NULL;
    }

    return true;
}

// -----------------------------------------------------------------------------
