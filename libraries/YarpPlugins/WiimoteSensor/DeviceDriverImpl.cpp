// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_DEVICE = 1;

constexpr auto DEFAULT_CALIB_ZERO_X = -30;
constexpr auto DEFAULT_CALIB_ZERO_Y = -22;
constexpr auto DEFAULT_CALIB_ZERO_Z = 72;

constexpr auto DEFAULT_CALIB_ONE_X = 69;
constexpr auto DEFAULT_CALIB_ONE_Y = -123;
constexpr auto DEFAULT_CALIB_ONE_Z = -25;

// -----------------------------------------------------------------------------

bool WiimoteSensor::open(yarp::os::Searchable& config)
{
#if YARP_VERSION_MINOR < 6
    yCDebug(WII) << "Config:" << config.toString();
#endif

    int deviceId = config.check("deviceId", yarp::os::Value(DEFAULT_DEVICE), "Wiimote device number").asInt32();

    char * syspath = getDevicePath(deviceId);

    if (syspath == nullptr)
    {
        yCError(WII) << "Cannot find device with number" << deviceId;
        return false;
    }

    int ret = xwii_iface_new(&iface, syspath);

    if (ret < 0)
    {
        yCError(WII, "Cannot create xwii_iface %s: %d", syspath, ret);
        std::free(syspath);
        return false;
    }

    yCInfo(WII) << "Created xwii_iface" << syspath;
    std::free(syspath);

    ret = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL);

    if (ret < 0)
    {
        yCError(WII) << "Cannot open interface, did you launch with sudo?" << ret;
        return false;
    }

    calibZeroX = config.check("calibZeroX", yarp::os::Value(DEFAULT_CALIB_ZERO_X), "normalization value for X axis (zero)").asInt32();
    calibZeroY = config.check("calibZeroY", yarp::os::Value(DEFAULT_CALIB_ZERO_Y), "normalization value for Y axis (zero)").asInt32();
    calibZeroZ = config.check("calibZeroZ", yarp::os::Value(DEFAULT_CALIB_ZERO_Z), "normalization value for Z axis (zero)").asInt32();

    calibOneX = config.check("calibOneX", yarp::os::Value(DEFAULT_CALIB_ONE_X), "normalization value for X axis (one)").asInt32();
    calibOneY = config.check("calibOneY", yarp::os::Value(DEFAULT_CALIB_ONE_Y), "normalization value for Y axis (one)").asInt32();
    calibOneZ = config.check("calibOneZ", yarp::os::Value(DEFAULT_CALIB_ONE_Z), "normalization value for Z axis (one)").asInt32();

    yCInfo(WII, "Calibration (zero): x = %d, y = %d, z = %d", calibZeroX, calibZeroY, calibZeroZ);
    yCInfo(WII, "Calibration (one): x = %d, y = %d, z = %d", calibOneX, calibOneY, calibOneZ);

    yawActive = false;

    dispatcherThread.setInterfacePointer(iface);
    return dispatcherThread.start();
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::close()
{
    dispatcherThread.stop();

    if (iface)
    {
        xwii_iface_unref(iface);
        iface = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------
