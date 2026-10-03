// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Pci.hpp"

#include <sys/ioctl.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

constexpr auto NUM_SENSORS = 4;

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
#define CHECK_SENSOR(idx) do { std::size_t n; auto ret = getNrOfSixAxisForceTorqueSensors(n); if (!ret || (idx) < 0 || (idx) > n - 1) return yarp::dev::ReturnValue_error_input_out_of_bounds; } while (0)
#else
#define CHECK_SENSOR(idx) do { int n = getNrOfSixAxisForceTorqueSensors(); if ((idx) < 0 || (idx) > n - 1) return false; } while (0)
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue Jr3Pci::getNrOfSixAxisForceTorqueSensors(std::size_t & num) const
{
    num = NUM_SENSORS;
    return yarp::dev::ReturnValue_ok;
}
#else
std::size_t Jr3Pci::getNrOfSixAxisForceTorqueSensors() const
{
    return NUM_SENSORS;
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status Jr3Pci::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue Jr3Pci::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
#else
bool Jr3Pci::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
#endif
{
    CHECK_SENSOR(sens_index);
    name = m_names[sens_index];
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue Jr3Pci::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
#else
bool Jr3Pci::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
#endif
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue Jr3Pci::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool Jr3Pci::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
    CHECK_SENSOR(sens_index);

    six_axis_array fm;

    if (::ioctl(fd, filters[sens_index], &fm) == -1)
    {
        yCError(JR3P) << "ioctl() on read sensor" << sens_index << "failed";
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    static constexpr auto factor = 1.0 / 16384.0;

    out = {
        fm.f[0] * fs[sens_index].f[0] * factor,
        fm.f[1] * fs[sens_index].f[1] * factor,
        fm.f[2] * fs[sens_index].f[2] * factor,
        // torque units are [daNm], therefore we need to divide by 10
        fm.m[0] * fs[sens_index].m[0] * factor * 0.1,
        fm.m[1] * fs[sens_index].m[1] * factor * 0.1,
        fm.m[2] * fs[sens_index].m[2] * factor * 0.1
    };

    if (!m_levogyrate)
    {
        // https://github.com/roboticslab-uc3m/jr3pci-linux/issues/10
        out[0] = -out[0];
        out[3] = -out[3];
    }

    timestamp = yarp::os::SystemClock::nowSystem();

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
