// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Pci.hpp"

#include <sys/ioctl.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

constexpr auto NUM_SENSORS = 4;

#define CHECK_SENSOR(n) do { if ((n) < 0 || (n) > NUM_SENSORS - 1) return false; } while (0)

// -----------------------------------------------------------------------------

std::size_t Jr3Pci::getNrOfSixAxisForceTorqueSensors() const
{
    return NUM_SENSORS;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status Jr3Pci::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

bool Jr3Pci::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index);
    name = m_names[sens_index];
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Pci::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

bool Jr3Pci::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index);

    six_axis_array fm;

    if (::ioctl(fd, filters[sens_index], &fm) == -1)
    {
        yCError(JR3P) << "ioctl() on read sensor" << sens_index << "failed";
        return false;
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

    return true;
}

// -----------------------------------------------------------------------------
