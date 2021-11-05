// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

#include <sys/ioctl.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto NUM_SENSORS = 4;

#define CHECK_SENSOR(n) do { if ((n) < 0 || (n) > NUM_SENSORS - 1) return false; } while (0)

// -----------------------------------------------------------------------------

std::size_t Jr3::getNrOfSixAxisForceTorqueSensors() const
{
    return NUM_SENSORS;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status Jr3::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

bool Jr3::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index);
    name = names[sens_index];
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

bool Jr3::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index);

    six_axis_array fm;

    if (::ioctl(fd, filters[sens_index], &fm) == -1)
    {
        yCError(JR3) << "ioctl() on read sensor" << sens_index << "failed";
        return false;
    }

    out.resize(6);

    constexpr auto factor = 1.0 / 16384.0;

    out[0] = fm.f[0] * fs[sens_index].f[0] * factor;
    out[1] = fm.f[1] * fs[sens_index].f[1] * factor;
    out[2] = fm.f[2] * fs[sens_index].f[2] * factor;
    out[3] = fm.m[0] * fs[sens_index].m[0] * factor;
    out[4] = fm.m[1] * fs[sens_index].m[1] * factor;
    out[5] = fm.m[2] * fs[sens_index].m[2] * factor;

    timestamp = yarp::os::SystemClock::nowSystem();

    return true;
}

// -----------------------------------------------------------------------------
