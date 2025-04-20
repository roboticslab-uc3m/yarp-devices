// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Pci.hpp"

#include <sys/ioctl.h>
#include <fcntl.h> // ::open
#include <unistd.h> // ::close

#include <algorithm> // std::equal
#include <iterator> // std::begin, std::end

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

#define INIT_FILTER_ARRAY(id) do { \
        filters[0] = IOCTL0_JR3_FILTER ## id; \
        filters[1] = IOCTL1_JR3_FILTER ## id; \
        filters[2] = IOCTL2_JR3_FILTER ## id; \
        filters[3] = IOCTL3_JR3_FILTER ## id; \
    } while (0)

namespace
{
    bool checkFullScales(int n, const force_array & actual, const force_array & reference)
    {
        bool isEqual =
            std::equal(std::cbegin(actual.f), std::cend(actual.f), std::cbegin(reference.f)) &&
            std::equal(std::cbegin(actual.m), std::cend(actual.m), std::cbegin(reference.m));

        if (isEqual)
        {
            yCInfo(JR3P) << "Full scales of channel" << n << "set to:" << actual.f << actual.m;
            return true;
        }
        else
        {
            yCError(JR3P) << "Full scales not correctly set for channel" << n;
            yCInfo(JR3P) << "Actual:" << actual.f << actual.m;
            yCInfo(JR3P) << "Reference:" << reference.f << reference.m;
            return false;
        }
    }
}

// -----------------------------------------------------------------------------

bool Jr3Pci::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(JR3P) << "Failed to parse parameters";
        return false;
    }

    if (!m_names.empty() && m_names.size() != 4)
    {
        yCError(JR3P) << "Sensor list must have 4 elements, instead it has" << m_names.size();
        return false;
    }

    if (m_filter < 0 || m_filter > 6)
    {
        yCError(JR3P) << "Filter value must be between 0 and 6, instead it is" << m_filter;
        return false;
    }

    loadFilters(m_filter);

    if ((fd = ::open("/dev/jr3", O_RDWR)) < 0)
    {
        yCError(JR3P) << "Can't open device, no way to read force!";
        return false;
    }

    //-- Force fullscales (hardcoded for TEO sensors).
    //-- Was `fs_w.m[X] = 5.5`, see https://github.com/roboticslab-uc3m/yarp-devices/issues/184

    force_array fs_w, fs_a0, fs_a1;

    fs_w.f[0] = 110;
    fs_w.f[1] = 110;
    fs_w.f[2] = 220;
    fs_w.m[0] = 5;
    fs_w.m[1] = 5;
    fs_w.m[2] = 5;

    fs_a0.f[0] = 317;
    fs_a0.f[1] = 314;
    fs_a0.f[2] = 845;
    fs_a0.m[0] = 221;
    fs_a0.m[1] = 219;
    fs_a0.m[2] = 307;

    fs_a1.f[0] = 326;
    fs_a1.f[1] = 325;
    fs_a1.f[2] = 870;
    fs_a1.m[0] = 224;
    fs_a1.m[1] = 224;
    fs_a1.m[2] = 309;

    ::ioctl(fd, IOCTL0_JR3_SET_FULL_SCALES, &fs_a0);
    ::ioctl(fd, IOCTL1_JR3_SET_FULL_SCALES, &fs_a1);
    ::ioctl(fd, IOCTL2_JR3_SET_FULL_SCALES, &fs_w);
    ::ioctl(fd, IOCTL3_JR3_SET_FULL_SCALES, &fs_w);

    yarp::os::SystemClock::delaySystem(0.5);

    //-- Make sure fullscales were set.
    ::ioctl(fd, IOCTL0_JR3_GET_FULL_SCALES, &fs[0]);
    ::ioctl(fd, IOCTL1_JR3_GET_FULL_SCALES, &fs[1]);
    ::ioctl(fd, IOCTL2_JR3_GET_FULL_SCALES, &fs[2]);
    ::ioctl(fd, IOCTL3_JR3_GET_FULL_SCALES, &fs[3]);

    return
        checkFullScales(0, fs[0], fs_a0) &&
        checkFullScales(1, fs[1], fs_a1) &&
        checkFullScales(2, fs[2], fs_w) &&
        checkFullScales(3, fs[3], fs_w) &&
        // The acquisition board will perform set-to-zero exactly once on power-up,
        // so there is no way to know the initial force/torque values in case a
        // payload is attached (e.g. a gripper). To keep things more deterministic,
        // this device will additionally set-to-zero during start by default.
        // https://github.com/roboticslab-uc3m/jr3pci-linux/issues/11
        calibrateSensor() == yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

bool Jr3Pci::close()
{
    if (fd)
    {
        ::close(fd);
        fd = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------

void Jr3Pci::loadFilters(int id)
{
    switch (id)
    {
    case 0:
        INIT_FILTER_ARRAY(0);
        break;
    case 1:
        INIT_FILTER_ARRAY(1);
        break;
    case 2:
        INIT_FILTER_ARRAY(2);
        break;
    case 3:
        INIT_FILTER_ARRAY(3);
        break;
    case 4:
        INIT_FILTER_ARRAY(4);
        break;
    case 5:
        INIT_FILTER_ARRAY(5);
        break;
    case 6:
        INIT_FILTER_ARRAY(6);
        break;
    }
}

// -----------------------------------------------------------------------------
