// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

#include <sys/ioctl.h>
#include <fcntl.h> // ::open
#include <unistd.h> // ::close

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

#define INIT_FILTER_ARRAY(id) do { \
        filters[0] = IOCTL0_JR3_FILTER ## id; \
        filters[1] = IOCTL1_JR3_FILTER ## id; \
        filters[2] = IOCTL2_JR3_FILTER ## id; \
        filters[3] = IOCTL3_JR3_FILTER ## id; \
    } while (0)

using namespace roboticslab;

constexpr auto DEFAULT_FILTER_ID = 0;

// -----------------------------------------------------------------------------

bool Jr3::open(yarp::os::Searchable& config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(JR3) << "Config:" << config.toString();
#endif

    yarp::os::Value * vNames;

    if (config.check("names", vNames, "sensor names"))
    {
        if (!vNames->isList())
        {
            yCError(JR3) << "Parameter --names must be a list";
            return false;
        }

        const auto b = vNames->asList();

        if (b->size() > 4)
        {
            yCError(JR3) << "Too many sensors:" << b->size();
            return false;
        }

        for (auto i = 0; i < b->size(); ++i)
        {
            if (!b->get(i).isString())
            {
                yCError(JR3) << "Sensor name must be a string";
                return false;
            }

            names[i] = b->get(i).asString();
        }

        yCInfo(JR3) << "Using sensor names:" << names;
    }

    int filterId = config.check("filter", yarp::os::Value(DEFAULT_FILTER_ID), "filter id (0-6)").asInt32();

    if (filterId < 0 || filterId > 6)
    {
        yCError(JR3) << "Illegal filter ID (<0 or >6):" << filterId;
        return false;
    }

    loadFilters(filterId);

    if ((fd = ::open("/dev/jr3", O_RDWR)) < 0)
    {
        yCError(JR3) << "Can't open device, no way to read force!";
        return false;
    }

    //-- Force fullscales.
    force_array fs_w, fs_a0, fs_a1;

    fs_w.f[0] = 110;
    fs_w.f[1] = 110;
    fs_w.f[2] = 220;
    fs_w.m[0] = 5.5;
    fs_w.m[1] = 5.5;
    fs_w.m[2] = 5.5;

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
    yCInfo(JR3) << "Full scales of Sensor 0 are:" << fs[0].f[0] << fs[0].f[1] << fs[0].f[2] << fs[0].m[0] << fs[0].m[1] << fs[0].m[2];
    ::ioctl(fd, IOCTL1_JR3_GET_FULL_SCALES, &fs[1]);
    yCInfo(JR3) << "Full scales of Sensor 1 are:" << fs[1].f[0] << fs[1].f[1] << fs[1].f[2] << fs[1].m[0] << fs[1].m[1] << fs[1].m[2];
    ::ioctl(fd, IOCTL2_JR3_GET_FULL_SCALES, &fs[2]);
    yCInfo(JR3) << "Full scales of Sensor 2 are:" << fs[2].f[0] << fs[2].f[1] << fs[2].f[2] << fs[2].m[0] << fs[2].m[1] << fs[2].m[2];
    ::ioctl(fd, IOCTL3_JR3_GET_FULL_SCALES, &fs[3]);
    yCInfo(JR3) << "Full scales of Sensor 3 are:" << fs[3].f[0] << fs[3].f[1] << fs[3].f[2] << fs[3].m[0] << fs[3].m[1] << fs[3].m[2];

    ::ioctl(fd, IOCTL0_JR3_ZEROOFFS);
    ::ioctl(fd, IOCTL1_JR3_ZEROOFFS);
    ::ioctl(fd, IOCTL2_JR3_ZEROOFFS);
    ::ioctl(fd, IOCTL3_JR3_ZEROOFFS);

    return true;
}

// -----------------------------------------------------------------------------

bool Jr3::close()
{
    if (fd)
    {
        ::close(fd);
        fd = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------

void Jr3::loadFilters(int id)
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
