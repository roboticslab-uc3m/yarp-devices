// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

#include <sys/ioctl.h>

#include <yarp/os/LogStream.h>

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
    yCDebug(JR3) << "Config:" << config.toString();

    int filterId = config.check("filter", yarp::os::Value(DEFAULT_FILTER_ID), "filter id (0-6)").asInt32();

    if (filterId < 0 || filterId > 6)
    {
        yCError(JR3) << "Illegal filter ID (<0 or >6):" << filterId;
        return false;
    }

    loadFilters(filterId);

    if (( fd = ::open("/dev/jr3", O_RDWR)) < 0)
    {
        yCError(JR3) << "Can't open device, no way to read force!";
        return false;
    }

    int ret;

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

    ioctl(fd, IOCTL0_JR3_SET_FULL_SCALES, &fs_a0);
    ioctl(fd, IOCTL1_JR3_SET_FULL_SCALES, &fs_a1);
    ioctl(fd, IOCTL2_JR3_SET_FULL_SCALES, &fs_w);
    ioctl(fd, IOCTL3_JR3_SET_FULL_SCALES, &fs_w);


    yarp::os::Time::delay(0.5);

    //-- Make sure fullscales were set.
    ret=ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs0);
    yCInfo(JR3) << "Full scales of Sensor 0 are:" << fs0.f[0] << fs0.f[1] << fs0.f[2] << fs0.m[0] << fs0.m[1] << fs0.m[2];
    ret=ioctl(fd,IOCTL1_JR3_GET_FULL_SCALES,&fs1);
    yCInfo(JR3) << "Full scales of Sensor 1 are:" << fs1.f[0] << fs1.f[1] << fs1.f[2] << fs1.m[0] << fs1.m[1] << fs1.m[2];
    ret=ioctl(fd,IOCTL2_JR3_GET_FULL_SCALES,&fs2);
    yCInfo(JR3) << "Full scales of Sensor 2 are:" << fs2.f[0] << fs2.f[1] << fs2.f[2] << fs2.m[0] << fs2.m[1] << fs2.m[2];
    ret=ioctl(fd,IOCTL3_JR3_GET_FULL_SCALES,&fs3);
    yCInfo(JR3) << "Full scales of Sensor 3 are:" << fs3.f[0] << fs3.f[1] << fs3.f[2] << fs3.m[0] << fs3.m[1] << fs3.m[2];

    ret = ioctl(fd, IOCTL0_JR3_ZEROOFFS);
    ret = ioctl(fd, IOCTL1_JR3_ZEROOFFS);
    ret = ioctl(fd, IOCTL2_JR3_ZEROOFFS);
    ret = ioctl(fd, IOCTL3_JR3_ZEROOFFS);

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
