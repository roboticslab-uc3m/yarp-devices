// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

// -----------------------------------------------------------------------------

bool teo::Jr3::open(yarp::os::Searchable& config)
{
    if ( ( fd = ::open("/dev/jr3",O_RDWR) ) < 0) {
        perror("Can't open device. No way to read force!\n");
        return false;
    }
    printf("Can read force!\n");

    int retFS = ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs);
    printf("Full scales of Sensor are %d %d %d %d %d %d\n",fs.f[0],fs.f[1],fs.f[2],fs.m[0],fs.m[1],fs.m[2]);
    int retZ = ioctl(fd,IOCTL0_JR3_ZEROOFFS);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Jr3::close()
{
    ::close(fd);
    return true;
}

// -----------------------------------------------------------------------------
