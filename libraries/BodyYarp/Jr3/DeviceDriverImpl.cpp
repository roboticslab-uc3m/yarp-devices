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

    ret=ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs0);
    printf("Full scales of Sensor 0 are %d %d %d %d %d %d\n",fs0.f[0],fs0.f[1],fs0.f[2],fs0.m[0],fs0.m[1],fs0.m[2]);
    ret=ioctl(fd,IOCTL1_JR3_GET_FULL_SCALES,&fs1);
    printf("Full scales of Sensor 1 are: %d %d %d %d %d %d\n", fs1.f[0],fs1.f[1],fs1.f[2],fs1.m[0],fs1.m[1],fs1.m[2]);
    ret=ioctl(fd,IOCTL0_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL1_JR3_ZEROOFFS);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Jr3::close()
{
    ::close(fd);
    return true;
}

// -----------------------------------------------------------------------------
