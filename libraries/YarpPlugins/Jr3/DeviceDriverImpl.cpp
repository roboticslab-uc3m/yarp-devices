// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::Jr3::open(yarp::os::Searchable& config)
{
    if ( ( fd = ::open("/dev/jr3",O_RDWR) ) < 0) {
        CD_ERROR("Can't open device. No way to read force!\n");
        return false;
    }
    CD_SUCCESS("Can read force!\n");

    int ret;

    //-- Force fullscales.
    force_array fs_w,fs_a;
    fs_w.f[0]=110;
    fs_w.f[1]=110;
    fs_w.f[2]=220;
    fs_w.m[0]=5.5;
    fs_w.m[1]=5.5;
    fs_w.m[2]=5.5;

    fs_a.f[0]=317;
    fs_a.f[1]=314;
    fs_a.f[2]=845;
    fs_a.m[0]=221;
    fs_a.m[1]=219;
    fs_a.m[2]=307;

    ioctl(fd,IOCTL0_JR3_SET_FULL_SCALES,&fs_a);
    ioctl(fd,IOCTL1_JR3_SET_FULL_SCALES,&fs_a);
    ioctl(fd,IOCTL2_JR3_SET_FULL_SCALES,&fs_w);
    ioctl(fd,IOCTL3_JR3_SET_FULL_SCALES,&fs_w);


    yarp::os::Time::delay(0.5);

    //-- Make sure fullscales were set.
    ret=ioctl(fd,IOCTL0_JR3_GET_FULL_SCALES,&fs0);
    CD_INFO("Full scales of Sensor 0 are %d %d %d %d %d %d\n",fs0.f[0],fs0.f[1],fs0.f[2],fs0.m[0],fs0.m[1],fs0.m[2]);
    ret=ioctl(fd,IOCTL1_JR3_GET_FULL_SCALES,&fs1);
    CD_INFO("Full scales of Sensor 1 are: %d %d %d %d %d %d\n", fs1.f[0],fs1.f[1],fs1.f[2],fs1.m[0],fs1.m[1],fs1.m[2]);
    ret=ioctl(fd,IOCTL2_JR3_GET_FULL_SCALES,&fs2);
    CD_INFO("Full scales of Sensor 2 are: %d %d %d %d %d %d\n", fs2.f[0],fs2.f[1],fs2.f[2],fs2.m[0],fs2.m[1],fs2.m[2]);
    ret=ioctl(fd,IOCTL3_JR3_GET_FULL_SCALES,&fs3);
    CD_INFO("Full scales of Sensor 3 are: %d %d %d %d %d %d\n", fs3.f[0],fs3.f[1],fs3.f[2],fs3.m[0],fs3.m[1],fs3.m[2]);

    ret=ioctl(fd,IOCTL0_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL1_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL2_JR3_ZEROOFFS);
    ret=ioctl(fd,IOCTL3_JR3_ZEROOFFS);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::Jr3::close()
{
    ::close(fd);
    return true;
}

// -----------------------------------------------------------------------------
