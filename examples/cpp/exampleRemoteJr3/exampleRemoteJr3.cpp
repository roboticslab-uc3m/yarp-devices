// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <yarp/sig/Vector.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "analogsensorclient");
    options.put("remote", "/jr3/ch0:o");
    options.put("local", "/jr3/ch0:i");

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }

    yarp::dev::IAnalogSensor *iAnalogSensor;

    if (!dd.view(iAnalogSensor))
    {
        std::printf("[error] Problems acquiring interface\n");
        return 1;
    }

    std::printf("[success] acquired interface\n");

    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    int channels = iAnalogSensor->getChannels();
    std::printf("channels: %d\n", channels);

    // Of course we dislike while(1)
    while (true)
    {
        yarp::sig::Vector vector;
        int ret = iAnalogSensor->read(vector);

        if (ret == yarp::dev::IAnalogSensor::AS_OK)
        {
            std::printf("Good read, got: %s\n", vector.toString().c_str());
        }
        else
        {
            std::printf("Bad read, error: %d\n", ret);
            //return 1;  // Commenting out, too draconian; on init there can be several until stabilized
        }
    }

    dd.close();

    return 0;
}
