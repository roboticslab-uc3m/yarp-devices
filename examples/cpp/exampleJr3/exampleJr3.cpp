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
    options.put("device", "Jr3");

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

    printf("[success] acquired interface\n");

    int channels = iAnalogSensor->getChannels();
    std::printf("channels: %d\n", channels);

    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    char ch;
    int cont = 0;

    // Of course we dislike while(1)
    while (true)
    {
        // every 500 readings of the sensor, the app asks you if you want to reset it (calibrate chennel's sensor to 0)
        if (cont > 500)
        {
            std::printf("reset sensor? select channel (0,1,2,3) or 'a' to reset all\n");
            ch = getchar();

            if (ch == '0' || ch == '1' || ch == '2' || ch == '3')
            {
                std::printf("channel chosen: %d\n", (int)ch - '0');

                int ret = iAnalogSensor->calibrateChannel((int)ch - '0');

                if (ret == yarp::dev::IAnalogSensor::AS_OK)
                {
                    std::printf("[OK] Channel (%d) reset\n", ch);
                }
                else
                {
                    std::printf("[ERROR] Calibrating channel...\n");
                    return 1;
                }
            }

            if (ch == 'a')
            {
                int ret = iAnalogSensor->calibrateSensor();

                if (ret == yarp::dev::IAnalogSensor::AS_OK)
                {
                    std::printf("[OK] All channels reseted\n");
                }
                else
                {
                    std::printf("[ERROR] Calibrating sensor...\n");
                    return 1;
                }
            }

            cont = 0;
        }

        // sensor reading
        yarp::sig::Vector vector;
        int ret = iAnalogSensor->read(vector);

        if (ret == yarp::dev::IAnalogSensor::AS_OK)
        {
            std::printf("Channel 0: ( ");

            for (int i = 0; i < 6; i++)
            {
                std::printf("%f ", vector[i]);
            }

            std::printf(")\n ");

            std::printf("Channel 1: ( ");

            for (int i = 6; i < 12; i++)
            {
                std::printf("%f ", vector[i]);
            }

            std::printf(")\n ");

            std::printf("Channel 2: ( ");

            for (int i = 12; i < 18; i++)
            {
                std::printf("%f ", vector[i]);
            }

            std::printf(")\n ");

            std::printf("Channel 3: ( ");

            for (int i = 18; i < 24; i++)
            {
                std::printf("%f ", vector[i]);
            }

            std::printf(")\n ");

            cont++;
            yarp::os::Time::delay(0.01);
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
