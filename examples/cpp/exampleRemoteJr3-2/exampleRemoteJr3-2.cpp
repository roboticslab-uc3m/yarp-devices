// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <yarp/sig/Vector.h>


using namespace std;

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if ( ! yarp::os::Network::checkNetwork() )
    {
        std::printf("Please start a yarp name server first\n");
        return(1);
    }
    yarp::os::Property options;
    options.put("device","Jr3");
    options.put("remote","/jr3:o");
    options.put("local","/jr3:i");

    yarp::dev::PolyDriver dd(options);
    if(!dd.isValid()) {
      std::printf("Device not available.\n");
      dd.close();
      yarp::os::Network::fini();
      return 1;
    }

    yarp::dev::IAnalogSensor *iAnalogSensor;

    if ( ! dd.view(iAnalogSensor) )
    {
        printf("[error] Problems acquiring interface\n");
        return 1;
    }
    printf("[success] acquired interface\n");

    int channels = iAnalogSensor->getChannels();
    printf("channels: %d\n", channels);

    if(argc > 1){
        if(string(argv[1])=="reset"){
            if(string(argv[2])=="all")
            {
                int ret = iAnalogSensor->calibrateSensor();
                if (ret == yarp::dev::IAnalogSensor::AS_OK)
                    printf("[OK] All channels reseted\n");
                else
                {
                    printf("[ERROR] Calibrating sensor...\n");
                    return 1;
                }
            }
            if(string(argv[2])=="channel")
            {
                int ch = atoi(argv[3]);
                int ret = iAnalogSensor->calibrateChannel(ch);
                if (ret == yarp::dev::IAnalogSensor::AS_OK)
                    printf("[OK] Channel (%d) reseted\n",ch);
                else
                {
                    printf("[ERROR] Calibrating channel...\n");
                    return 1;
                }
            }
        }
    }

    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    char ch;
    int cont=0;
    // Of course we dislike while(1)
    while(1)
    {
        if (cont>500) {
            printf("reset sensor? select channel (0,1,2,3) or 'a' to reset all\n");
            ch=getchar();
                if(ch=='0' || ch=='1' || ch=='2' || ch=='3')
                {
                    printf("channel chosen: %d\n",(int)ch-'0');
                    int ret = iAnalogSensor->calibrateChannel((int)ch-'0');
                    if (ret == yarp::dev::IAnalogSensor::AS_OK)
                        printf("[OK] Channel (%d) reseted\n",ch);
                    else
                    {
                        printf("[ERROR] Calibrating channel...\n");
                        return 1;
                    }
                }
                if(ch=='a')
                {
                    int ret = iAnalogSensor->calibrateSensor();
                    if (ret == yarp::dev::IAnalogSensor::AS_OK)
                        printf("[OK] All channels reseted\n");
                    else
                    {
                        printf("[ERROR] Calibrating sensor...\n");
                        return 1;
                    }
                }

                cont=0;
            }

        yarp::sig::Vector vector;
        int ret = iAnalogSensor->read(vector);
        if (ret == yarp::dev::IAnalogSensor::AS_OK)
        {
            printf("Channel 0: ( ");
            for(int i=0; i<6; i++)
                printf("%f ",vector[i]);
            printf(")\n ");

            printf("Channel 1: ( ");
            for(int i=6; i<12; i++)
                printf("%f ",vector[i]);
            printf(")\n ");

            printf("Channel 2: ( ");
            for(int i=12; i<18; i++)
                printf("%f ",vector[i]);
            printf(")\n ");

            printf("Channel 3: ( ");
            for(int i=18; i<24; i++)
                printf("%f ",vector[i]);
            printf(")\n ");

            cont++;
            yarp::os::Time::delay(0.01);
        }
        else
        {
            std::printf("Bad read, error: %d\n",ret);
            //return 1;  // Commenting out, too draconian; on init there can be several until stabilized
        }
    }

    dd.close();

    return 0;
}

