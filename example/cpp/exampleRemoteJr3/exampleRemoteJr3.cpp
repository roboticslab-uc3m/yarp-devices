// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if ( ! yarp::os::Network::checkNetwork() )
    {
        printf("Please start a yarp name server first\n");
        return(1);
    }
    yarp::os::Property options;
    options.put("device","analogsensorclient");
    options.put("remote","/jr3/ch0:o");
    options.put("local","/jr3/ch0:i");

    yarp::dev::PolyDriver dd(options);
    if(!dd.isValid()) {
      printf("Device not available.\n");
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

    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    int channels = iAnalogSensor->getChannels();
    printf("channels: %d\n", channels);

    // Of course we dislike while(1)
    while(1)
    {
        yarp::sig::Vector vector;
        int ret = iAnalogSensor->read(vector);
        if (ret == yarp::dev::IAnalogSensor::AS_OK)
        {
            printf("Good read, got: %s\n",vector.toString().c_str());
        }
        else
        {
            printf("Bad read, error: %d\n",ret);
            //return 1;  // Commenting out, too draconian; on init there can be several until stabilized
        }
    }

	dd.close();

    return 0;
}


