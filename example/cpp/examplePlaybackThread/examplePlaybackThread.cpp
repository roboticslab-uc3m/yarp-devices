// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "IPlaybackThread.h"

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if ( ! yarp::os::Network::checkNetwork() )
    {
        printf("Please start a yarp name server first\n");
        return(1);
    }
    yarp::os::Property options;
    options.put("device","PlaybackThread");
    options.put("file","/usr/local/share/teo-body/contexts/Playback/txt/yarpdatadumper-teo-right-arm.txt");
    options.put("timeIdx",1);
    options.put("timeScale",0.000001);
    options.fromString("(mask 0 0 1 1 1 1 1 1 1)",false);
    yarp::dev::PolyDriver dd(options);
    if(!dd.isValid()) {
      printf("Device not available.\n");
	  dd.close();
      yarp::os::Network::fini();
      return 1;
    }

    teo::IPlaybackThread *iPlaybackThread;

    if ( ! dd.view(iPlaybackThread) )
    {
        printf("[error] Problems acquiring interface\n");
        return 1;
    }
	printf("[success] acquired interface\n");

    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );

	dd.close();

    return 0;
}


