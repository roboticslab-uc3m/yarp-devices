// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "IPlaybackThread.h"

class PositionMoveRunnable : public teo::IRunnable
{
public:
    virtual bool run(std::vector<double> &v)
    {
        iPositionControl->positionMove( v.data() );
        return true;
    }
    yarp::dev::IPositionControl* iPositionControl;
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    yarp::dev::PolyDriver playbackThreadDevice;
    teo::IPlaybackThread *iPlaybackThread;

    yarp::dev::PolyDriver robotDevice;
    PositionMoveRunnable positionMoveRunnable;

    if ( ! yarp::os::Network::checkNetwork() )
    {
        printf("Please start a yarp name server first\n");
        return(1);
    }

    //-- playbackThreadDevice and interface
    yarp::os::Property playbackThreadOptions;
    playbackThreadOptions.put("device","PlaybackThread");
    playbackThreadOptions.put("file","/usr/local/share/yarp-devices/contexts/Playback/txt/yarpdatadumper-teo-right-arm.txt");
    playbackThreadOptions.put("timeIdx",1);
    playbackThreadOptions.put("timeScale",0.000001);
    playbackThreadOptions.fromString("(mask 0 0 1 1 1 1 1 1 1)",false);
    playbackThreadDevice.open(playbackThreadOptions);
    if( ! playbackThreadDevice.isValid() )
    {
        printf("playbackThreadDevice not available.\n");
        playbackThreadDevice.close();
        yarp::os::Network::fini();
        return 1;
    }

    if ( ! playbackThreadDevice.view(iPlaybackThread) )
    {
        printf("[error] Problems acquiring iPlaybackThread interface\n");
        return 1;
    }
    printf("[success] acquired iPlaybackThread interface\n");

    //-- robotDevice
    yarp::os::Property robotOptions;
    robotOptions.put("device","remote_controlboard");
    robotOptions.put("local","/playback");
    robotOptions.put("remote","/teoSim/rightArm");
    robotDevice.open(robotOptions);
    if( ! robotDevice.isValid() )
    {
        printf("robotDevice not available.\n");
        robotDevice.close();
        yarp::os::Network::fini();
        return 1;
    }
    robotDevice.view( positionMoveRunnable.iPositionControl );
    teo::IRunnable* iRunnable = dynamic_cast< teo::IRunnable* >( &positionMoveRunnable );
    iPlaybackThread->setIRunnable( iRunnable );

    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );

    robotDevice.close();
    playbackThreadDevice.close();

    return 0;
}


