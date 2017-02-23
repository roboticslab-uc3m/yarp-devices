#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ColorDebug.hpp"

#include "IPlaybackThread.h"

namespace teo
{

class MockupRunnable : public IRunnable
{
    virtual bool run(std::vector<double> &v)
    {
        std::cout << "MockupRunnable: ";
        for(int i=0;i<v.size();i++)
        {
            std::cout << v[i] << " ";
        }
        std::cout << std::endl;
    }
};

/**
* @brief Tests \ref Playback
*/
class PlaybackThreadTest : public testing::Test // -- inherit the Test class (gtest.h)
{

public:

    virtual void SetUp()
    {

        // -- code here will execute just before the test ensues

        yarp::os::Property playbackThreadConf;
        playbackThreadConf.put("device","PlaybackThread");
        playbackThreadConf.put("file","/usr/local/share/teo-body/contexts/Playback/txt/testPlayback.txt");
        playbackThreadConf.put("timeIdx",0);
        playbackThreadConf.fromString("(mask 0 1 0 1)",false);
        bool ok = true;
        ok &= playbackDevice.open(playbackThreadConf);
        ok &= playbackDevice.view(iPlaybackThread);
        if(ok)
        {
            CD_SUCCESS("Configuration sucessful :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration\n");
            ::exit(1);
        }

    }

    virtual void TearDown()
    {
        // -- code here will be called just after the test completes
        // -- ok to through exceptions from here if need be
        playbackDevice.close();
    }

protected:

    /** Playback device. */
    yarp::dev::PolyDriver playbackDevice;
    teo::IPlaybackThread* iPlaybackThread;
};

TEST_F( PlaybackThreadTest, PlaybackThreadTestPlayQuick )
{
    iPlaybackThread->play();
}

TEST_F( PlaybackThreadTest, PlaybackThreadTestPlayFull )
{
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
}

TEST_F( PlaybackThreadTest, PlaybackThreadTestPlayReplay )
{
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
}

TEST_F( PlaybackThreadTest, PlaybackThreadTestPause )
{
    CD_INFO("Play 2 seconds.\n");
    iPlaybackThread->play();
    yarp::os::Time::delay(0.5);
    iPlaybackThread->pause();
    CD_INFO("Pause 7 seconds (normal for one to pass).\n");
    yarp::os::Time::delay(7);
    CD_INFO("Play untill end.\n");
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
}

TEST_F( PlaybackThreadTest, PlaybackThreadTestTimeScale )
{
    CD_INFO("Play x2 slower.\n");
    iPlaybackThread->setTimeScale(2);
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
    CD_INFO("Play x10 faster.\n");
    iPlaybackThread->setTimeScale(0.1);
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
    iPlaybackThread->setTimeScale(1);
}

TEST_F( PlaybackThreadTest, PlaybackThreadTestRunnable )
{
    IRunnable* iRunnable = new MockupRunnable;
    iPlaybackThread->setIRunnable( iRunnable );
    iPlaybackThread->play();
    while( iPlaybackThread->isPlaying() );
    delete iRunnable;
}

}
