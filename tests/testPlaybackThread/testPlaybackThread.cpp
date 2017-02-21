#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ColorDebug.hpp"

#include "IPlaybackThread.h"

namespace teo
{

/**
* @brief Tests \ref Playback
*/
class PlaybackThreadTest : public testing::Test // -- inherit the Test class (gtest.h)
{

public:

    virtual void SetUp()
    {

        // -- code here will execute just before the test ensues

        yarp::os::Property playbackThreadConf ("(device PlaybackThread) (file /usr/local/share/teo-body/contexts/Playback/txt/testPlayback.txt)");
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

TEST_F( PlaybackThreadTest, PlaybackThreadTestPlay )
{
    iPlaybackThread->play();
}

}
