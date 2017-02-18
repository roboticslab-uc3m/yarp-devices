#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ColorDebug.hpp"

#include "Playback.hpp"

namespace teo
{

/**
* @brief Tests \ref Playback
*/
class PlaybackTest : public testing::Test // -- inherit the Test class (gtest.h)
{

public:

    virtual void SetUp()
    {

        // -- code here will execute just before the test ensues

        yarp::os::Property playbackConf ("(device Playback) (file test.txt)");
        bool ok = true;
        ok &= playback.open(playbackConf);   // -- we introduce the configuration properties defined in property object (p) and them, we stard the device (HicoCAN)

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
        playback.close();
    }

protected:

    /** Playback. */
    Playback playback;

};

TEST_F( PlaybackTest, PlaybackTestGetNumRows )
{
    int num;
    ASSERT_TRUE( playback.getNumRows( &num ) );
    std::cout << "Num rows: " << num << std::endl;
}

TEST_F( PlaybackTest, PlaybackTestGetNext )
{
    std::vector<double> row;

    int rowCounter = 0;
    while( playback.getNext(row) )
    {
        std::cout << "Row[" << rowCounter << "]: ";
        for(int i=0;i<row.size();i++)
        {
            std::cout << row[i] << " ";
        }
        std::cout << std::endl;
        rowCounter++;
    }
}

}
