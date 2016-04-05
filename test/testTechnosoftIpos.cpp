#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ColorDebug.hpp"

YARP_DECLARE_PLUGINS(BodyYarp)

namespace teo
{

/**
 * @brief Tests \ref KdlSolver ikin and idyn on a simple mechanism.
 */
class TechnosoftIposTest : public testing::Test
{

    public:
        virtual void SetUp() {
            YARP_REGISTER_PLUGINS(BodyYarp);

            /*if( ! dd.view(iCartesianSolver) ) {
                CD_ERROR("Could not view ICartesianSolver.\n");
                return;
            }*/
        }

        virtual void TearDown()
        {
            dd.close();
        }

    protected:
        yarp::dev::PolyDriver dd;
        //teo::ICartesianSolver *iCartesianSolver;
};

TEST_F( TechnosoftIposTest, TechnosoftIposTest1)
{
    yarp::os::Property p("(device CanBusHico) (canDevice /dev/can0) (canBitrate 8)");
    bool ok = dd.open(p);
    ASSERT_EQ(ok, true);
}

}  // namespace teo

