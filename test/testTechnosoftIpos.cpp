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

            yarp::os::Property p("(device CanBusHico) (canDevice /dev/can0) (canBitrate 8)");

            dd.open(p);
            if( ! dd.isValid() ) {
                CD_ERROR("CAN device not valid.\n");
                return;
            }
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
    ASSERT_EQ(1, 1);
}

}  // namespace teo

