
#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

//#include "ColorDebug.hpp"

#include "ICanBusSharer.h"

YARP_DECLARE_PLUGINS(BodyYarp)

namespace teo
{

/**
 * @brief Tests \ref KdlSolver ikin and idyn on a simple mechanism.
 */
class TechnosoftIposTest : public testing::Test // -- inherit the Test class (gtest.h)
{

    public:
        virtual void SetUp() {
        // -- code here will execute just before the test ensues
            YARP_REGISTER_PLUGINS(BodyYarp);
        }

        virtual void TearDown()
        {
        // -- code here will be called just after the test completes
        // -- ok to through exceptions from here if need be
            canBusDevice.close();
        }

    protected:
        /** A CAN device. */
        yarp::dev::PolyDriver canBusDevice;  //
        CanBusHico* iCanBus;
    };

TEST_F( TechnosoftIposTest, TechnosoftIposTest1) // -- we call the class that we want to do the test and we assign it a name
{
    yarp::os::Property p("(device CanBusHico) (canDevice /dev/can0) (canBitrate 8)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
    bool ok = canBusDevice.open(p);   // -- we introduce the configuration properties defined up and them, we stard the device (HicoCAN)
    ASSERT_EQ(ok, true);    // -- we run the first test
}

}  // namespace teo

