
#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ColorDebug.hpp"

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
            canNodeDevice.close();
            canBusDevice.close();
        }

    protected:
        /** CAN BUS device. */
        yarp::dev::PolyDriver canBusDevice;  //
        CanBusHico* iCanBus;

        /** CAN node object. */
        yarp::dev::PolyDriver canNodeDevice;
        ICanBusSharer* iCanBusSharer;

    };

TEST_F( TechnosoftIposTest, TechnosoftIposOpenCanBus) // -- we call the class that we want to do the test and we assign it a name
{
    yarp::os::Property p("(device CanBusHico) (canDevice /dev/can0) (canBitrate 8)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
    bool ok = true;
    ok &= canBusDevice.open(p);   // -- we introduce the configuration properties defined up and them, we stard the device (HicoCAN)
    ASSERT_EQ(ok, true);    // -- we run the first test
    ok &= canBusDevice.view(iCanBus);
    ASSERT_EQ(ok, true);    // -- we run the first test
}

TEST_F( TechnosoftIposTest, TechnosoftIposOpenCanNode) // -- we call the class that we want to do the test and we assign it a name
{
    yarp::os::Property p("(device TechnosoftIpos) (canId 15) (min -45) (max 70) (tr 160) (refAcceleration 0.575) (refSpeed 5.0)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
    bool ok = true;
    ok &= canNodeDevice.open(p);   // -- we introduce the configuration properties defined up and them, we stard the device (HicoCAN)
    ASSERT_EQ(ok, true);    // -- we run the first test
    ok &= canNodeDevice.view(iCanBusSharer);
    ASSERT_EQ(ok, true);    // -- we run the first test
}

TEST_F( TechnosoftIposTest, TechnosoftIposGetPresence) // -- we call the class that we want to do the test and we assign it a name
{
    struct can_msg buffer;

    int canId = 0;
    int ret = 0;
    //-- Blocking read until we get a message from the expected canId
    while ( canId != 15 )
    {
        while ( ret <= 0 )
        {
            //ret = iCanBus->read_timeout(&buffer,1);
        }
        canId = buffer.id  & 0x7F;
    }
    //-- Assert the message is of "indicating presence" type.
    ASSERT_EQ(buffer.id-canId , 0x700);
}

}  // namespace teo

