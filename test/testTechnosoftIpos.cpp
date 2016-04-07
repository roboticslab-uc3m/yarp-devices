
#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ColorDebug.hpp"

#include "ICanBusSharer.h"

#define CAN_ID 15

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

        yarp::os::Property hicoCanConf ("(device CanBusHico) (canDevice /dev/can0) (canBitrate 8)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
        bool ok = true;
        ok &= canBusDevice.open(hicoCanConf);   // -- we introduce the configuration properties defined in property object (p) and them, we stard the device (HicoCAN)
        ok &= canBusDevice.view(iCanBus);
        if(ok)
        {
            CD_SUCCESS("Configuration of HicoCAN sucessfully :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration of HicoCAN :(\n");
            ::exit(1);
        }

        yarp::os::Property TechnosoftIposConf("(device TechnosoftIpos) (canId 15) (min -45) (max 70) (tr 160) (refAcceleration 0.575) (refSpeed 5.0)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
        bool ok2 = true;
        ok2 &= canNodeDevice.open(TechnosoftIposConf);   // -- we introduce the configuration properties defined ........
        ok2 &= canNodeDevice.view(iCanBusSharer);
        if(ok2)
        {
            CD_SUCCESS("Configuration of TechnosoftIpos sucessfully :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration of TechnosoftIpos :(\n");
            ::exit(1);
        }

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
    ICanBusSharer* iCanBusSharer; // -- ??

    struct can_msg buffer;

    std::string msgToStr(can_msg* message)
    {
        std::stringstream tmp;
        for(int i=0; i < message->dlc-1; i++)
        {
            tmp << std::hex << static_cast<int>(message->data[i]) << " ";
        }
        tmp << std::hex << static_cast<int>(message->data[message->dlc-1]);
        tmp << ". canId(";
        tmp << std::dec << (message->id & 0x7F);
        tmp << ") via(";
        tmp << std::hex << (message->id & 0xFF80);
        tmp << ").";
        return tmp.str();
    }

};

TEST_F( TechnosoftIposTest, TechnosoftIposGetPresence) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    int ret = 0;
    //-- Blocking read until we get a message from the expected canId
    CD_INFO("Blocking read until we get a message from the expected canId...\n");

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
        //CD_DEBUG("Read ok from %d\n", canId);
    }
    //-- Assert the message is of "indicating presence" type.
    ASSERT_EQ(buffer.id-canId , 0x700);
}

// idea: getControlMode

TEST_F( TechnosoftIposTest, TechnosoftIposStart) // -- we call the class that we want to do the test and we assign it a name
{
    //-- Pass CAN bus (HicoCAN) pointer to CAN node (TechnosoftIpos).
    iCanBusSharer->setCanBusPtr( iCanBus );

    int canId = 0;
    int ret = 0;
    
    bool ok = iCanBusSharer->start();  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    ASSERT_EQ(buffer.data[0] , 0x40);
    ASSERT_EQ(buffer.data[1] , 0x02);
}

}  // namespace teo

