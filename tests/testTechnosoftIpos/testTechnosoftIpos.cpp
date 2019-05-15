#include "gtest/gtest.h"

#include <cstdlib>

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CanBusInterface.h>

#include <ColorDebug.h>

#include "ICanBusSharer.h"
#include "ITechnosoftIpos.h"

#define CAN_ID 124

namespace roboticslab
{

/**
 * @ingroup yarp_devices_tests
 * @brief Tests \ref TechnosoftIpos on a single CAN node.
 */
class TechnosoftIposTest : public testing::Test // -- inherit the Test class (gtest.h)
{

public:

    virtual void SetUp()
    {
        // -- code here will execute just before the test ensues

        yarp::os::Property canDeviceConf("(device CanBusHico) (canDevice /dev/can1) (canBitrate 1000000)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
        bool ok = true;
        ok &= canBusDevice.open(canDeviceConf);   // -- we introduce the configuration properties defined in property object (p) and them, we stard the device (HicoCAN)
        ok &= canBusDevice.view(iCanBus);
        ok &= canBusDevice.view(iCanBufferFactory);

        if(ok)
        {
            CD_SUCCESS("Configuration of CAN device sucessfully :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration of CAN device :(\n");
            std::exit(1);
        }

        //yarp::os::Property TechnosoftIposConf("(device TechnosoftIpos) (canId 15) (min -45) (max 70) (tr 160) (refAcceleration 0.575) (refSpeed 5.0)"); // -- frontal right arm
        //yarp::os::Property TechnosoftIposConf("(device TechnosoftIpos) (canId 6) (min -90) (max 90) (tr 400) (refAcceleration 0.575) (refSpeed 5.0)"); // -- axial right leg
        //yarp::os::Property TechnosoftIposConf("(device TechnosoftIpos) (canId 2) (min -25) (max 25) (tr 270.4) (refAcceleration 0.575) (refSpeed 5.0)"); // -- frontal right ankle (tobillo)
        yarp::os::Property TechnosoftIposConf("(device TechnosoftIpos) (canId 124) (min -100) (max 10) (tr 160) (refAcceleration 0.575) (refSpeed 5.0)"); // -- frontal left elbow (codo)

        bool ok2 = true;
        ok2 &= canNodeDevice.open( TechnosoftIposConf );   // -- we introduce the configuration properties defined ........
        //j//fail due to yarp non-public inheritance// ok2 &= canNodeDevice.view( iControlLimitsRaw );
        ok2 &= canNodeDevice.view( iControlModeRaw );
        ok2 &= canNodeDevice.view( iEncodersTimedRaw );
        ok2 &= canNodeDevice.view( iPositionControlRaw );
        ok2 &= canNodeDevice.view( iPositionDirectRaw );
        ok2 &= canNodeDevice.view( iTorqueControlRaw );
        ok2 &= canNodeDevice.view( iVelocityControlRaw );
        ok2 &= canNodeDevice.view( iCanBusSharer );
        ok2 &= canNodeDevice.view( technosoftIpos );  //-- ok practice?
        if(ok2)
        {
            CD_SUCCESS("Configuration of TechnosoftIpos sucessfully :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration of TechnosoftIpos :(\n");
            std::exit(1);
        }

        //-- Pass CAN bus pointer to CAN node (TechnosoftIpos).
        iCanBusSharer->setCanBusPtr( iCanBus );

        canInputBuffer = iCanBufferFactory->createBuffer(1);
    }

    virtual void TearDown()
    {
        // -- code here will be called just after the test completes
        // -- ok to through exceptions from here if need be
        iCanBufferFactory->destroyBuffer(canInputBuffer);

        canNodeDevice.close();
        canBusDevice.close();
    }

protected:

    /** CAN BUS device. */
    yarp::dev::PolyDriver canBusDevice;  //
    yarp::dev::ICanBus* iCanBus;
    yarp::dev::ICanBufferFactory* iCanBufferFactory;
    yarp::dev::CanBuffer canInputBuffer;

    /** CAN node object. */
    yarp::dev::PolyDriver canNodeDevice;
    yarp::dev::IControlLimitsRaw* iControlLimits2Raw;
    yarp::dev::IControlModeRaw* iControlModeRaw;
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRaw;
    yarp::dev::IPositionControlRaw* iPositionControlRaw;
    yarp::dev::IPositionDirectRaw* iPositionDirectRaw;
    yarp::dev::ITorqueControlRaw* iTorqueControlRaw;
    yarp::dev::IVelocityControlRaw* iVelocityControlRaw;
    ICanBusSharer* iCanBusSharer; // -- ??
    ITechnosoftIpos* technosoftIpos;

    /** Function definitions **/
    std::string msgToStr(const yarp::dev::CanMessage & message)
    {
        std::stringstream tmp;
        for(int i=0; i < message.getLen()-1; i++)
        {
            tmp << std::hex << static_cast<int>(message.getData()[i]) << " ";
        }
        tmp << std::hex << static_cast<int>(message.getData()[message.getLen()-1]);
        tmp << ". canId(";
        tmp << std::dec << (message.getId() & 0x7F);
        tmp << ") via(";
        tmp << std::hex << (message.getId() & 0xFF80);
        tmp << ").";
        return tmp.str();
    }
};

/*
TEST_F( TechnosoftIposTest, TechnosoftIposGetPresencewithReset) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    int ret = 0;

    // -- doing reset of node after delay
    yarp::os::Time::delay(1);
    technosoftIpos->resetNode(canId);


    //-- Blocking read until we get a message from the expected canId
    CD_INFO("Blocking read until we get a message from the expected canId...\n");

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ){
            //technosoftIpos->resetCommunication();
            //yarp::os::Time::delay(0.3);

            //technosoftIpos->resetNode();
            //yarp::os::Time::delay(10);
            continue;
        }
        // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
        //CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());
    }
    //-- Assert the message is of "indicating presence" type.
    ASSERT_EQ(buffer.id-canId , 0x700);
}
*/

/*
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
*/
/************************************************************************************
 ************** Test of seting initial parameters on physical motor drivers *********
 ************************************************************************************/
// -- Set Ref Aceleration Raw
// idea: getControlMode (Juan)

TEST_F( TechnosoftIposTest, TechnosoftIposSetRefAccelerationRaw )
{
    int canId = 0;
    int ret = 0;

    yarp::os::Time::delay(1);
    //-- Set initial parameter on physical motor driver.
    // --( implemented in Technosoftipos->IPositionControlRawImpl.cpp)
    bool ok = iPositionControlRaw->setRefAccelerationRaw( 0, 0.575 );  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    yarp::dev::CanMessage &msg = canInputBuffer[0];

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        unsigned int read;
        bool ok = iCanBus->canRead(canInputBuffer, 1, &read, true);
        if( !ok || read == 0 ) continue;    // -- is waiting for recive message
        canId = msg.getId()  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(msg).c_str());

    //-- Print interpretation of message
    iCanBusSharer->interpretMessage(msg); // without ASSERT (we don't need assert interpretMessage function)

    // Manual 8.2.3. 6083h: Profile acceleration (SDO ack \"posmode_acc\" from driver)
    ASSERT_EQ(msg.getData()[0] , 0x60);  //-- ??
    ASSERT_EQ(msg.getData()[1] , 0x83);  //-- 83
    ASSERT_EQ(msg.getData()[2] , 0x60);  //-- 60


}

/*


// -- Set Ref Speed Raw

TEST_F( TechnosoftIposTest, TechnosoftIposSetRefSpeedRaw )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = iPositionControlRaw->setRefSpeedRaw( 0, 5.0 );  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    // Manual 8.2.2. 6083h: Profile velocity (SDO ack \"posmode_acc\" from driver)
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x81);  //-- 81
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)
}

// -- Set Minimum Limits Raw
TEST_F( TechnosoftIposTest, TechnosoftIposSetMinLimitsRaw )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->setMinLimitRaw(-45);  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    // Manual 8.2.2. 6083h: Profile  (SDO ack \"posmode_acc\" from driver)
    // Index: 607D
    // Subindex:    (0)-Number of entries
    //              (1)-Minimal position limit
    //              (2)-Maximal position limit
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x7d);  //-- 76
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
    ASSERT_EQ(buffer.data[3] , 0x01);  //-- 01 -> min

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)
}

// -- Set Maximum Limits Raw
TEST_F( TechnosoftIposTest, TechnosoftIposSetMaxLimitsRaw )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->setMaxLimitRaw(70);  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );
    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    // Manual 8.2.2. 6083h: Profile  (SDO ack \"posmode_acc\" from driver)
    // Index: 607D
    // Subindex:    (0)-Number of entries
    //              (1)-Minimal position limit
    //              (2)-Maximal position limit
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x7d);  //-- 76
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
    ASSERT_EQ(buffer.data[3] , 0x02);  //-- 02 -> max

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)
}
*/

/************************************************************************************
 **************** Test of seting all motor drivers to differents mode ***************
 ************************************************************************************/
/*
//-- Set Velocity Mode

TEST_F( TechnosoftIposTest, TechnosoftIposSetVelocityMode )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok =  iControlModeRaw->setVelocityModeRaw(0); //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    // -- Manual 5.2.5. Object 6060h/6061h: Modes of Operation Display (SDO ack \"modes of operation display\" from driver)
    //  The object reflects the actual mode of operation set with object Modes of Operation (index 6060 h )
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)

}

// -- Set Torque Mode
TEST_F( TechnosoftIposTest, TechnosoftIposSetTorqueMode )
{
    int canId = 0;
    int ret = 0;
    bool ok = true;

    //-- Segment (1) of setTorqueMode
    ok &=  technosoftIpos->setTorqueModeRaw1(); //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    // Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)
    // ASSERTS of message 1
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x1d);  //-- 1d
    ASSERT_EQ(buffer.data[2] , 0x20);  //-- 20

    //-- Segment (2) of setTorqueMode
    ok &=  technosoftIpos->setTorqueModeRaw2(); //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    // Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)
    // ASSERTS of message 2
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x1d);  //-- 1d
    ASSERT_EQ(buffer.data[2] , 0x20);  //-- 20

    //-- Segment (3) of setTorqueMode
    ok &=  technosoftIpos->setTorqueModeRaw3(); //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)

    // -- Manual 11.2.6. Object 201Dh: External Reference Type
    //    This object is used to set the type of external reference for use with electronic gearing position,
    //    electronic camming position, position external, speed external and torque external modes.
    // ASSERTS of message 3
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x1d);  //-- 1d
    ASSERT_EQ(buffer.data[2] , 0x20);  //-- 20
}

// -- Set Position Mode
TEST_F( TechnosoftIposTest, TechnosoftIposSetPositionMode )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok =  iControlModeRaw->setPositionModeRaw(0); //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer); // without ASSERT (we don't need assert interpretMessage function)

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}
*/
/*************************************************************************************
 ****************************** Get Controls Mode Raw  ********************************
 ************************************************************************************/
/*
// -- get control mode raw [1]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_1_1 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw1() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}


// -- get control mode raw [2]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_1_2 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw2() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    ASSERT_EQ(buffer.data[0] , 0x4f);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x61);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [3]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_1_3 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw3() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [4]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_1_4 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw4() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}
*/
/*************************************************************************************
 **************************** Initialization test drivers ****************************
 ************************************************************************************/

//-- Check the status of each driver.
//std::vector<int> tmp( nodes.size() ); // -- creating a "tmp"vector with "nodes" vector size
//this->getControlModes( tmp.data() );

//-- Test of: START REMOTE NODE
/*
TEST_F( TechnosoftIposTest, TechnosoftIposStart) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    int ret = 0;

    bool ok = iCanBusSharer->start();   // -- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_INFO_NO_HEADER("\n~~~~~~~~~~~~~~~~~~ TechnosoftIposStart ~~~~~~~~~~~~~~~~~~~~ \n\n");
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Got PDO1 that it is observed as TRANSITION performed upon \"start\"
    //ASSERT_EQ(buffer.data[0] , 0x40);
    //ASSERT_EQ(buffer.data[1] , 0x02);
}
*/
/*************************************************************************************
 ****************************** Get Controls Mode Raw  ********************************
 ************************************************************************************/
/*
// -- get control mode raw [1]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_2_1 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw1() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [2]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_2_2 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw2() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    ASSERT_EQ(buffer.data[0] , 0x4f);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x61);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [3]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_2_3 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw3() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [4]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_2_4 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw4() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}


//-- Test of READY TO SWITCH ON REMOTE NODE (with delay 0.1)

TEST_F( TechnosoftIposTest, TechnosoftIposReadyToSwitchOn) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    int ret = 0;

    yarp::os::Time::delay(0.1);         // -- delay 100 [ms]
    bool ok = iCanBusSharer->readyToSwitchOn();  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_INFO_NO_HEADER("\n~~~~~~~~~~~~~~~~~~ TechnosoftIposReadyToSwitchOn ~~~~~~~~~~~~~~~~~~~~ \n\n");

    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // --
    //ASSERT_EQ(buffer.data[0] , 0x40);
    //ASSERT_EQ(buffer.data[1] , 0x02);
}
*/
/*************************************************************************************
 ****************************** Get Controls Mode Raw  ********************************
 ************************************************************************************/
/*
// -- get control mode raw [1]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_3_1 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw1() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [2]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_3_2 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw2() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    ASSERT_EQ(buffer.data[0] , 0x4f);  //-- ??
    ASSERT_EQ(buffer.data[1] , 0x61);  //-- 60
    ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [3]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_3_3 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw3() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [4]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_3_4 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw4() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

//-- Test of SWITCH ON REMOTE NODE (with delay 0.1)
TEST_F( TechnosoftIposTest, TechnosoftIposSwitchOn) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    int ret = 0;

    yarp::os::Time::delay(0.1);                     // -- delay 100 [ms]
    bool ok = iCanBusSharer->switchOn();     //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_INFO_NO_HEADER("\n~~~~~~~~~~~~~~~~~~ TechnosoftIposSwitchOn ~~~~~~~~~~~~~~~~~~~~ \n\n");

    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // --
    //ASSERT_EQ(buffer.data[0] , 0x40);
    //ASSERT_EQ(buffer.data[1] , 0x02);
}
*/
/*************************************************************************************
 ****************************** Get Controls Mode Raw  ********************************
 ************************************************************************************/
/*
// -- get control mode raw [1]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_4_1 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw1() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [2]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_4_2 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw2() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x4f);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x61);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [3]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_4_3 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw3() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [4]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_4_4 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw4() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}


//-- Test of ENABLE REMOTE NODE (with delay 0.2)
TEST_F( TechnosoftIposTest, TechnosoftIposEnable) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    int ret = 0;

    yarp::os::Time::delay(0.2);         // -- delay 200 [ms]
    bool ok = iCanBusSharer->enable();  //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_INFO_NO_HEADER("\n~~~~~~~~~~~~~~~~~~ TechnosoftIposiposEnable ~~~~~~~~~~~~~~~~~~~~ \n\n");

    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // --
    //ASSERT_EQ(buffer.data[0] , 0x40);
    //ASSERT_EQ(buffer.data[1] , 0x02);
}
*/

/*************************************************************************************
 ****************************** Get Controls Mode Raw  ********************************
 ************************************************************************************/
/*
// -- get control mode raw [1]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_5_1 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw1() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [2]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_5_2 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw2() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x4f);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x61);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [3]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_5_3 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw3() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual ??
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}

// -- get control mode raw [4]
TEST_F( TechnosoftIposTest, TechnosoftIposGetControlModeRaw_5_4 )
{
    int canId = 0;
    int ret = 0;

    //-- Set initial parameter on physical motor driver.
    bool ok = technosoftIpos->getControlModeRaw4() ; //-- ok corresponds to send (not read)
    ASSERT_TRUE( ok );

    while ( canId != CAN_ID ) // -- it will check the ID
    {
        ret = iCanBus->read_timeout(&buffer,1); // -- return value of message with timeout of 1 [ms]
        if( ret <= 0 ) continue;    // -- is waiting for recive message
        canId = buffer.id  & 0x7F;  // -- if it recive the message, it will get ID
    }
    CD_DEBUG("Read: %s\n", msgToStr(&buffer).c_str());

    //-- Print interpretation of message
    technosoftIpos->interpretMessage(&buffer);

    // -- Manual 5.2.4. Object 6060h: Modes of Operation (SDO ack \"posmode_acc\" from driver)
    //ASSERT_EQ(buffer.data[0] , 0x60);  //-- ??
    //ASSERT_EQ(buffer.data[1] , 0x60);  //-- 60
    //ASSERT_EQ(buffer.data[2] , 0x60);  //-- 60
}
*/
}  // namespace roboticslab
