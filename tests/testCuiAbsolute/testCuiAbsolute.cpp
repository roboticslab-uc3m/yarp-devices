#include "gtest/gtest.h"

#include <cstdlib>
#include <ios>
#include <string>
#include <sstream>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ColorDebug.h"

#include "ICanBusSharer.h"
#include "ICuiAbsolute.h"

#define CAN_ID 126 // ID of Cui Absolute encoder that you want to check...

namespace roboticslab
{

/**
 * @ingroup yarp_devices_tests
 * @brief Tests \ref CuiAbsolute on a single CAN node.
 */
class CuiAbsoluteTest : public testing::Test // -- inherit the Test class (gtest.h)
{

public:

    virtual void SetUp()
    {
        // -- code here will execute just before the test ensues

        yarp::os::Property canDeviceConf("(device CanBusControlboard) (canDevice /dev/pcan3) (canBitrate 1000000)");
        bool ok = true;
        ok &= canBusDevice.open(canDeviceConf);   // -- we introduce the configuration properties defined in property object (p) and them, we stard the device (CanBusPeak)
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

        // -- adding configuration of Cui Absolute Encoders (minimal configuration that CuiAbsolute need to run correctly)
        std::stringstream strconf;
        strconf << "(device CuiAbsolute) (canId " << CAN_ID << ") (min 0) (max 0) (tr 1) (refAcceleration 0.0) (refSpeed 0.0)";
        CD_DEBUG("%s\n",strconf.str().c_str());
        yarp::os::Property CuiAbsoluteConf (strconf.str().c_str());

        ok &= canNodeDevice.open( CuiAbsoluteConf );   // -- we introduce the configuration properties defined ........
        if ( ! canNodeDevice.isValid() )
        {
            CD_ERROR("Bad device of CuiAbsolute :(\n");
            std::exit(1);
        }
        ok &= canNodeDevice.view( iControlLimits2Raw );
        ok &= canNodeDevice.view( iControlModeRaw );
        ok &= canNodeDevice.view( iEncodersTimedRaw );
        ok &= canNodeDevice.view( iPositionControlRaw );
        ok &= canNodeDevice.view( iPositionDirectRaw );
        ok &= canNodeDevice.view( iTorqueControlRaw );
        ok &= canNodeDevice.view( iVelocityControlRaw );
        ok &= canNodeDevice.view( iCanBusSharer );
        ok &= canNodeDevice.view( cuiAbsolute );

        if(ok)
        {
            CD_SUCCESS("Configuration of CuiAbsolute sucessfully :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration of CuiAbsolute :(\n");
            std::exit(1);
        }

        //-- Pass CAN bus pointer to CAN node.
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
    yarp::dev::PolyDriver canBusDevice;
    yarp::dev::ICanBus* iCanBus;
    yarp::dev::ICanBufferFactory* iCanBufferFactory;
    yarp::dev::CanBuffer canInputBuffer;

    /** CAN node object. */
    yarp::dev::PolyDriver canNodeDevice;
    yarp::dev::IControlLimits2Raw* iControlLimits2Raw;
    yarp::dev::IControlModeRaw* iControlModeRaw;
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRaw;
    yarp::dev::IPositionControlRaw* iPositionControlRaw;
    yarp::dev::IPositionDirectRaw* iPositionDirectRaw;
    yarp::dev::ITorqueControlRaw* iTorqueControlRaw;
    yarp::dev::IVelocityControlRaw* iVelocityControlRaw;
    ICanBusSharer* iCanBusSharer;
    ICuiAbsolute* cuiAbsolute;

    /** Function definitions **/
    std::string msgToStr(const yarp::dev::CanMessage& message)
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


TEST_F( CuiAbsoluteTest, CuiAbsoluteSendingMessageInPullMode )
{

    int canId = 0;
    double timeOut = 2;
    double timeStamp = 0.0;
    bool timePassed = false;
    double cleaningTime = 0.5; // time to empty the buffer
    std::map< int, int > idxFromCanId;

    bool startSending = cuiAbsolute->startPullPublishing();
    timeStamp = yarp::os::Time::now();

    yarp::dev::CanMessage &msg = canInputBuffer[0];

    //-- Blocking read until we get a message from the expected canId
    while ( (canId != CAN_ID) && !timePassed )          // -- it will check the ID (poner condición nAn)
    {
            // -- timer
            if(int(yarp::os::Time::now()-timeStamp)==timeOut)
            {
                CD_ERROR("Time out exceeded\n");
                timePassed = true;
                //continue;
            }

            unsigned int read;
            bool ok = iCanBus->canRead(canInputBuffer, 1, &read, true);

            // This line is needed to clear the buffer (old messages that has been received)
            // if((yarp::os::Time::now()-timeStamp) < cleaningTime) continue;

            if( !ok || read == 0 ) continue;                 // -- is waiting for recive message
            canId = msg.getId() & 0x7F;                      // -- if it recive the message, it will get ID


            if (canId == CAN_ID) {
                 // -- Reading Cui Absolute Encoder Value
                iCanBusSharer->interpretMessage(msg); // necessary for read CuiAbsolute
                double value;
                while( ! iEncodersTimedRaw->getEncoderRaw(0,&value) ){
                    CD_ERROR("Wrong value of Cui \n");
                }
                CD_DEBUG("Reading in pull mode from CuiAbsolute %d (Value: %f)\n", canId, value);                
            }
    }

    ASSERT_TRUE(startSending);         // -- testing startPullPublishing function
    ASSERT_FALSE(timePassed);          // -- testing the time (it have to be less than 2 sec (timeOut) )
    ASSERT_EQ(canId , CAN_ID);  // -- testing if ID of the CUI is the same that it has received
}


TEST_F( CuiAbsoluteTest, CuiAbsoluteSendingMessageInContinuousMode )
{
    bool startSending = cuiAbsolute->startContinuousPublishing(0);

    yarp::dev::CanMessage &msg = canInputBuffer[0];

    // -- In continuous mode, we are goint to do three test to ensure that we are receiving multiple messages
    for(int i=1; i<4 ; i++)
    {

        int canId = 0;
        double timeOut = 5; // -- 2 seconds
        double timeStamp = 0.0;
        double cleaningTime = 0.5; // time to empty the buffer
        bool timePassed = false;

        timeStamp = yarp::os::Time::now();

        //-- Blocking read until we get a message from the expected canId

        while ( (canId != CAN_ID) && !timePassed ) // -- it will check the ID
        {
            // -- if it exceeds the timeout...NOT PASS the test
            if(int(yarp::os::Time::now()-timeStamp)==timeOut)
            {
                CD_ERROR("Time out exceeded\n");
                timePassed = true;
            }

            unsigned int read;
            bool ok = iCanBus->canRead(canInputBuffer, 1, &read, true);

            // This line is needed to clear the buffer (old messages that has been received)
            if((yarp::os::Time::now()-timeStamp) < cleaningTime) continue;

            if( !ok || read == 0 ) continue;                 // -- is waiting for recive message
            canId = msg.getId() & 0x7F;                      // -- if it recive the message, it will get ID
            if (canId == CAN_ID)
            {
                // -- Reading Cui Absolute Encoder Value
                iCanBusSharer->interpretMessage(msg); // necessary for read CuiAbsolute
                double value = 0;
                while( ! iEncodersTimedRaw->getEncoderRaw(0,&value) ){
                    CD_ERROR("Wrong value of Cui \n");
                }
                CD_DEBUG("Reading in continuous mode from CuiAbsolute %d (Value: %f) [check %d]\n", canId, value, i);
            }
        }

        ASSERT_TRUE(startSending);  // -- testing startContinuousPublishing function
        ASSERT_FALSE(timePassed);   // -- testing the time (it have to be less than 2 sec)
        ASSERT_EQ(canId , CAN_ID);  // -- testing if the CAN ID of CUI is the same that it has received (3 tests)
        yarp::os::Time::delay(1);
    }
}


TEST_F( CuiAbsoluteTest, CuiAbsoluteStopSendingMessage ) // -- we call the class that we want to do the test and we assign it a name
{
    int canId = 0;
    double timeOut = 1;
    double timeStamp = 0.0;
    double cleaningTime = 0.5; // time to empty the buffer
    bool timePassed = false;

    bool stopSending = cuiAbsolute->stopPublishingMessages();
    yarp::os::Time::delay(1);                                     // -- one second delay to empty the buffer

    timeStamp = yarp::os::Time::now();

    yarp::dev::CanMessage &msg = canInputBuffer[0];

    //-- Blocking read until we get a message from the expected canId
    while ( (canId != CAN_ID) && !timePassed ) // -- it will check the ID
    {
        //printf("timeOut: %d\n", int(yarp::os::Time::now()-timeStamp));

        // -- if it exceeds the timeout (1 secod) ...PASS the test
        if(int(yarp::os::Time::now()-timeStamp)==timeOut)
        {
            CD_INFO("Time out passed and CuiAbsolute stopped successfully\n");
            timePassed = true;
        }

        unsigned int read;
        bool ok = iCanBus->canRead(canInputBuffer, 1, &read, true);

        // This line is needed to clear the buffer (old messages that has been received)
        if((yarp::os::Time::now()-timeStamp) < cleaningTime) continue;

        if( !ok || read == 0 ) continue;                 // -- is waiting for recive message
        canId = msg.getId() & 0x7F;                      // -- if it recive the message, it will get ID
        CD_DEBUG("Read a message from CuiAbsolute %d\n", canId);

    }

    ASSERT_TRUE(stopSending);  // -- testing stopPublishingMessages function
    ASSERT_TRUE(timePassed);   // -- testing the time (if it exceeds the timeout (1 secod) ...it will PASS the test)
    ASSERT_NE(canId , CAN_ID); // -- testing if the CAN ID of CUI is NOT the same that it has received

}

}
