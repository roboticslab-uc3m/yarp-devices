
#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ColorDebug.hpp"

#include "ICanBusSharer.h"
#include "CuiAbsolute/CuiAbsolute.hpp"

#define CAN_ID 108 // ID of CuiAbsolute encoder that we want to test

YARP_DECLARE_PLUGINS(BodyYarp)

namespace teo
{

/**
 * @brief Tests \ref KdlSolver ikin and idyn on a simple mechanism.
 */
class CuiAbsoluteTest : public testing::Test // -- inherit the Test class (gtest.h)
{

public:

    virtual void SetUp() {

        // -- Variables
        int id = CAN_ID;


    // -- code here will execute just before the test ensues
        YARP_REGISTER_PLUGINS(BodyYarp);

        yarp::os::Property hicoCanConf ("(device CanBusHico) (canDevice /dev/can1) (canBitrate 8)");
        bool ok = true;
        ok &= canBusDevice.open(hicoCanConf);   // -- we introduce the configuration properties defined in property object (p) and them, we stard the device (HicoCAN)
        ok &= canBusDevice.view(iCanBus);

        if(ok) {
            CD_SUCCESS("Configuration of HicoCAN sucessfully :)\n");
        }
        else {
            CD_ERROR("Bad Configuration of HicoCAN :(\n");
            ::exit(1);
        }

        // ---------- adding configuration of Cui Absolute Encoders (se trata de la configuración minima que necesita el encoder)
        std::stringstream strconf;
        strconf << "(device CuiAbsolute) (canId " << id << ") (min 0) (max 0) (tr 1) (refAcceleration 0.0) (refSpeed 0.0)";
        CD_DEBUG("%s\n",strconf.str().c_str());
        yarp::os::Property CuiAbsoluteConf (strconf.str().c_str());

        ok &= canNodeDevice.open( CuiAbsoluteConf );   // -- we introduce the configuration properties defined ........
        ok &= canNodeDevice.view( iControlLimitsRaw );
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
            CD_SUCCESS("Configuration of TechnosoftIpos sucessfully :)\n");
        }
        else
        {
            CD_ERROR("Bad Configuration of TechnosoftIpos :(\n");
            ::exit(1);
        }

        //-- Pass CAN bus (HicoCAN) pointer to CAN node (TechnosoftIpos).
        iCanBusSharer->setCanBusPtr( iCanBus );
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
    yarp::dev::IControlLimitsRaw* iControlLimitsRaw;
    yarp::dev::IControlModeRaw* iControlModeRaw;
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRaw;
    yarp::dev::IPositionControlRaw* iPositionControlRaw;
    yarp::dev::IPositionDirectRaw* iPositionDirectRaw;
    yarp::dev::ITorqueControlRaw* iTorqueControlRaw;
    yarp::dev::IVelocityControlRaw* iVelocityControlRaw;
    ICanBusSharer* iCanBusSharer;
    CuiAbsolute* cuiAbsolute;

    struct can_msg buffer;

        /** Function definitions **/
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


        TEST_F( CuiAbsoluteTest, CuiAbsoluteSendingMessageInPullMode )
        {

                int canId = 0;
                int ret = 0;
                double timeOut = 2;
                double timeStamp = 0.0;
                bool timePassed = false;

                bool startSending = cuiAbsolute->startPullPublishing();
                timeStamp = yarp::os::Time::now();

                //-- Blocking read until we get a message from the expected canId
                while ( (canId != CAN_ID) && !timePassed )          // -- it will check the ID
                {
                    // -- timer
                    if(int(yarp::os::Time::now()-timeStamp)==timeOut) {
                        CD_ERROR("Time out passed\n");
                        timePassed = true;
                    }

                    ret = iCanBus->read_timeout(&buffer, 0);         // -- return value of message with timeout of 1 [ms]
                    if( ret <= 0 ) continue;                        // -- is waiting for recive message
                    canId = buffer.id  & 0x7F;                      // -- if it recive the message, it will get ID
                    CD_DEBUG("Read ok from CuiAbsolute %d\n", canId);                    
                }

                ASSERT_TRUE(startSending);  // -- testing startPullPublishing function
                ASSERT_FALSE(timePassed);   // -- testing the time (it have to be less than 2 sec)
                ASSERT_EQ(canId , CAN_ID);  // -- testing canId (message received)
        }


        TEST_F( CuiAbsoluteTest, CuiAbsoluteSendingMessageInContinuousMode ) // -- Al tratarse de envío continuo, haremos una comprobación de 3 veces para ver si está enviando mensajes
        {                        

            bool startSending = cuiAbsolute->startContinuousPublishing(0);          // -- manda al PIC una orden de que publique (modo continuo)


            for(int i=1; i<4 ; i++){

                int canId = 0;
                int ret = 0;
                double timeOut = 2; // -- 2 segundos
                double timeStamp = 0.0;                
                bool timePassed = false;

                timeStamp = yarp::os::Time::now();                            // -- tiempo actual

                //-- Blocking read until we get a message from the expected canId

                while ( (canId != CAN_ID) && !timePassed ) // -- it will check the ID
                {
                    ret = iCanBus->read_timeout(&buffer,1);         // -- return value of message with timeout of 1 [ms]
                    if( ret <= 0 ) continue;                        // -- is waiting for recive message
                    canId = buffer.id  & 0x7F;                      // -- if it recive the message, it will get ID
                    CD_DEBUG("Read ok from CuiAbsolute %d (%d)\n", canId, i);
                        if(int(yarp::os::Time::now()-timeStamp)==timeOut) {
                            CD_ERROR("Time out passed\n");
                            timePassed = true;
                        }
                }

                //-- Como se trata de envío continuo, comprobamos varias veces la llegada del mensaje
                ASSERT_TRUE(startSending);  // -- comprobamos startContinuousPublishing
                ASSERT_FALSE(timePassed);   // -- comprobamos que no supera el tiempo de espera
                ASSERT_EQ(canId , CAN_ID);  // -- comprobamos la llegada de un mensaje
                yarp::os::Time::delay(1);
            }
        }


        TEST_F( CuiAbsoluteTest, CuiAbsoluteStopSendingMessage ) // -- we call the class that we want to do the test and we assign it a name
        {

            int canId = 0;
            int ret = 0;
            double timeOut = 1; // -- 1 segundo
            double timeStamp = 0.0;
            bool timePassed = false;


            bool stopSending = cuiAbsolute->stopPublishingMessages();
            yarp::os::Time::delay(1);                                     // -- aplico un delay de 1 segundo para vaciar el buffer

            timeStamp = yarp::os::Time::now();                            // -- tiempo actual


            //-- Blocking read until we get a message from the expected canId
            while ( canId != CAN_ID  && !timePassed ) // -- it will check the ID
            {

                if(int(yarp::os::Time::now()-timeStamp)==timeOut) {
                    CD_INFO("Time out passed and CuiAbsolute stopped successfully\n");
                    timePassed = true;
                }

                ret = iCanBus->read_timeout(&buffer,1);         // -- return value of message with timeout of 1 [ms]
                if( ret <= 0 ) continue;                        // -- is waiting for recive message
                canId = buffer.id  & 0x7F;                      // -- if it recive the message, it will get ID
                CD_DEBUG("Read ok from CuiAbsolute %d\n", canId);

            }

            //-- Como se trata de envío continuo, comprobamos varias veces la llegada del mensaje
            ASSERT_TRUE(stopSending);  // -- comprobamos startContinuousPublishing
            ASSERT_TRUE(timePassed);   // -- comprobamos que supera el tiempo de espera y no recibe ningún mensaje
            ASSERT_NE(canId , CAN_ID); // -- comprobamos que ningún mensaje que le llegue al CAN sea el del encoder absoluto
        }

}
