
#include "gtest/gtest.h" // -- We load the librarie of GoogleTest

// -- We load the rest of libraries that we will use to call the functions of our code
#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ColorDebug.hpp"

#include "ICanBusSharer.h"
#include "TechnosoftIpos.hpp"  //-- ok practice?

#define CAN_ID 24

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
    // -- Variables
    int id;


    // -- code here will execute just before the test ensues
        YARP_REGISTER_PLUGINS(BodyYarp);

        yarp::os::Property hicoCanConf ("(device CanBusHico) (canDevice /dev/can1) (canBitrate 8)"); // -- truco para agregar directamente un conjunto de propiedades sin tener que llamar a la función "put"
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
        ok &= canNodeDevice.view( technosoftIpos );  //-- ok practice?

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
    ICanBusSharer* iCanBusSharer; // -- ??
    TechnosoftIpos* technosoftIpos;    //-- ok practice?

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
}
