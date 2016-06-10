// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEST_CUI_ABSOLUTE__
#define __TEST_CUI_ABSOLUTE__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <stdlib.h>

#include "ICanBusSharer.h"
#include "ColorDebug.hpp"

#include "TechnosoftIpos/TechnosoftIpos.hpp"    // -- uso de librería de drivers
#include "CuiAbsolute/CuiAbsolute.hpp"          // -- uso de librería de encoders absolutos


namespace teo
{

/**
 * @ingroup testCuiAbsolute
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */

class TestCuiAbsolute : public yarp::os::RFModule {
    public:
        TestCuiAbsolute();
        bool configure(yarp::os::ResourceFinder &rf);

        // -- Nuevas variables:        
        double firstTime;  // -- tiempo en el arranque (valor de tiempo aleatorio)
        double cleaningTime; // -- tiempo de espera para que no lleguen mensajes "basura" de encoders absolutos
        int id;              // -- id del encoder al que vamos a mandar mensaje        

    protected:

        /** CAN BUS device. */
        yarp::dev::PolyDriver deviceDevCan0; // -- Dispositivo (HicoCan) que se crea.
        CanBusHico* iCanBus;

        /** CAN node object (canNodeDevice): CuiAbsolute */
            yarp::dev::PolyDriver canNodeCuiAbsolute;
            yarp::dev::IControlLimitsRaw* iControlLimitsRaw;
            yarp::dev::IControlModeRaw* iControlModeRaw;
            yarp::dev::IEncodersTimedRaw* iEncodersTimedRaw;
            yarp::dev::IPositionControlRaw* iPositionControlRaw;
            yarp::dev::IPositionDirectRaw* iPositionDirectRaw;
            yarp::dev::ITorqueControlRaw* iTorqueControlRaw;
            yarp::dev::IVelocityControlRaw* iVelocityControlRaw;
            ICanBusSharer* iCanBusSharer;
            CuiAbsolute* cuiAbsoluteEncoder;

        /** Functions **/
        virtual double getPeriod() {return 3.0;}  // Periodicidad de llamada a updateModule en [s]
        virtual bool updateModule();
        virtual bool close();

};

}  // namespace teo

#endif  // __TEST_CUI_ABSOLUTE__

