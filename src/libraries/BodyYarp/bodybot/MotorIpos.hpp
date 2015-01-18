// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MOTOR_IPOS__
#define __MOTOR_IPOS__

#include <yarp/os/all.h>
#include <sstream>

#include "ColorDebug.hpp"
#include "CanBusHico.hpp"

using namespace yarp::os;

namespace teo
{

/**
 *
 * @ingroup bodybot
 * @brief Specifies the Technosoft iPOS behaviour and specifications.
 *
 */
class MotorIpos  {

    public:

        MotorIpos(CanBusHico *canDevicePtr, const int& canId, const double& tr);

        int getCanId();
        double getEncoder();  //-- Thread safe.
        double getMax();
        double getMin();
        double getRefAcceleration();
        double getRefSpeed();
        double getTr();

        void setEncoder(const double& value);  //-- Thread safe.
        void setMax(const double& value);
        void setMin(const double& value);
        void setRefAcceleration(const double& value);
        void setRefSpeed(const double& value);
        void setTr(const double& value);

        /**
         * Write message to the CAN buffer.
         * @param cob Message's COB
         * @param len Data field length
         * @param msgData Data to send
         * @return true/false on success/failure.
         */
        bool send(uint32_t cob, uint16_t len, uint8_t * msgData);

        /** "start". All starting functions have to do with:
         * Figure 5.1 Drive’s status machine. States and transitions (p68, 84/263). */
        bool start();
        /** "ready to switch on", also acts as "shutdown" */
        bool readyToSwitchOn();
        /** "switch on", also acts as "disable operation" */
        bool switchOn();
        /** enable */
        bool enable();

        /** pt-related **/
        int ptPointCounter;
        yarp::os::Semaphore ptBuffer;
        bool ptMovementDone;

        bool targetReached;

        bool interpretMessage( can_msg * message);

    protected:

        int canId;

        CanBusHico *canDevicePtr;

        double max, min, refAcceleration, refSpeed, tr;

        double lastUsage;

        double encoder;

        yarp::os::Semaphore encoderReady;

        std::string msgToStr(can_msg* message);

};

}  // namespace teo

#endif  // __MOTOR_IPOS__

