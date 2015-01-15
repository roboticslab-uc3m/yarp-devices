// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MOTOR_DRIVER__
#define __MOTOR_DRIVER__

#include <yarp/os/all.h>

#include "ColorDebug.hpp"
#include "CanBusHico.hpp"

using namespace yarp::os;

class MotorDriver  {

    public:

        MotorDriver(CanBusHico *canDevicePtr, const int& canId, const double& tr);

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

        /** start */
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

    protected:

        int canId;

        CanBusHico *canDevicePtr;

        double max, min, refAcceleration, refSpeed, tr;

        double lastUsage;

        double encoder;

        yarp::os::Semaphore encoderReady;


};
 
#endif  // __MOTOR_DRIVER__

