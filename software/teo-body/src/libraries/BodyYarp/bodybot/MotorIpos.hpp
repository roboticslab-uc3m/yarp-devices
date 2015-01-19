// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MOTOR_IPOS__
#define __MOTOR_IPOS__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

#include "ColorDebug.hpp"
#include "CanBusHico.hpp"

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 *
 * @ingroup bodybot
 * @brief Specifies the Technosoft iPOS behaviour and specifications.
 *
 */
// Note: IEncodersTimed inherits from IEncoders
class MotorIpos : public IPositionControlRaw, public IEncodersTimedRaw  {

    public:

        MotorIpos(CanBusHico *canDevicePtr, const int& canId, const double& tr);
        ~MotorIpos(){}

        int getCanId();
        double getMax();
        double getMin();
        double getTr();

        void setMax(const double& value);
        void setMin(const double& value);
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

        //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
        virtual bool setLimitsRaw(int axis, double min, double max);
        virtual bool getLimitsRaw(int axis, double *min, double *max);

        //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
        virtual bool setPositionModeRaw(int j);
        virtual bool setVelocityModeRaw(int j);
        virtual bool setTorqueModeRaw(int j);
        virtual bool setImpedancePositionModeRaw(int j);
        virtual bool setImpedanceVelocityModeRaw(int j);
        virtual bool setOpenLoopModeRaw(int j);
        virtual bool getControlModeRaw(int j, int *mode);
        virtual bool getControlModes(int *modes){
            CD_ERROR("\n");
            return false;
        }

        // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------
        virtual bool getAxes(int *ax){
            *ax = 1;
            return true;
        }
        virtual bool setPositionModeRaw() {
            CD_ERROR("\n");
            return false;
        }
        virtual bool positionMoveRaw(int j, double ref);
        virtual bool positionMoveRaw(const double *refs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool relativeMoveRaw(int j, double delta);
        virtual bool relativeMoveRaw(const double *deltas) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool checkMotionDoneRaw(int j, bool *flag);
        virtual bool checkMotionDoneRaw(bool *flag) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setRefSpeedRaw(int j, double sp);
        virtual bool setRefSpeedsRaw(const double *spds) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setRefAccelerationRaw(int j, double acc);
        virtual bool setRefAccelerationsRaw(const double *accs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getRefSpeedRaw(int j, double *ref);
        virtual bool getRefSpeedsRaw(double *spds) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getRefAccelerationRaw(int j, double *acc);
        virtual bool getRefAccelerationsRaw(double *accs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool stopRaw(int j);
        virtual bool stopRaw() {
            CD_ERROR("\n");
            return false;
        }

        //  ---------- IEncodersRaw Declarations. Implementation in IEncodersRawImpl.cpp ----------
        virtual bool resetEncoderRaw(int j);
        virtual bool resetEncodersRaw() {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setEncoderRaw(int j, double val);
        virtual bool setEncodersRaw(const double *vals) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderRaw(int j, double *v);
        virtual bool getEncodersRaw(double *encs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderSpeedRaw(int j, double *sp);
        virtual bool getEncoderSpeedsRaw(double *spds) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderAccelerationRaw(int j, double *spds);
        virtual bool getEncoderAccelerationsRaw(double *accs) {
            CD_ERROR("\n");
            return false;
        }

        //  ---------- IEncodersTimedRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------

        virtual bool getEncodersTimedRaw(double *encs, double *time) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderTimedRaw(int j, double *encs, double *time);

    protected:

        int canId;

        CanBusHico *canDevicePtr;

        double max, min, refAcceleration, refSpeed, tr;

        double lastUsage;

        double encoder;

        yarp::os::Semaphore encoderReady;

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message);
        std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

};

}  // namespace teo

#endif  // __MOTOR_IPOS__

