// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RECORD_RATE_THREAD__
#define __RECORD_RATE_THREAD__

#include <vector>

#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#define DEFAULT_MS 50  // [ms], overwritten by parent DEFAULT_PT_MODE_MS.

namespace teo
{

/**
 * @ingroup recordManipulation
 *
 * @brief WARNING repeated class (you are seeing the recordManipulation version).
 *
 */
class RecordRateThread : public yarp::os::RateThread {

    public:
        // Set the Thread Rate in the class constructor
        RecordRateThread() : RateThread(DEFAULT_MS) {}  // In ms

        /**
         * Loop function. This is the thread itself.
         */
        virtual void run();

        void setFilePtr(FILE *value);

        yarp::dev::IEncoders *leftArmEnc;
        yarp::dev::ITorqueControl *leftArmTrq;

        yarp::dev::IEncoders *rightArmEnc;
        yarp::dev::ITorqueControl *rightArmTrq;

        int leftArmNumMotors;
        int rightArmNumMotors;

    protected:

        FILE * filePtr;


};

}  // namespace teo

#endif  // __RECORD_RATE_THREAD__

