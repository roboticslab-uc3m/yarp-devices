// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_SR_PORT_HPP__
#define __IN_SR_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include "InCvPort.hpp"

#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')


namespace teo
{

/**
 * @ingroup executionCore1
 *
 * @brief Input port of speech recognition data.
 *
 */
class InSrPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
    public:
        void setInCvPortPtr(InCvPort *inCvPortPtr) {
            this->inCvPortPtr = inCvPortPtr;
        }

    protected:
        /** Callback on incoming Bottle. **/
        virtual void onRead(yarp::os::Bottle& b);

        InCvPort* inCvPortPtr;
};

}  // namespace teo

#endif  // __IN_SR_PORT_HPP__
