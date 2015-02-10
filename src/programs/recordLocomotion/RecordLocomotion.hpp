// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RECORD_MANIPULATION__
#define __RECORD_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>
#include <stdlib.h>  // Used for ::getenv

#include "ColorDebug.hpp"

#include "RecordRateThread.hpp"

#define DEFAULT_PT_MODE_MS 50
#define DEFAULT_FILE_NAME "locomotionRecording.txt"

/**
 * @ingroup recordLocomotion
 *
 * The RecordLocomotion class tests the class as a controlboard.
 * 
 */
class RecordLocomotion : public yarp::os::RFModule {

    public:
        RecordLocomotion();
        bool configure(yarp::os::ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver leftLegDevice;
        yarp::dev::PolyDriver rightLegDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        FILE * filePtr;

        RecordRateThread recordRateThread;
};

#endif  // __RECORD_MANIPULATION__

