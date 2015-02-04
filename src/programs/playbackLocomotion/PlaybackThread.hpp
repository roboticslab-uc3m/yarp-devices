// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK_THREAD__
#define __PLAYBACK_THREAD__

#include <vector>
#include <fstream>

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ColorDebug.hpp"

#define DEFAULT_MS 20  // [ms]

/**
 * @ingroup playbackLocomotion
 *
 * The PlaybackThread class tests the class as a controlboard.
 *
 */
class PlaybackThread : public yarp::os::Thread {

    public:
        virtual bool threadInit();

        /**
         * Main body of the new thread.
         * Override this method to do what you want.
         * After Thread::start is called, this
         * method will start running in a separate thread.
         * It is important that this method either keeps checking
         * Thread::isStopping to see if it should stop, or
         * you override the Thread::onStop method to interact
         * with it in some way to shut the new thread down.
         * There is no really reliable, portable way to stop
         * a thread cleanly unless that thread cooperates.
         */
        virtual void run();


        yarp::dev::IPositionControl *leftLegPos;
        yarp::dev::IPositionDirect *leftLegPosDirect;

        yarp::dev::IPositionControl *rightLegPos;
        yarp::dev::IPositionDirect *rightLegPosDirect;

        int leftLegNumMotors;
        int rightLegNumMotors;

        std::ifstream ifs;

        bool leftLegDone;
        bool rightLegDone;

        int hold;

protected:


};

#endif  // __PLAYBACK_THREAD__

