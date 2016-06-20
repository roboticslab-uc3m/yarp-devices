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

namespace teo
{

/**
 * @ingroup playbackManipulation
 *
 * @brief WARNING repeated class (you are seeing the playbackManipulation version).
 *
 */
class PlaybackThread : public yarp::os::Thread
{

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


    yarp::dev::IPositionControl *leftArmPos;
    yarp::dev::IPositionDirect *leftArmPosDirect;
    yarp::dev::IEncodersTimed *leftArmEncTimed;
    yarp::dev::ITorqueControl *leftArmTorque;

    yarp::dev::IPositionControl *rightArmPos;
    yarp::dev::IPositionDirect *rightArmPosDirect;
    yarp::dev::IEncodersTimed *rightArmEncTimed;
    yarp::dev::ITorqueControl *rightArmTorque;

    int leftArmNumMotors;
    int rightArmNumMotors;

    std::ifstream ifs;

    bool leftArmDone;
    bool rightArmDone;

    bool hold;

    FILE * logFilePtr;
    bool log;

protected:


};

}  // namespace teo

#endif  // __PLAYBACK_THREAD__

