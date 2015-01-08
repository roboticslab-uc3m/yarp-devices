// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MOVE_GRIPPER_THREAD__
#define __MOVE_GRIPPER_THREAD__

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#include <yarp/os/Thread.h>
#include <yarp/dev/ControlBoardInterfaces.h>

// Thanks: Alnitak @ http://stackoverflow.com/questions/448944/c-non-blocking-keyboard-input

/**
 * @ingroup recordManipulation
 *
 * The MoveGripperThread class tests the class as a controlboard.
 *
 */
class MoveGripperThread : public yarp::os::Thread {

    public:

        bool setLeftOpenChar(const char& openLeftChar);
        bool setLeftCloseChar(const char& closeLeftChar);
        bool setRightOpenChar(const char& openRightChar);
        bool setRightCloseChar(const char& closeRightChar);

        virtual bool threadInit();

        /**
         * Loop function. This is the thread itself.
         */
        virtual void run();

        yarp::dev::IPositionControl *leftGripperPos;
        yarp::dev::IPositionControl *rightGripperPos;


    protected:

        char openLeftChar;
        char closeLeftChar;
        char openRightChar;
        char closeRightChar;

        WINDOW *w;
};


#endif  // __MOVE_GRIPPER_THREAD__

