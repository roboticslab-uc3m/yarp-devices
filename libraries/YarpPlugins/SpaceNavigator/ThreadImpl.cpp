// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <yarp/os/SystemClock.h>

constexpr auto SLEEP_DURATION = 0.001; // [s]

// -----------------------------------------------------------------------------

void SpaceNavigator::run()
{
    spnav_event sev;

    unsigned int noDataCounter = 0;

    while (!yarp::os::Thread::isStopping())
    {
        if (std::lock_guard lock(mtx); ::spnav_poll_event(&sev) != 0)
        {
            if (sev.type == SPNAV_EVENT_MOTION)
            {
                dx = sev.motion.z;
                dy = -sev.motion.x;
                dz = sev.motion.y;
                drx = sev.motion.rz;
                dry = -sev.motion.rx;
                drz = sev.motion.ry;
            }
            else if (sev.type == SPNAV_EVENT_BUTTON)
            {
                float * p_button = (sev.button.bnum == 0) ? &button1 : &button2;
                *p_button = sev.button.press; // 1: press flank, 0: release flank
            }

            noDataCounter = 0;
        }
        else
        {
            noDataCounter++;

            if (noDataCounter >= m_maxNoDataIterations)
            {
                noDataCounter = 0;
                dx = dy = dz = drx = dry = drz = 0.0;
                // button1 = button2 = 0.f; // buttons should preserve pressed/released state
            }
        }

        yarp::os::SystemClock::delaySystem(SLEEP_DURATION);
    }
}

// -----------------------------------------------------------------------------
