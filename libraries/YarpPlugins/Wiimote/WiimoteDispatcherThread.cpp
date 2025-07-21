// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Wiimote.hpp"

#include <cerrno>
#include <cstring>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

void WiimoteDispatcherThread::beforeStart()
{
    std::memset(fds, 0, sizeof(fds));

    fds[0].fd = 0;
    fds[0].events = POLLIN;

    fds[1].fd = ::xwii_iface_get_fd(iface);
    fds[1].events = POLLIN;

    fds_num = 2;
}

// -----------------------------------------------------------------------------

void WiimoteDispatcherThread::run()
{
    WiimoteEventData localEventData;

    while (!yarp::os::Thread::isStopping())
    {
        if (::poll(fds, fds_num, -1) < 0 && errno != EINTR)
        {
            yCError(WII) << "Cannot poll fds:" << -errno;
            return;
        }

#ifdef _XWIIMOTE_LEGACY_INTERFACE
        // xwii_iface_dispatch not available on Trusty
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/134
        int ret = ::xwii_iface_poll(iface, &event);
#else
        int ret = ::xwii_iface_dispatch(iface, &event, sizeof(event));
#endif

        if (ret != 0)
        {
            if (ret != -EAGAIN)
            {
                yCError(WII) << "Read failed with err:" << ret;
                return;
            }
        }

        switch (event.type)
        {
        case XWII_EVENT_KEY:
            yCDebug(WII, "Keypress event: code %d, state %d", event.v.key.code, event.v.key.state);

            switch (event.v.key.code)
            {
            case XWII_KEY_A:
                localEventData.buttonA = event.v.key.state == 1;
                break;
            case XWII_KEY_B:
                localEventData.buttonB = event.v.key.state == 1;
                break;
            case XWII_KEY_ONE:
                localEventData.button1 = event.v.key.state == 1;
                break;
            case XWII_KEY_TWO:
                localEventData.button2 = event.v.key.state == 1;
                break;
            default:
                continue;
            }

            break;
        case XWII_EVENT_ACCEL:
            yCDebug(WII, "Accel event: [x] %d, [y] %d, [z] %d", event.v.abs[0].x, event.v.abs[0].y, event.v.abs[0].z);
            localEventData.accelX = event.v.abs[0].x;
            localEventData.accelY = event.v.abs[0].y;
            localEventData.accelZ = event.v.abs[0].z;
            break;
        default:
            continue;
        }

        eventDataMutex.lock();
        eventData = localEventData;
        eventDataMutex.unlock();
    }
}

// -----------------------------------------------------------------------------

WiimoteEventData WiimoteDispatcherThread::getEventData() const
{
    std::lock_guard lock(eventDataMutex);
    return eventData;
}

// -----------------------------------------------------------------------------
