// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BusLoadMonitor.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    unsigned long computeLength(const can_message & frame)
    {
        return 0;
    }
}

// -----------------------------------------------------------------------------

bool BusLoadMonitor::notifyMessage(const can_message & msg)
{
    bits += computeLength(msg);
    return true;
}

// -----------------------------------------------------------------------------

void BusLoadMonitor::run()
{
    double rate = bits.exchange(0) / yarp::os::PeriodicThread::getPeriod();
    prepare().addFloat64(rate / bitrate);
    write();
}

// -----------------------------------------------------------------------------
