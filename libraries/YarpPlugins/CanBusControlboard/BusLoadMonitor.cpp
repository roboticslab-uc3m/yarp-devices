// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BusLoadMonitor.hpp"

#include <cmath>

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline unsigned long computeLength(unsigned int len)
    {
        // 44-bit base frame + 3-bit intermission field + stuff bits, see https://w.wiki/GDt
        return 8 * len + 44 + std::floor((34 + 8 * len - 1) / 4.0) + 3;
    }
}

// -----------------------------------------------------------------------------

bool BusLoadMonitor::notifyMessage(const can_message & msg)
{
    bits += computeLength(msg.len);
    return true;
}

// -----------------------------------------------------------------------------

void BusLoadMonitor::run()
{
    double rate = bits.exchange(0) / getPeriod();
    prepare().addFloat64(rate / bitrate);
    write();
}

// -----------------------------------------------------------------------------
