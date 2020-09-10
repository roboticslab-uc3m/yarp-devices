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

bool OneWayMonitor::notifyMessage(const can_message & msg)
{
    bits += computeLength(msg.len);
    return true;
}

// -----------------------------------------------------------------------------

unsigned int OneWayMonitor::reset()
{
    return bits.exchange(0);
}

// -----------------------------------------------------------------------------

void BusLoadMonitor::run()
{
    unsigned int readBits = readMonitor.reset();
    unsigned int writtenBits = writeMonitor.reset();
    unsigned int overallBits = readBits + writtenBits;

    double limit = bitrate * getPeriod(); // bits per each thread step

    auto & b = prepare();
    b.clear();
    b.addFloat64(readBits / limit);
    b.addFloat64(writtenBits / limit);
    b.addFloat64(overallBits / limit);

    write();
}

// -----------------------------------------------------------------------------
