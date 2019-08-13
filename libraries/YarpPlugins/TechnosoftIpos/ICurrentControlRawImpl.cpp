// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

namespace
{
    // return -1 for negative numbers, +1 for positive numbers, 0 for zero
    // https://stackoverflow.com/a/4609795
    template <typename T>
    inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
}

// ------------------- ICurrentControlRaw Related ------------------------------------

bool roboticslab::TechnosoftIpos::getNumberOfMotorsRaw(int *number)
{
    CD_DEBUG("\n");
    return getAxes(number);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRaw(int m, double *curr)
{
    //CD_DEBUG("(%d)\n", m);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if (m != 0) return false;

    //*************************************************************
    uint8_t msg_getCurrent[]= {0x40,0x7E,0x20,0x00,0x00,0x00,0x00,0x00};

    if (!send(0x600, 4, msg_getCurrent))
    {
        CD_ERROR("Could not send msg_getCurrent. %s\n", msgToStr(0x600, 4, msg_getCurrent).c_str());
        return false;
    }
    //CD_SUCCESS("Sent msg_getCurrent. %s\n", msgToStr(0x600, 4, msg_getCurrent).c_str());    //-- Too verbose in controlboardwrapper2 stream.

    if (!sdoSemaphore->await(msg_getCurrent))
    {
        CD_ERROR("Did not receive msg_getCurrent response. %s\n", msgToStr(0x600, 4, msg_getCurrent).c_str());
        return false;
    }

    int16_t got;
    std::memcpy(&got, msg_getCurrent + 4, 2);
    *curr = got * sgn(tr) * 2.0 * drivePeakCurrent / 65520.0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentsRaw(double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRangeRaw(int m, double *min, double *max)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if (m != 0) return false;

    //*************************************************************
    uint8_t msg_getCurrentLimit[]= {0x40,0x7F,0x20,0x00,0x00,0x00,0x00,0x00};

    if (!send(0x600, 4, msg_getCurrentLimit))
    {
        CD_ERROR("Could not send \"Current limit\" query. %s\n", msgToStr(0x600, 4, msg_getCurrentLimit).c_str());
        return false;
    }
    CD_SUCCESS("Sent msg_getCurrentLimit. %s\n", msgToStr(0x600, 4, msg_getCurrentLimit).c_str());

    if (!sdoSemaphore->await(msg_getCurrentLimit))
    {
        CD_ERROR("Did not receive \"Current limit\" response. %s\n", msgToStr(0x600, 4, msg_getCurrentLimit).c_str());
        return false;
    }

    uint16_t got;
    std::memcpy(&got, msg_getCurrentLimit + 4, 2);

    *max = 2 * drivePeakCurrent * (32767 - got) / 65520;
    *min = -(*max);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRangesRaw(double *min, double *max)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentsRaw(const double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentRaw(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if (m != 0) return false;

    //*************************************************************
    uint8_t msg_ref_current[]= {0x23,0x1C,0x20,0x00,0x00,0x00,0x00,0x00}; // put 23 because it is a target

    int sendRefCurrent = curr * sgn(tr) * 65520.0 / (2 * drivePeakCurrent); // Page 109 of 263.
    std::memcpy(msg_ref_current + 6, &sendRefCurrent, 2);

    if (!send(0x600, 8, msg_ref_current))
    {
        CD_ERROR("Could not send refCurrent. %s\n", msgToStr(0x600, 8, msg_ref_current).c_str());
        return false;
    }

    CD_SUCCESS("Sent refCurrent. %s\n", msgToStr(0x600, 8, msg_ref_current).c_str());

    if (!sdoSemaphore->await(msg_ref_current))
    {
        CD_ERROR("Did not receive refCurrent ack. %s\n", msgToStr(0x600, 8, msg_ref_current).c_str());
        return false;
    }

    refCurrentSemaphore.wait();
    refCurrent = curr;
    refCurrentSemaphore.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentsRaw(const int n_motor, const int *motors, const double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefCurrentsRaw(double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefCurrentRaw(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    refCurrentSemaphore.wait();
    *curr = refCurrent;
    refCurrentSemaphore.post();

    return true;
}

// -----------------------------------------------------------------------------
