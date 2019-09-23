// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderTimed(int j, double * enc, double * stamp)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IEncodersTimedRaw * p = deviceMapper.getDevice(j, &localAxis).iEncodersTimedRaw;
    return p ? p->getEncoderTimedRaw(localAxis, enc, stamp) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncodersTimed(double * encs, double * stamps)
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IEncodersTimedRaw * p = rawDevices[i].iEncodersTimedRaw;
        ok &= p ? p->getEncodersTimedRaw(encs + localAxisOffsets[i], stamps + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------
