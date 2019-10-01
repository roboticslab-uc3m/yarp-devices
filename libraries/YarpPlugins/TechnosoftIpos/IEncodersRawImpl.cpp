// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::resetEncoderRaw(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::resetEncodersRaw()
{
    CD_DEBUG("\n");
    return resetEncoderRaw(0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::setEncoderRaw(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    std::int32_t data = vars.degreesToInternalUnits(val);

    if (!can->sdo()->download("Set actual position", data, 0X2081))
    {
        return false;
    }

    vars.lastEncoderRead.reset(val);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::setEncodersRaw(const double *vals)
{
    CD_DEBUG("\n");
    return setEncoderRaw(0, vals[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderRaw(int j, double *v)
{
    //CD_DEBUG("%d\n", j); //-- Too verbose in stream.
    CHECK_JOINT(j);
    *v = vars.lastEncoderRead.queryPosition();
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncodersRaw(double *encs)
{
    CD_DEBUG("\n");
    return getEncoderRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *sp = vars.lastEncoderRead.querySpeed();
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderSpeedsRaw(double *spds)
{
    CD_DEBUG("\n");
    return getEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderAccelerationRaw(int j, double *acc)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *acc = vars.lastEncoderRead.queryAcceleration();
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderAccelerationsRaw(double *accs)
{
    CD_DEBUG("\n");
    return getEncoderAccelerationRaw(0, &accs[0]);
}

//---------------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderTimedRaw(int j, double * enc, double * time)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *enc =  vars.lastEncoderRead.queryPosition();
    *time = vars.lastEncoderRead.queryTime();
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncodersTimedRaw(double * encs, double * times)
{
    CD_DEBUG("\n");
    return getEncoderTimedRaw(0, &encs[0], &times[0]);
}

// -----------------------------------------------------------------------------
