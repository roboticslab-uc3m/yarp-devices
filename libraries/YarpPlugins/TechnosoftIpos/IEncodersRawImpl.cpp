// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getAxes(int * ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::resetEncoderRaw(int j)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::resetEncodersRaw()
{
    return resetEncoderRaw(0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::setEncoderRaw(int j, double val)
{
    yTrace("%d %f", j, val);
    CHECK_JOINT(j);
    std::int32_t data = vars.degreesToInternalUnits(val);

    if (!can->sdo()->download("Set actual position", data, 0X2081))
    {
        return false;
    }

    vars.lastEncoderRead->reset(data);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::setEncodersRaw(const double * vals)
{
    return setEncoderRaw(0, vals[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderRaw(int j, double * v)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    std::int32_t temp = vars.lastEncoderRead->queryPosition();
    *v = vars.internalUnitsToDegrees(temp);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncodersRaw(double * encs)
{
    return getEncoderRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderSpeedRaw(int j, double * sp)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    double temp = vars.lastEncoderRead->querySpeed();
    *sp = vars.internalUnitsToDegrees(temp, 1);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderSpeedsRaw(double * spds)
{
    return getEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderAccelerationRaw(int j, double * acc)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    double temp = vars.lastEncoderRead->queryAcceleration();
    *acc = vars.internalUnitsToDegrees(temp, 2);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderAccelerationsRaw(double * accs)
{
    return getEncoderAccelerationRaw(0, &accs[0]);
}

//---------------------------------------------------------------------------------------

bool TechnosoftIpos::getEncoderTimedRaw(int j, double * enc, double * time)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    std::int32_t temp = vars.lastEncoderRead->queryPosition();
    *enc = vars.internalUnitsToDegrees(temp);
    *time = vars.lastEncoderRead->queryTime();
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIpos::getEncodersTimedRaw(double * encs, double * times)
{
    return getEncoderTimedRaw(0, &encs[0], &times[0]);
}

// -----------------------------------------------------------------------------
