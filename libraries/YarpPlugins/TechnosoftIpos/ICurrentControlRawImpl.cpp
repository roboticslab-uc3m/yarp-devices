// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------- ICurrentControlRaw Related ------------------------------------

bool roboticslab::TechnosoftIpos::getNumberOfMotorsRaw(int *number)
{
    CD_DEBUG("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRaw(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentsRaw(double *currs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRangeRaw(int m, double *min, double *max)
{
    CD_DEBUG("(%d)\n", m);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentsRaw(const double *currs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentRaw(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentsRaw(const int n_motor, const int *motors, const double *currs)
{
    CD_ERROR("(%d)\n", n_motor);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefCurrentsRaw(double *currs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefCurrentRaw(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);
    return false;
}

// -----------------------------------------------------------------------------
