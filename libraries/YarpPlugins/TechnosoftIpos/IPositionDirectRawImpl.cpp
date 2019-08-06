// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool roboticslab::TechnosoftIpos::setPositionRaw(int j, double ref)
{
    CD_DEBUG("(%d,%f)\n", j, ref);

    //-- Check index within range
    if (j != 0) return false;

    linInterpBuffer->updateTarget(ref);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionsRaw(const double *refs)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefPositionRaw(const int joint, double *ref)
{
    CD_DEBUG("(&d)\n", joint);

    //-- Check index within range
    if (joint != 0) return false;

    *ref = linInterpBuffer->getLastTarget();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefPositionsRaw(double *refs)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------
