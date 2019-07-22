// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool roboticslab::TechnosoftIpos::setPositionRaw(int j, double ref)
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;
    linInterpBuffer->updateTarget(ref);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Not implemented yet.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionsRaw(const double *refs)
{
    CD_WARNING("Not implemented yet.\n");
    return true;
}

// -----------------------------------------------------------------------------
