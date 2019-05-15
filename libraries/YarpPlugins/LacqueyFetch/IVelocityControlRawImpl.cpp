// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ######################### IVelocityControlRaw Related ###########################

bool roboticslab::LacqueyFetch::velocityMoveRaw(int j, double sp)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################### IVelocityControl2Raw Related #########################

bool roboticslab::LacqueyFetch::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefVelocityRaw(const int joint, double *vel)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefVelocitiesRaw(double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------
/*
bool roboticslab::LacqueyFetch::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("\n");
    return false;
}
*/
// -----------------------------------------------------------------------------
/*
bool roboticslab::LacqueyFetch::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_ERROR("\n");
    return false;
}
*/
// -----------------------------------------------------------------------------
/*
bool roboticslab::LacqueyFetch::stopRaw(const int n_joint, const int *joints)
{
    CD_ERROR("\n");
    return false;
}
*/
// -----------------------------------------------------------------------------
