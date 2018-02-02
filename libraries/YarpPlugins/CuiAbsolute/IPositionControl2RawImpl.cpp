// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// ------------------ IPositionControlRaw Related ----------------------------------------

bool roboticslab::CuiAbsolute::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (CuiAbsolute).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::positionMoveRaw(const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (CuiAbsolute).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

bool roboticslab::CuiAbsolute::stopRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------- IPositionControl2Raw Related ----------------------------


bool roboticslab::CuiAbsolute::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getTargetPositionsRaw(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

bool roboticslab::CuiAbsolute::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

