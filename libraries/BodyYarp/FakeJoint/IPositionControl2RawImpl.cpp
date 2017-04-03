// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ######################## IPositionControlRaw Related ##########################

bool teo::FakeJoint::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::setPositionModeRaw()
{
    return setPositionModeRaw(0);
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    encoderReady.wait();
    this->encoder = ref;  // Already passed through Adjust range.
    encoderReady.post();

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::positionMoveRaw(const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (FakeJoint).\n");

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -------------------------------------------------------------------------------

bool teo::FakeJoint::stopRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################## IPositionControl2Raw Related ##########################

bool teo::FakeJoint::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::stopRaw(const int n_joint, const int *joints)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    targetPositionSemaphore.wait();
    *ref = targetPosition;
    targetPositionSemaphore.post();

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::getTargetPositionsRaw(double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::FakeJoint::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
