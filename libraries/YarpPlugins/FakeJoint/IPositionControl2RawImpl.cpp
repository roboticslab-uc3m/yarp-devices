// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ######################## IPositionControlRaw Related ##########################

bool roboticslab::FakeJoint::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::positionMoveRaw(int j, double ref)    // encExposed = ref;
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

bool roboticslab::FakeJoint::positionMoveRaw(const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (FakeJoint).\n");

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -------------------------------------------------------------------------------

bool roboticslab::FakeJoint::stopRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################## IPositionControl2Raw Related ##########################

bool roboticslab::FakeJoint::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::stopRaw(const int n_joint, const int *joints)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    targetPositionSemaphore.wait();
    *ref = targetPosition;
    targetPositionSemaphore.post();

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTargetPositionsRaw(double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
