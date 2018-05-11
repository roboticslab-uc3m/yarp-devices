// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IPositionControl2 related --------------------------------

bool roboticslab::AmorControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] = toRad(refs[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] += toRad(deltas[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    amor_movement_status status;

    if (amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    bool flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);

    for (int j = 0; j < n_joint; j++)
    {
        flags[j] = flag;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Not available (%d).\n", n_joint);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("Not available (%d).\n", n_joint);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            CD_ERROR("%s\n", amor_error());
            return false;
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            CD_ERROR("%s\n", amor_error());
            return false;
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(const int n_joint, const int *joints)
{
    CD_WARNING("Selective stop not available, stopping all joints at once (%d).\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPosition(const int joint, double *ref)
{
    CD_DEBUG("(%d)\n", joint);

    if (!indexWithinRange(joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *ref = toDeg(positions[joint]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPositions(double *refs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 positions;

    if (amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        refs[j] = toDeg(positions[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        refs[j] = toDeg(positions[joints[j]]);
    }

    return true;
}

// -----------------------------------------------------------------------------
