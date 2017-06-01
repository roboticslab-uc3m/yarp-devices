// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IPositionControl related --------------------------------

bool roboticslab::AmorControlboard::getAxes(int *ax)
{
    CD_DEBUG("\n");
    *ax = AMOR_NUM_JOINTS;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    positions[j] = toRad(ref);

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(const double *refs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] = toRad(refs[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n", j, delta);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    positions[j] += toRad(delta);

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const double *deltas)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] += toRad(deltas[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(int j, bool *flag)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return checkMotionDone(flag);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(bool *flag)
{
    CD_DEBUG("\n");

    amor_movement_status status;

    if (amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeed(int j, double sp)
{
    CD_ERROR("Not available (%d, %f).\n", j, sp);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const double *spds)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAcceleration(int j, double acc)
{
    CD_ERROR("Not available (%d, %f).\n", j, acc);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const double *accs)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeed(int j, double *ref)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *ref = toDeg(parameters.maxVelocity);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(double *spds)
{
    CD_ERROR("\n");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            CD_ERROR("%s\n", amor_error());
            return false;
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAcceleration(int j, double *acc)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *acc = toDeg(parameters.maxAcceleration);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(double *accs)
{
    CD_ERROR("\n");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            CD_ERROR("%s\n", amor_error());
            return false;
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(int j)
{
    CD_WARNING("Selective stop not available, stopping all joints at once (%d).\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop()
{
    CD_DEBUG("\n");
    return amor_controlled_stop(handle) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------
