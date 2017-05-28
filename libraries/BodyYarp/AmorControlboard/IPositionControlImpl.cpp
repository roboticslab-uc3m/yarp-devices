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
        return false;

    AMOR_VECTOR7 refs;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        if (i == j)
            refs[i] = ref * 180 / 3.14159;
        else
            refs[i] = 0;
    }

    return amor_set_positions(handle, refs) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(const double *refs)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
        ok &= positionMove(j, refs[j]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n", j, delta);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const double *deltas)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(int j, bool *flag)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return checkMotionDone(flag);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(bool *flag)
{
    CD_DEBUG("\n");
    amor_movement_status status;
    if (amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not get AMOR movement status!\n");
        return false;
    }
    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeed(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const double *spds)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= setRefSpeed(i, spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAcceleration(int j, double acc)
{
    CD_DEBUG("(%d, %f)\n", j, acc);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const double *accs)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= setRefAcceleration(i, accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeed(int j, double *ref)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(double *spds)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= getRefSpeed(i, &spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAcceleration(int j, double *acc)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(double *accs)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= getRefAcceleration(i, &accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return stop();
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop()
{
    CD_DEBUG("\n");
    return amor_controlled_stop(handle) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------
