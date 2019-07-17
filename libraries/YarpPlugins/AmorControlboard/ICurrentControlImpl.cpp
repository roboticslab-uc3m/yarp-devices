// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <algorithm>

// ------------------ ICurrentControl Related -----------------------------------------

bool roboticslab::AmorControlboard::getNumberOfMotors(int *ax)
{
    CD_DEBUG("\n");
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getCurrent(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 currents;

    if (amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *curr = currents[m];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getCurrents(double *currs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 currents;

    if (amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    std::copy(currents, currents + AMOR_NUM_JOINTS, currs);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getCurrentRange(int m, double *min, double *max)
{
    CD_DEBUG("(%d)\n", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, m, &parameters) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *min = -parameters.maxCurrent;
    *max = parameters.maxCurrent;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getCurrentRanges(double *min, double *max)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        ok &= getCurrentRange(j, &min[j], &max[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefCurrents(const double *currs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 currents;

    std::copy(currs, currs + AMOR_NUM_JOINTS, currents);

    if (amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefCurrent(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 currents;

    if (amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    currents[m] = curr;

    if (amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefCurrents(const int n_motor, const int *motors, const double *currs)
{
    CD_DEBUG("(%d)\n", n_motor);

    AMOR_VECTOR7 currents;

    if (amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    for (int i = 0; i < n_motor; i++)
    {
        currents[motors[i]] = currs[i];
    }

    if (amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefCurrents(double *currs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 currents;

    if (amor_get_req_currents(handle, &currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    std::copy(currents, currents + AMOR_NUM_JOINTS, currs);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefCurrent(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 currents;

    if (amor_get_req_currents(handle, &currents) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *curr = currents[m];

    return true;
}

// -----------------------------------------------------------------------------
