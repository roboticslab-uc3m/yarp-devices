// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <algorithm>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ ICurrentControl Related -----------------------------------------

bool AmorControlBoard::getNumberOfMotors(int *ax)
{
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getCurrent(int m, double *curr)
{
    yCTrace(AMOR, "%d", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_currents() failed: %s", amor_error());
        return false;
    }

    *curr = currents[m];

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getCurrents(double *currs)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_currents() failed: %s", amor_error());
        return false;
    }

    std::copy(currents, currents + AMOR_NUM_JOINTS, currs);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getCurrentRange(int m, double *min, double *max)
{
    yCTrace(AMOR, "%d", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, m, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info() failed: %s", amor_error());
        return false;
    }

    *min = -parameters.maxCurrent;
    *max = parameters.maxCurrent;

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getCurrentRanges(double *min, double *max)
{
    yCTrace(AMOR, "");

    bool ok = true;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        ok &= getCurrentRange(j, &min[j], &max[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefCurrents(const double *currs)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 currents;

    std::copy(currs, currs + AMOR_NUM_JOINTS, currents);

    if (std::lock_guard lock(handleMutex); amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_set_currents() failed: %s", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefCurrent(int m, double curr)
{
    yCTrace(AMOR, "%d", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_currents() failed: %s", amor_error());
        return false;
    }

    currents[m] = curr;

    if (std::lock_guard lock(handleMutex); amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_set_currents() failed: %s", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefCurrents(const int n_motor, const int *motors, const double *currs)
{
    yCTrace(AMOR, "%d", n_motor);

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_currents() failed: %s", amor_error());
        return false;
    }

    for (int i = 0; i < n_motor; i++)
    {
        currents[motors[i]] = currs[i];
    }

    if (std::lock_guard lock(handleMutex); amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_set_currents() failed: %s", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefCurrents(double *currs)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_req_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_currents() failed: %s", amor_error());
        return false;
    }

    std::copy(currents, currents + AMOR_NUM_JOINTS, currs);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefCurrent(int m, double *curr)
{
    yCTrace(AMOR, "%d", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_req_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_currents() failed: %s", amor_error());
        return false;
    }

    *curr = currents[m];

    return true;
}

// -----------------------------------------------------------------------------
