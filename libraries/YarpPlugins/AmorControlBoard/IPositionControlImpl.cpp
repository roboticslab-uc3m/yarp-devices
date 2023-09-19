// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IPositionControl related --------------------------------

bool AmorControlBoard::getAxes(int *ax)
{
    *ax = AMOR_NUM_JOINTS;
    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::positionMove(int j, double ref)
{
    yCTrace(AMOR, "%d %f", j, ref);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    positions[j] = toRad(ref);

    std::lock_guard lock(handleMutex);
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::positionMove(const double *refs)
{
    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] = toRad(refs[j]);
    }

    std::lock_guard lock(handleMutex);
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::relativeMove(int j, double delta)
{
    yCTrace(AMOR, "%d %f", j, delta);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    positions[j] += toRad(delta);

    std::lock_guard lock(handleMutex);
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::relativeMove(const double *deltas)
{
    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] += toRad(deltas[j]);
    }

    std::lock_guard lock(handleMutex);
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::checkMotionDone(int j, bool *flag)
{
    if (!indexWithinRange(j))
    {
        return false;
    }

    return checkMotionDone(flag);
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::checkMotionDone(bool *flag)
{
    yCTrace(AMOR, "");

    amor_movement_status status;

    if (std::lock_guard lock(handleMutex); amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_movement_status(): %s", amor_error());
        return false;
    }

    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefSpeed(int j, double sp)
{
    yCError(AMOR, "setRefSpeed() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefSpeeds(const double *spds)
{
    yCError(AMOR, "setRefSpeeds() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefAcceleration(int j, double acc)
{
    yCError(AMOR, "setRefAcceleration() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefAccelerations(const double *accs)
{
    yCError(AMOR, "setRefAccelerations() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefSpeed(int j, double *ref)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
        return false;
    }

    *ref = toDeg(parameters.maxVelocity);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefSpeeds(double *spds)
{
    yCTrace(AMOR, "");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefAcceleration(int j, double *acc)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
        return false;
    }

    *acc = toDeg(parameters.maxAcceleration);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefAccelerations(double *accs)
{
    yCTrace(AMOR, "");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::stop(int j)
{
    yCWarning(AMOR, "Selective stop not available, stopping all joints at once (%d)", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::stop()
{
    yCTrace(AMOR, "");
    std::lock_guard lock(handleMutex);
    return amor_controlled_stop(handle) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] = toRad(refs[j]);
    }

    std::lock_guard lock(handleMutex);
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] += toRad(deltas[j]);
    }

    std::lock_guard lock(handleMutex);
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    amor_movement_status status;

    if (std::lock_guard lock(handleMutex); amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_movement_status(): %s", amor_error());
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

bool AmorControlBoard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    yCError(AMOR, "setRefSpeeds() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    yCError(AMOR, "setRefAccelerations() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::stop(const int n_joint, const int *joints)
{
    yCWarning(AMOR, "Selective stop not available, stopping all joints at once (%d)", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getTargetPosition(const int joint, double *ref)
{
    yCTrace(AMOR, "%d", joint);

    if (!indexWithinRange(joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_positions(): %s", amor_error());
        return false;
    }

    *ref = toDeg(positions[joint]);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getTargetPositions(double *refs)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        refs[j] = toDeg(positions[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        refs[j] = toDeg(positions[joints[j]]);
    }

    return true;
}

// -----------------------------------------------------------------------------
