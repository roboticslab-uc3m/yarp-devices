// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <cmath>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IPositionControl Related --------------------------------

bool EmulatedControlBoard::getAxes(int *ax)
{
    *ax = axes;
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::positionMove(int j, double ref)  // encExposed = ref;
{
    if ((unsigned int)j > axes)
    {
        yCError(ECB, "Axis index exceeds number of axes");
        return false;
    }

    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yCError(ECB, "Will not positionMove as not in positionMode");
        return false;
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j] = ref;
    double encExposed = getEncExposed(j);

    if (std::abs(targetExposed[j] - encExposed) < jointTol[j])
    {
        stop(j);  // puts jointStatus[j] = 0;
        yCInfo(ECB, "Joint q%d reached target", j + 1);
        return true;
    }
    else if (ref > encExposed)
    {
        //if(!velocityMove(j, refSpeed[j])) return false;
        velRaw[j] = (refSpeed[j] * velRawExposed[j]);
    }
    else
    {
        //if(!velocityMove(j, -refSpeed[j])) return false;
        velRaw[j] = -(refSpeed[j] * velRawExposed[j]);
    }

    jointStatus[j] = POSITION_MOVE;

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::positionMove(const double *refs)  // encExposed = refs;
{
    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yCError(ECB, "Will not positionMove as not in positionMode");
        return false;
    }

    // Find out the maximum time to move
    double max_time = 0;
    std::vector<double> encsExposed = getEncsExposed();

    for (unsigned int motor = 0; motor < axes; motor++)
    {
        yCInfo(ECB, "dist[%d]: %f", motor, std::abs(refs[motor] - encsExposed[motor]));
        yCInfo(ECB, "refSpeed[%d]: %f", motor, refSpeed[motor]);

        if (std::abs((refs[motor] - encsExposed[motor]) / refSpeed[motor]) > max_time)
        {
            max_time = std::abs((refs[motor] - encsExposed[motor]) / refSpeed[motor]);
            yCInfo(ECB, "candidate: %f", max_time);
        }
    }

    yCInfo(ECB, "max_time[final]: %f", max_time);

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    for (unsigned int motor = 0; motor < axes; motor++)
    {
        targetExposed[motor] = refs[motor];
        velRaw[motor] = ((refs[motor] - encsExposed[motor]) / max_time) * velRawExposed[motor];

        if (velRaw[motor] != velRaw[motor])
        {
            velRaw[motor] = 0;  // protect against NaN
        }

        yCInfo(ECB, "velRaw[%d]: %f", motor, velRaw[motor]);
        jointStatus[motor] = POSITION_MOVE;

        if (std::abs(targetExposed[motor] - encsExposed[motor]) < jointTol[motor])
        {
            stop(motor);  // puts jointStatus[motor]=0;
            yCInfo(ECB, "Joint q%d reached target", motor + 1);
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::relativeMove(int j, double delta)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yCError(ECB, "EmulatedControlBoard will not relativeMove as not in positionMode");
        return false;
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    double encExposed = getEncExposed(j);
    targetExposed[j]= encExposed + delta;

    if (std::abs(targetExposed[j] - encExposed) < jointTol[j])
    {
        stop(j);  // puts jointStatus[j]=0;
        yCInfo(ECB, "Joint q%d already at target", j + 1);
        return true;
    }
    else if (targetExposed[j] > encExposed)
    {
        // if(!velocityMove(j, refSpeed[j])) return false;
        velRaw[j] = (refSpeed[j] * velRawExposed[j]);
    }
    else
    {
        // if(!velocityMove(j, -refSpeed[j])) return false;
        velRaw[j] = -(refSpeed[j] * velRawExposed[j]);
    }

    jointStatus[j] = POSITION_MOVE;

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::relativeMove(const double *deltas)  // encExposed = deltas + encExposed
{
    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yCError(ECB, "Will not relativeMove as not in positionMode");
        return false;
    }

    // Find out the maximum angle to move
    double max_dist = 0;
    double time_max_dist = 0;

    for (unsigned int motor = 0; motor < axes; motor++)
    {
        if (std::abs(deltas[motor]) > max_dist)
        {
            max_dist = std::abs(deltas[motor]);
            time_max_dist = max_dist / refSpeed[motor];  // the max_dist motor will be at refSpeed
        }
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    std::vector<double> encsExposed = getEncsExposed();

    for (unsigned int motor = 0; motor < axes; motor++)
    {
      targetExposed[motor] = encsExposed[motor] + deltas[motor];
      velRaw[motor] = ((deltas[motor]) / time_max_dist) * velRawExposed[motor];
      yCInfo(ECB, "velRaw[%d]: %f", motor, velRaw[motor]);
      jointStatus[motor] = POSITION_MOVE;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::checkMotionDone(int j, bool *flag)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    bool done = true;

    if (jointStatus[j] != NOT_CONTROLLING)
    {
        done = false;
    }

    *flag = done;
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::checkMotionDone(bool *flag)
{
    bool done = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        if (jointStatus[i] != NOT_CONTROLLING)
        {
            done = false;
        }
    }

    *flag = done;
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setRefSpeed(int j, double sp)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    refSpeed[j] = sp;
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setRefSpeeds(const double *spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= setRefSpeed(i, spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setRefAcceleration(int j, double acc)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    refAcc[j] = acc;
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setRefAccelerations(const double *accs)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= setRefAcceleration(i, accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefSpeed(int j, double *ref)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    *ref = refSpeed[j];
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefSpeeds(double *spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getRefSpeed(i, &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefAcceleration(int j, double *acc)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    *acc = refAcc[j];
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefAccelerations(double *accs)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getRefAcceleration(i, &accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::stop(int j)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    velRaw[j] = 0.0;
    jointStatus[j] = NOT_CONTROLLING;

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::stop()
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= stop(i);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    // must implement mask!
    return positionMove(refs);
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    yCWarning(ECB, "Group relativeMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    yCWarning(ECB, "Group checkMotionDone() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    yCWarning(ECB, "Group setRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    yCWarning(ECB, "Group setRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    yCWarning(ECB, "Group getRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    yCWarning(ECB, "Group getRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::stop(const int n_joint, const int *joints)
{
    yCWarning(ECB, "Group stop() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getTargetPosition(const int joint, double *ref)
{
    yCWarning(ECB, "getTargetPosition() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getTargetPositions(double *refs)
{
    yCWarning(ECB, "getTargetPositions() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    yCWarning(ECB, "getTargetPositions() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------
