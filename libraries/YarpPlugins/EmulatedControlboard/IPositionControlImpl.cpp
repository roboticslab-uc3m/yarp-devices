// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <cmath>
#include <yarp/os/Log.h>

// ------------------- IPositionControl Related --------------------------------

bool roboticslab::EmulatedControlboard::getAxes(int *ax)
{
    *ax = axes;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::positionMove(int j, double ref)  // encExposed = ref;
{
    if ((unsigned int)j > axes)
    {
        yError("Axis index exceeds number of axes");
        return false;
    }

    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {  
        yError("Will not positionMove as not in positionMode");
        return false;
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j] = ref;
    double encExposed = getEncExposed(j);

    if (std::abs(targetExposed[j] - encExposed) < jointTol[j])
    {
        stop(j);  // puts jointStatus[j] = 0;
        yInfo("Joint q%d reached target", j + 1);
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

bool roboticslab::EmulatedControlboard::positionMove(const double *refs)  // encExposed = refs;
{
    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yError("Will not positionMove as not in positionMode");
        return false;
    }

    // Find out the maximum time to move
    double max_time = 0;
    std::vector<double> encsExposed = getEncsExposed();

    for (unsigned int motor = 0; motor < axes; motor++)
    {
        yInfo("dist[%d]: %f", motor, std::abs(refs[motor] - encsExposed[motor]));
        yInfo("refSpeed[%d]: %f", motor, refSpeed[motor]);

        if (std::abs((refs[motor] - encsExposed[motor]) / refSpeed[motor]) > max_time)
        {
            max_time = std::abs((refs[motor] - encsExposed[motor]) / refSpeed[motor]);
            yInfo("candidate: %f", max_time);
        }
    }

    yInfo("max_time[final]: %f", max_time);

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    for (unsigned int motor = 0; motor < axes; motor++)
    {
        targetExposed[motor] = refs[motor];
        velRaw[motor] = ((refs[motor] - encsExposed[motor]) / max_time) * velRawExposed[motor];

        if (velRaw[motor] != velRaw[motor])
        {
            velRaw[motor] = 0;  // protect against NaN
        }

        yInfo("velRaw[%d]: %f", motor, velRaw[motor]);
        jointStatus[motor] = POSITION_MOVE;

        if (std::abs(targetExposed[motor] - encsExposed[motor]) < jointTol[motor])
        {
            stop(motor);  // puts jointStatus[motor]=0;
            yInfo("Joint q%d reached target", motor + 1);
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::relativeMove(int j, double delta)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yError("EmulatedControlboard will not relativeMove as not in positionMode");
        return false;
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    double encExposed = getEncExposed(j);
    targetExposed[j]= encExposed + delta;

    if (std::abs(targetExposed[j] - encExposed) < jointTol[j])
    {
        stop(j);  // puts jointStatus[j]=0;
        yInfo("Joint q%d already at target", j + 1);
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

bool roboticslab::EmulatedControlboard::relativeMove(const double *deltas)  // encExposed = deltas + encExposed
{
    // Check if we are in position mode.
    if (controlMode != POSITION_MODE)
    {
        yError("Will not relativeMove as not in positionMode");
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
      yInfo("velRaw[%d]: %f", motor, velRaw[motor]);
      jointStatus[motor] = POSITION_MOVE;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::checkMotionDone(int j, bool *flag)
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

bool roboticslab::EmulatedControlboard::checkMotionDone(bool *flag)
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

bool roboticslab::EmulatedControlboard::setRefSpeed(int j, double sp)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    refSpeed[j] = sp;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefSpeeds(const double *spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= setRefSpeed(i, spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefAcceleration(int j, double acc)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    refAcc[j] = acc;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefAccelerations(const double *accs)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= setRefAcceleration(i, accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefSpeed(int j, double *ref)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    *ref = refSpeed[j];
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefSpeeds(double *spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getRefSpeed(i, &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefAcceleration(int j, double *acc)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    *acc = refAcc[j];
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefAccelerations(double *accs)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getRefAcceleration(i, &accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::stop(int j)
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

bool roboticslab::EmulatedControlboard::stop()
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= stop(i);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    // must implement mask!
    return positionMove(refs);
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    yWarning("Group relativeMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    yWarning("Group checkMotionDone() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    yWarning("Group setRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    yWarning("Group setRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    yWarning("Group getRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    yWarning("Group getRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::stop(const int n_joint, const int *joints)
{
    yWarning("Group stop() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTargetPosition(const int joint, double *ref)
{
    yWarning("getTargetPosition() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTargetPositions(double *refs)
{
    yWarning("getTargetPositions() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    yWarning("getTargetPositions() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------
