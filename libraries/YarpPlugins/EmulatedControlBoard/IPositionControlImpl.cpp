// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IPositionControl Related --------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getAxes(std::size_t & ax)
{
    ax = m_axes;
    return yarp::dev::ReturnValue_ok;
}
#else
bool EmulatedControlBoard::getAxes(int * ax)
{
    *ax = m_axes;
    return true;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::positionMove(int j, double ref)
#else
bool EmulatedControlBoard::positionMove(int j, double ref)
#endif
{
    CHECK_JOINT(j);

    if (controlMode != VOCAB_CM_POSITION)
    {
        yCError(ECB) << "Will not positionMove as not in positionMode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return false;
#endif
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j] = ref;
    double encExposed = getEncExposed(j);

    if (std::abs(targetExposed[j] - encExposed) < m_jointTols[j])
    {
        stop(j);  // puts jointStatus[j] = 0;
        yCInfo(ECB, "Joint q%d reached target", j + 1);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_ok;
#else
        return true;
#endif
    }
    else if (ref > encExposed)
    {
        //if(!velocityMove(j, m_refSpeeds[j])) return false;
        velRaw[j] = (m_refSpeeds[j] * m_velRawExposeds[j]);
    }
    else
    {
        //if(!velocityMove(j, -m_refSpeeds[j])) return false;
        velRaw[j] = -(m_refSpeeds[j] * m_velRawExposeds[j]);
    }

    jointStatus[j] = POSITION_MOVE;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::positionMove(const double * refs)
#else
bool EmulatedControlBoard::positionMove(const double * refs)
#endif
{
    if (controlMode != VOCAB_CM_POSITION)
    {
        yCError(ECB) << "Will not positionMove as not in positionMode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return false;
#endif
    }

    // Find out the maximum time to move
    double max_time = 0;
    std::vector<double> encsExposed = getEncsExposed();

    for (unsigned int motor = 0; motor < m_axes; motor++)
    {
        yCInfo(ECB, "dist[%d]: %f", motor, std::abs(refs[motor] - encsExposed[motor]));
        yCInfo(ECB, "m_refSpeeds[%d]: %f", motor, m_refSpeeds[motor]);

        if (std::abs((refs[motor] - encsExposed[motor]) / m_refSpeeds[motor]) > max_time)
        {
            max_time = std::abs((refs[motor] - encsExposed[motor]) / m_refSpeeds[motor]);
            yCInfo(ECB) << "candidate:" << max_time;
        }
    }

    yCInfo(ECB) << "max_time[final]:" << max_time;

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    for (unsigned int motor = 0; motor < m_axes; motor++)
    {
        targetExposed[motor] = refs[motor];
        velRaw[motor] = ((refs[motor] - encsExposed[motor]) / max_time) * m_velRawExposeds[motor];

        if (velRaw[motor] != velRaw[motor])
        {
            velRaw[motor] = 0;  // protect against NaN
        }

        yCInfo(ECB, "velRaw[%d]: %f", motor, velRaw[motor]);
        jointStatus[motor] = POSITION_MOVE;

        if (std::abs(targetExposed[motor] - encsExposed[motor]) < m_jointTols[motor])
        {
            stop(motor);  // puts jointStatus[motor]=0;
            yCInfo(ECB, "Joint q%d reached target", motor + 1);
        }
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::positionMove(int n_joint, const int * joints, const double * refs)
#else
bool EmulatedControlBoard::positionMove(int n_joint, const int * joints, const double * refs)
#endif
{
    yCWarning(ECB) << "Group positionMove() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::relativeMove(int j, double delta)
#else
bool EmulatedControlBoard::relativeMove(int j, double delta)
#endif
{
    CHECK_JOINT(j);

    if (controlMode != VOCAB_CM_POSITION)
    {
        yCError(ECB, "EmulatedControlBoard will not relativeMove as not in positionMode");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return false;
#endif
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    double encExposed = getEncExposed(j);
    targetExposed[j]= encExposed + delta;

    if (std::abs(targetExposed[j] - encExposed) < m_jointTols[j])
    {
        stop(j);  // puts jointStatus[j]=0;
        yCInfo(ECB, "Joint q%d already at target", j + 1);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_ok;
#else
        return true;
#endif
    }
    else if (targetExposed[j] > encExposed)
    {
        // if(!velocityMove(j, m_refSpeeds[j])) return false;
        velRaw[j] = (m_refSpeeds[j] * m_velRawExposeds[j]);
    }
    else
    {
        // if(!velocityMove(j, -m_refSpeeds[j])) return false;
        velRaw[j] = -(m_refSpeeds[j] * m_velRawExposeds[j]);
    }

    jointStatus[j] = POSITION_MOVE;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::relativeMove(const double * deltas)  // encExposed = deltas + encExposed
#else
bool EmulatedControlBoard::relativeMove(const double * deltas)  // encExposed = deltas + encExposed
#endif
{
    // Check if we are in position mode.
    if (controlMode != VOCAB_CM_POSITION)
    {
        yCError(ECB, "Will not relativeMove as not in positionMode");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return false;
#endif
    }

    // Find out the maximum angle to move
    double max_dist = 0;
    double time_max_dist = 0;

    for (unsigned int motor = 0; motor < m_axes; motor++)
    {
        if (std::abs(deltas[motor]) > max_dist)
        {
            max_dist = std::abs(deltas[motor]);
            time_max_dist = max_dist / m_refSpeeds[motor];  // the max_dist motor will be at m_refSpeeds
        }
    }

    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    std::vector<double> encsExposed = getEncsExposed();

    for (unsigned int motor = 0; motor < m_axes; motor++)
    {
      targetExposed[motor] = encsExposed[motor] + deltas[motor];
      velRaw[motor] = ((deltas[motor]) / time_max_dist) * m_velRawExposeds[motor];
      yCInfo(ECB, "velRaw[%d]: %f", motor, velRaw[motor]);
      jointStatus[motor] = POSITION_MOVE;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::relativeMove(int n_joint, const int * joints, const double * deltas)
#else
bool EmulatedControlBoard::relativeMove(int n_joint, const int * joints, const double * deltas)
#endif
{
    yCWarning(ECB) << "Group relativeMove() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::checkMotionDone(int j, bool & flag)
#else
bool EmulatedControlBoard::checkMotionDone(int j, bool * flag)
#endif
{
    CHECK_JOINT(j);

    bool done = true;

    if (jointStatus[j] != NOT_CONTROLLING)
    {
        done = false;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = done;
    return yarp::dev::ReturnValue_ok;
#else
    *flag = done;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::checkMotionDone(bool & flag)
#else
bool EmulatedControlBoard::checkMotionDone(bool * flag)
#endif
{
    bool done = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        if (jointStatus[i] != NOT_CONTROLLING)
        {
            done = false;
        }
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = done;
    return yarp::dev::ReturnValue_ok;
#else
    *flag = done;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::checkMotionDone(const std::vector<int> & joints, bool & flag)
#else
bool EmulatedControlBoard::checkMotionDone(int n_joint, const int * joints, bool * flags)
#endif
{
    yCWarning(ECB) << "Group checkMotionDone() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setTrajSpeed(int j, double sp)
#else
bool EmulatedControlBoard::setRefSpeed(int j, double sp)
#endif
{
    CHECK_JOINT(j);
    m_refSpeeds[j] = sp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setTrajSpeeds(const double * spds)
#else
bool EmulatedControlBoard::setRefSpeeds(const double * spds)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= setTrajSpeed(i, spds[i]);
#else
        ok &= setRefSpeed(i, spds[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setTrajSpeeds(int n_joint, const int * joints, const double * spds)
#else
bool EmulatedControlBoard::setRefSpeeds(int n_joint, const int * joints, const double * spds)
#endif
{
    yCWarning(ECB) << "Group setTrajSpeeds() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setTrajAcceleration(int j, double acc)
#else
bool EmulatedControlBoard::setRefAcceleration(int j, double acc)
#endif
{
    CHECK_JOINT(j);
    refAcc[j] = acc;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setTrajAccelerations(const double * accs)
#else
bool EmulatedControlBoard::setRefAccelerations(const double * accs)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= setTrajAcceleration(i, accs[i]);
#else
        ok &= setRefAcceleration(i, accs[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setTrajAccelerations(int n_joint, const int * joints, const double * accs)
#else
bool EmulatedControlBoard::setRefAccelerations(int n_joint, const int * joints, const double * accs)
#endif
{
    yCWarning(ECB) << "Group setTrajAccelerations() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTrajSpeed(int j, double * ref)
#else
bool EmulatedControlBoard::getRefSpeed(int j, double * ref)
#endif
{
    CHECK_JOINT(j);
    *ref = m_refSpeeds[j];
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTrajSpeeds(double * spds)
#else
bool EmulatedControlBoard::getRefSpeeds(double * spds)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getTrajSpeed(i, &spds[i]);
#else
        ok &= getRefSpeed(i, &spds[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTrajSpeeds(int n_joint, const int * joints, double * spds)
#else
bool EmulatedControlBoard::getRefSpeeds(int n_joint, const int * joints, double * spds)
#endif
{
    yCWarning(ECB) << "Group getTrajSpeeds() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTrajAcceleration(int j, double * acc)
#else
bool EmulatedControlBoard::getRefAcceleration(int j, double * acc)
#endif
{
    CHECK_JOINT(j);
    *acc = refAcc[j];
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTrajAccelerations(double * accs)
#else
bool EmulatedControlBoard::getRefAccelerations(double * accs)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getTrajAcceleration(i, &accs[i]);
#else
        ok &= getRefAcceleration(i, &accs[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTrajAccelerations(int n_joint, const int * joints, double * accs)
#else
bool EmulatedControlBoard::getRefAccelerations(int n_joint, const int * joints, double * accs)
#endif
{
    yCWarning(ECB) << "Group getTrajAccelerations() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::stop(int j)
#else
bool EmulatedControlBoard::stop(int j)
#endif
{
    CHECK_JOINT(j);

    velRaw[j] = 0.0;
    jointStatus[j] = NOT_CONTROLLING;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::stop()
#else
bool EmulatedControlBoard::stop()
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= stop(i);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::stop(int n_joint, const int * joints)
#else
bool EmulatedControlBoard::stop(int n_joint, const int * joints)
#endif
{
    yCWarning(ECB) << "Group stop() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetPosition(int joint, double * ref)
#else
bool EmulatedControlBoard::getTargetPosition(int joint, double * ref)
#endif
{
    yCWarning(ECB) << "getTargetPosition() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetPositions(double * refs)
#else
bool EmulatedControlBoard::getTargetPositions(double * refs)
#endif
{
    yCWarning(ECB) << "getTargetPositions() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetPositions(int n_joint, const int * joints, double * refs)
#else
bool EmulatedControlBoard::getTargetPositions(int n_joint, const int * joints, double * refs)
#endif
{
    yCWarning(ECB) << "getTargetPositions() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
