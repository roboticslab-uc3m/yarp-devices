// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "QbRoboticsControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IPositionControl Related --------------------------------

bool QbRoboticsControlBoard::getAxes(int *ax)
{
    *ax = 2;
    return true;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::positionMove(int j, double ref)
{
    yCWarning(QBROB, "positionMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::positionMove(const double * refs)
{
    yCWarning(QBROB, "positionMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::relativeMove(int j, double delta)
{
    yCWarning(QBROB, "relativeMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::relativeMove(const double * deltas)
{
    yCWarning(QBROB, "relativeMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::checkMotionDone(int j, bool * flag)
{
    yCWarning(QBROB, "checkMotionDone() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::checkMotionDone(bool * flag)
{
    yCWarning(QBROB, "checkMotionDone() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setRefSpeed(int j, double sp)
{
    yCWarning(QBROB, "setRefSpeed() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setRefSpeeds(const double * spds)
{
    yCWarning(QBROB, "setRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setRefAcceleration(int j, double acc)
{
    yCWarning(QBROB, "setRefAcceleration() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setRefAccelerations(const double * accs)
{
    yCWarning(QBROB, "setRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefSpeed(int j, double * ref)
{
    yCWarning(QBROB, "getRefSpeed() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefSpeeds(double * spds)
{
    yCWarning(QBROB, "getRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefAcceleration(int j, double * acc)
{
    yCWarning(QBROB, "getRefAcceleration() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefAccelerations(double * accs)
{
    yCWarning(QBROB, "getRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::stop(int j)
{
    yCWarning(QBROB, "stop() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::stop()
{
    yCWarning(QBROB, "stop() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::positionMove(int n_joint, const int * joints, const double * refs)
{
    yCWarning(QBROB, "Group positionMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::relativeMove(int n_joint, const int * joints, const double * deltas)
{
    yCWarning(QBROB, "Group relativeMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::checkMotionDone(int n_joint, const int * joints, bool * flags)
{
    yCWarning(QBROB, "Group checkMotionDone() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setRefSpeeds(int n_joint, const int * joints, const double * spds)
{
    yCWarning(QBROB, "Group setRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setRefAccelerations(int n_joint, const int * joints, const double * accs)
{
    yCWarning(QBROB, "Group setRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefSpeeds(int n_joint, const int * joints, double * spds)
{
    yCWarning(QBROB, "Group getRefSpeeds() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefAccelerations(int n_joint, const int * joints, double * accs)
{
    yCWarning(QBROB, "Group getRefAccelerations() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::stop(int n_joint, const int * joints)
{
    yCWarning(QBROB, "Group stop() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getTargetPosition(int joint, double * ref)
{
    yCWarning(QBROB, "getTargetPosition() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getTargetPositions(double * refs)
{
    yCWarning(QBROB, "getTargetPositions() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getTargetPositions(int n_joint, const int * joints, double * refs)
{
    yCWarning(QBROB, "getTargetPositions() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------
