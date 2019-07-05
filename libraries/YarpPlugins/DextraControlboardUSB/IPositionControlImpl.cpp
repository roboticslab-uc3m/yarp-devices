// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <cstring>

#include <algorithm>

#include <ColorDebug.h>

// ############################## IPositionControl Related ##############################

bool roboticslab::DextraControlboardUSB::getAxes(int *ax)
{
    *ax = Synapse::DATA_POINTS;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::positionMove(int j, double ref)    // encExposed = ref;
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);

    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    setpoints[j] = ref;

    if (!synapse.writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoint(j, ref);
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::positionMove(const double *refs)
{
    CD_DEBUG("\n");

    Synapse::Setpoints setpoints;
    std::copy(refs, refs + Synapse::DATA_POINTS, setpoints);

    if (!synapse.writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::relativeMove(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (DextraControlboardUSB).\n");

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::relativeMove(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::checkMotionDone(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::checkMotionDone(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefSpeed(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefSpeeds(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefAcceleration(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefAccelerations(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefSpeed(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefSpeeds(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefAcceleration(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefAccelerations(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::stop(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::stop()
{
    CD_ERROR("\n");
    return false;
}


// ############################## IPositionControl2 Related ##############################


bool roboticslab::DextraControlboardUSB::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::stop(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTargetPosition(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTargetPositions(double *refs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}
