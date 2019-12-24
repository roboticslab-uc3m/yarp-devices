// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IForceControl Related ------------------------------------

bool roboticslab::EmulatedControlboard::getRefTorques(double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefTorque(int j, double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefTorques(const double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefTorque(int j, double t)
{
    CD_DEBUG("joint: %d, refTorque: %f.\n", j, t);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTorque(int j, double *t)
{
    //CD_DEBUG("joint: %d.\n",j);  //-- Way too verbose
    *t = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTorques(double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTorqueRange(int j, double *min, double *max)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getTorqueRanges(double *min, double *max)
{
    return true;
}

// -----------------------------------------------------------------------------
