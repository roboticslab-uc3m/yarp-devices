// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.hpp>

// ------------------- IForceControl Related ------------------------------------

bool roboticslab::FakeControlboard::getRefTorques(double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefTorque(int j, double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefTorques(const double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefTorque(int j, double t)
{
    CD_DEBUG("joint: %d, refTorque: %f.\n", j, t);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorque(int j, double *t)
{
    //CD_DEBUG("joint: %d.\n",j);  //-- Way too verbose
    *t = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorques(double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueRange(int j, double *min, double *max)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueRanges(double *min, double *max)
{
    return true;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_MAJOR != 3
bool roboticslab::FakeControlboard::getBemfParam(int j, double *bemf)
{
    yarp::dev::MotorTorqueParameters params;

    if (!getMotorTorqueParams(j, &params))
    {
        return false;
    }

    *bemf = params.bemf;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setBemfParam(int j, double bemf)
{
    yarp::dev::MotorTorqueParameters params;

    if (!getMotorTorqueParams(j, &params))
    {
        return false;
    }

    params.bemf = bemf;

    return setMotorTorqueParams(j, params);
}

// -----------------------------------------------------------------------------
#endif // YARP_VERSION_MAJOR != 3
