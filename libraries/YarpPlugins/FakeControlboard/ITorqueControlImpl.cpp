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
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setBemfParam(int j, double bemf)
{
    return true;
}

// -----------------------------------------------------------------------------
#endif // YARP_VERSION_MAJOR != 3
