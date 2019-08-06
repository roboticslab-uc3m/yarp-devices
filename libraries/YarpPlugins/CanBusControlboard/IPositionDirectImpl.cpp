// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool roboticslab::CanBusControlboard::setPosition(int j, double ref)
{
    CD_DEBUG("\n");

    return iPositionDirectRaw[j]->setPositionRaw(0, ref);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("n_joint:%d, drivers.size():" CD_SIZE_T "\n",n_joint,nodes.size());

    bool ok = true;

    for (unsigned int i = 0; i < n_joint; i++)
    {
        ok &= this->setPosition(joints[i], refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setPositions(const double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        ok &= this->setPosition(i, refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefPosition(const int joint, double *ref)
{
    CD_DEBUG("(%d)\n", joint);

    return iPositionDirectRaw[joint]->getRefPositionRaw(0, ref);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefPositions(double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        ok &= this->getRefPosition(i, &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (unsigned int i = 0; i < n_joint; i++)
    {
        ok &= this->getRefPosition(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
