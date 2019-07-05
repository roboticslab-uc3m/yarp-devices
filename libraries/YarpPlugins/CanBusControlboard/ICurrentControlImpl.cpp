// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ ICurrentControl Related -----------------------------------------

bool roboticslab::CanBusControlboard::getNumberOfMotors(int *ax)
{
    CD_DEBUG("\n");
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getCurrent(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if ( ! this->indexWithinRange(m) ) return false;

    return iCurrentControlRaw[m]->getCurrentRaw(0, curr);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getCurrents(double *currs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getCurrentRange(int m, double *min, double *max)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if ( ! this->indexWithinRange(m) ) return false;

    return iCurrentControlRaw[m]->getCurrentRangeRaw(0, min, max);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getCurrentRanges(double *min, double *max)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefCurrents(const double *currs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefCurrent(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if ( ! this->indexWithinRange(m) ) return false;

    return iCurrentControlRaw[m]->setRefCurrentRaw(0, curr);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefCurrents(const int n_motor, const int *motors, const double *currs)
{
    CD_DEBUG("(%d)\n", n_motor);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefCurrents(double *currs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefCurrent(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if ( ! this->indexWithinRange(m) ) return false;

    return iCurrentControlRaw[m]->getRefCurrentRaw(0, curr);
}

// -----------------------------------------------------------------------------
