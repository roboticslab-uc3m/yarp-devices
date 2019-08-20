// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>
#include <cstring>

#include "CanUtils.hpp"

// ######################## IVelocityControlRaw Related #############################

bool roboticslab::TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    CD_DEBUG("(%d),(%f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    if ( sp > maxVel )
    {
        CD_WARNING("Requested speed exceeds maximum velocity (%f).\n", maxVel);
        return false;
    }

    double value = degreesToInternalUnits(sp, 1);

    int16_t dataInt;
    uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    int32_t data = (dataInt << 16) + dataFrac;
    return sdoClient->download("Target velocity", data, 0x60FF);
}

// ----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################## IVelocityControlRaw2 Related ########################

bool roboticslab::TechnosoftIpos::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefVelocityRaw(const int joint, double *vel)
{
    CD_DEBUG("(%d)\n",joint);

    //-- Check index within range
    if ( joint != 0 ) return false;

    int32_t data;

    if (!sdoClient->upload("Target velocity", &data, 0x60FF))
    {
        return false;
    }

    int16_t dataInt = data >> 16;
    uint16_t dataFrac = data & 0xFFFF;
    double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);

    *vel = internalUnitsToDegrees(value, 1);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefVelocitiesRaw(double *vels)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------
