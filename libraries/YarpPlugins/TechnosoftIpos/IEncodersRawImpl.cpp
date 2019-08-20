// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -------------------------- IEncodersRaw Related ----------------------------------

bool roboticslab::TechnosoftIpos::resetEncoderRaw(int j)
{
    CD_INFO("(%d)\n", j);
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0);
}

bool roboticslab::TechnosoftIpos::resetEncodersRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setEncoderRaw(int j, double val)
{
    CD_INFO("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    int32_t data = degreesToInternalUnits(val);
    return sdoClient->download("Set actual position", data, 0X2081);
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setEncodersRaw(const double *vals)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderRaw(int j, double *v)
{
    //CD_INFO("%d\n", j);  //-- Too verbose in stream.
    CHECK_JOINT(j);

    if (!iEncodersTimedRawExternal)
    {
        return sdoClient->upload<int32_t>("Position actual value", [=](int32_t * data)
                {
                    lastEncoderRead.update(internalUnitsToDegrees(*data));
                    *v = lastEncoderRead.queryPosition();
                },
                0x6064);
    }

    iEncodersTimedRawExternal->getEncoderRaw(0, v);
    lastEncoderRead.update(*v);
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncodersRaw(double *encs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_INFO("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *sp = lastEncoderRead.querySpeed();
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderAccelerationRaw(int j, double *acc)
{
    //CD_INFO("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *acc = lastEncoderRead.queryAcceleration();
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
