// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ############################# ITorqueControlRaw Related #############################

bool roboticslab::TechnosoftIpos::getRefTorquesRaw(double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *t = refTorque;

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefTorquesRaw(const double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefTorqueRaw(int j, double t)
{
    CD_INFO("(%d,%f)\n",j,t);

    //-- Check index within range
    if ( j!= 0 ) return false;

    //*************************************************************
    uint8_t msg_ref_torque[]= {0x23,0x1C,0x20,0x00,0x00,0x00,0x00,0x00}; // put 23 because it is a target

    int sendRefTorque = t * (65520.0/20.0) / (this->tr * this->k);  // Page 109 of 263, supposing 10 Ipeak.
    //memcpy(msg_ref_torque+4,&sendRefTorque,4);  // was +6 not +4, but +6 seems terrible with 4!
    memcpy(msg_ref_torque+6,&sendRefTorque,2);

    if(! send(0x600, 8, msg_ref_torque) )
    {
        CD_ERROR("Could not send refTorque. %s\n", msgToStr(0x600, 8, msg_ref_torque).c_str() );
        return false;
    }
    CD_SUCCESS("Sent refTorque. %s\n", msgToStr(0x600, 8, msg_ref_torque).c_str() );
    //*************************************************************

    refTorqueSemaphore.wait();
    refTorque = t;
    refTorqueSemaphore.post();


    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("(%d,%f)\n",j,bemf);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueRaw(int j, double *t)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_getCurrent[]= {0x40,0x7E,0x20,0x00}; // Query current. Ok only 4.

    if(! send(0x600, 4, msg_getCurrent) )
    {
        CD_ERROR("Could not send msg_getCurrent. %s\n", msgToStr(0x600, 4, msg_getCurrent).c_str() );
        return false;
    }
    //CD_SUCCESS("Sent msg_getCurrent. %s\n", msgToStr(0x600, 4, msg_getCurrent).c_str() );    //-- Too verbose in controlboardwrapper2 stream.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    yarp::os::Time::delay(DELAY);  // Must delay as it will be from same driver.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    getTorqueReady.wait();
    *t = getTorque;
    getTorqueReady.post();

    //*************************************************************
    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorquesRaw(double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("(%d,%f)\n",j,limit);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::resetTorquePidRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::disableTorquePidRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::enableTorquePidRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("(%d,%f)\n",j,v);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

    return true;
}

// -------------------------------------------------------------------------------------
