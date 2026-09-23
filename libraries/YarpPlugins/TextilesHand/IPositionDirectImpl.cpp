// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getAxes(std::size_t & ax)
{
    ax = 1;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#else
bool TextilesHand::getAxes(int * ax)
{
    *ax = 1;
    return true;
}
#endif

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::setPosition(int j, double ref)
#else
bool TextilesHand::setPosition(int j, double ref)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (j != 0) return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
    if (j != 0) return false;
#endif

    char cmdByte[1];

    if (ref == 0.0)
    {
        cmdByte[0] = 'a';
    }
    else if (ref == 1.0)
    {
        cmdByte[0] = 'b';
    }
    else
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    if (!iSerialDevice->send(cmdByte, 1))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    lastTarget = ref;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::setPositions(const double * refs)
#else
bool TextilesHand::setPositions(const double * refs)
#endif
{
    return setPosition(0, refs[0]);
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::setPositions(int n_joint, const int * joints, const double * refs)
#else
bool TextilesHand::setPositions(int n_joint, const int * joints, const double * refs)
#endif
{
    return setPosition(joints[0], refs[0]);
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getRefPosition(int joint, double * ref)
#else
bool TextilesHand::getRefPosition(int joint, double * ref)
#endif
{
    *ref = lastTarget;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getRefPositions(double * refs)
#else
bool TextilesHand::getRefPositions(double * refs)
#endif
{
    return getRefPosition(0, &refs[0]);
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getRefPositions(int n_joint, const int * joints, double * refs)
#else
bool TextilesHand::getRefPositions(int n_joint, const int * joints, double * refs)
#endif
{
    return getRefPosition(joints[0], &refs[0]);
}

// ----------------------------------------------------------------------------------------
