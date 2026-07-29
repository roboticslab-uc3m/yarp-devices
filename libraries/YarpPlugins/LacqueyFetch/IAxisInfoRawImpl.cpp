// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::getAxes(int * ax)
{
    return getNumberOfMotorsRaw(ax);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getAxisNameRaw(int axis, std::string & name)
{
    CHECK_JOINT(axis);
    name = m_name;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
    CHECK_JOINT(axis);
    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
