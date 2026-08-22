// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <yarp/conf/version.h>

bool JointCalibrator::attach(yarp::dev::PolyDriver * poly)
{
    std::size_t localAxes;

    return poly->view(iControlMode)
        && poly->view(iEncoders)
        && poly->view(iPositionControl)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        && iEncoders->getAxes(localAxes)
#else
        && iEncoders->getAxes(reinterpret_cast<int *>(&localAxes))
#endif
        && localAxes == m_joints;
}

bool JointCalibrator::detach()
{
    return true;
}
