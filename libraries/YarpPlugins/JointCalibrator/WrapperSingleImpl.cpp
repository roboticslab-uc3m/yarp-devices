// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

bool JointCalibrator::attach(yarp::dev::PolyDriver * poly)
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    std::size_t localAxes;
#else
    int localAxes;
#endif

    return poly->view(iControlMode)
        && poly->view(iEncoders)
        && poly->view(iPositionControl)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        && iEncoders->getAxes(localAxes)
#else
        && iEncoders->getAxes(&localAxes)
#endif
        && localAxes == m_joints;
}

bool JointCalibrator::detach()
{
    return true;
}
