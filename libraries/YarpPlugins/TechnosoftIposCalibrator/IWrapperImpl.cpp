// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposCalibrator.hpp"

using namespace roboticslab;

bool TechnosoftIposCalibrator::attach(yarp::dev::PolyDriver * poly)
{
    return poly->view(iControlMode)
            && poly->view(iEncoders)
            && poly->view(iPositionControl)
            && iEncoders->getAxes(&axes);
}

bool TechnosoftIposCalibrator::detach()
{
    return true;
}
