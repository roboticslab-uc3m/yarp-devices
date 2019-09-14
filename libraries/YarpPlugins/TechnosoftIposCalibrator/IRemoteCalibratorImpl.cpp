// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposCalibrator.hpp"

using namespace roboticslab;

#include <vector>

#include <ColorDebug.h>

bool TechnosoftIposCalibrator::calibrateSingleJoint(int j)
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool TechnosoftIposCalibrator::calibrateWholePart()
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool TechnosoftIposCalibrator::homingSingleJoint(int j)
{
    if (j < 0 || j > axes - 1)
    {
        CD_ERROR("Invalid joint id: %d.\n", j);
        return false;
    }

    CD_INFO("Starting homing procedure on joint %d.\n", j);

    if (!iControlMode->setControlMode(j, VOCAB_CM_POSITION))
    {
        CD_ERROR("Unable to switch to position mode.\n");
        return false;
    }

    double enc;

    if (!iEncoders->getEncoder(j, &enc))
    {
        CD_ERROR("Unable to retrieve current position.\n");
        return false;
    }

    CD_INFO("Current position: %f.\n", enc);

    if (!iPositionControl->positionMove(j, 0.0))
    {
        CD_ERROR("Unable to move motors to zero.\n");
        return false;
    }

    bool done;

    while (!iPositionControl->checkMotionDone(j, &done))
    {
        yarp::os::Time::delay(0.1);
    }

    CD_INFO("Homing procedure on joint %d finished.\n", j);

    return true;
}

bool TechnosoftIposCalibrator::homingWholePart()
{
    CD_INFO("Starting homing procedure.\n");

    std::vector<yarp::conf::vocab32_t> modes(axes, VOCAB_CM_POSITION);

    if (!iControlMode->setControlModes(modes.data()))
    {
        CD_ERROR("Unable to switch to position mode.\n");
        return false;
    }

    std::vector<double> encs(axes);

    if (!iEncoders->getEncoders(encs.data()))
    {
        CD_ERROR("Unable to retrieve current position.\n");
        return false;
    }

    CD_INFO("Current position:");

    for (auto enc : encs)
    {
        CD_INFO_NO_HEADER(" %f", enc);
    }

    CD_INFO_NO_HEADER("\n");

    std::vector<double> poss(axes, 0.0);

    if (!iPositionControl->positionMove(poss.data()))
    {
        CD_ERROR("Unable to move motors to zero.\n");
        return false;
    }

    bool done;

    while (!iPositionControl->checkMotionDone(&done))
    {
        yarp::os::Time::delay(0.1);
    }

    CD_INFO("Homing procedure finished.\n");

    return true;
}

bool TechnosoftIposCalibrator::parkSingleJoint(int j, bool wait)
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool TechnosoftIposCalibrator::parkWholePart()
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool TechnosoftIposCalibrator::quitCalibrate()
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool TechnosoftIposCalibrator::quitPark()
{
    CD_WARNING("Not supported.\n");
    return false;
}
