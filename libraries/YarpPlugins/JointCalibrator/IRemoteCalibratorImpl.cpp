// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <cmath>

#include <numeric>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    constexpr double MOTION_CHECK_INTERVAL = 0.1; // seconds
    constexpr double POSITION_EPSILON = 1e-3; // degrees
}

bool JointCalibrator::move(const std::vector<int> & joints, const MovementSpecs & specs)
{
    for (int joint : joints)
    {
        if (joint < 0 || joint > axes - 1)
        {
            CD_ERROR("Invalid joint id: %d.\n", joint);
            return false;
        }
    }

    std::vector<double> encs;

    if (!iEncoders->getEncoders(encs.data()))
    {
        CD_ERROR("Unable to retrieve initial position.\n");
        return false;
    }

    std::vector<int> ids;

    for (int joint : joints)
    {
        if (std::abs(encs[joint] - specs.pos[joint]) < POSITION_EPSILON)
        {
            CD_INFO("Joint %d already in target position.\n", joint);
            continue;
        }

        ids.push_back(joint);
    }

    if (ids.empty())
    {
        CD_INFO("All joints in target position, not moving.\n");
        return true;
    }

    std::vector<double> initialRefSpeeds(ids.size());

    if (!iPositionControl->getRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
    {
        CD_ERROR("Unable to retrieve initial reference speeds.\n");
        return false;
    }

    std::vector<double> initialRefAccs(ids.size());

    if (!iPositionControl->getRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
    {
        CD_ERROR("Unable to retrieve initial reference accelerations.\n");
        return false;
    }

    std::vector<yarp::conf::vocab32_t> targetModes(ids.size(), VOCAB_CM_POSITION);

    if (!iControlMode->setControlModes(ids.size(), ids.data(), targetModes.data()))
    {
        CD_ERROR("Unable to switch to position mode.\n");
        return false;
    }

    std::vector<double> targetRefSpeeds;
    std::vector<double> targetRefAccs;
    std::vector<double> targets;

    for (int id : ids)
    {
        targetRefSpeeds.push_back(specs.vel[id]);
        targetRefAccs.push_back(specs.acc[id]);
        targets.push_back(specs.pos[id]);
    }

    if (!iPositionControl->setRefSpeeds(ids.size(), ids.data(), targetRefSpeeds.data()))
    {
        CD_ERROR("Unable to set new reference speeds.\n");
        return false;
    }

    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), targetRefAccs.data()))
    {
        CD_ERROR("Unable to set new reference accelerations.\n");
        return false;
    }

    if (!iPositionControl->positionMove(ids.size(), ids.data(), targets.data()))
    {
        CD_ERROR("Unable to move motors to new position.\n");
        return false;
    }

    bool done;

    while (!iPositionControl->checkMotionDone(ids.size(), ids.data(), &done))
    {
        yarp::os::Time::delay(MOTION_CHECK_INTERVAL);
    }

    if (!iEncoders->getEncoders(encs.data()))
    {
        CD_ERROR("Unable to retrieve target position.\n");
        return false;
    }

    for (int id : ids)
    {
        if (std::abs(encs[id] - specs.pos[id]) > POSITION_EPSILON)
        {
            CD_ERROR("Joint %d has not reached the desired position.\n", id);
            return false;
        }
    }

    if (!iPositionControl->setRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
    {
        CD_ERROR("Unable to restore initial reference speeds.\n");
        return false;
    }

    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
    {
        CD_ERROR("Unable to restore initial reference accelerations.\n");
        return false;
    }

    return true;
}

bool JointCalibrator::calibrateSingleJoint(int j)
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool JointCalibrator::calibrateWholePart()
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool JointCalibrator::homingSingleJoint(int j)
{
    CD_INFO("Performing homing procedure on joint %d.\n", j);
    std::vector<int> targets{j};
    return move(targets, homeSpecs);
}

bool JointCalibrator::homingWholePart()
{
    CD_INFO("Performing homing procedure on whole part.\n");
    std::vector<int> targets(axes);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, homeSpecs);
}

bool JointCalibrator::parkSingleJoint(int j, bool wait)
{
    CD_INFO("Performing park procedure on joint %d.\n", j);
    std::vector<int> targets{j};
    return move(targets, parkSpecs);
}

bool JointCalibrator::parkWholePart()
{
    CD_INFO("Performing park procedure on whole part.\n");
    std::vector<int> targets(axes);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, parkSpecs);
}

bool JointCalibrator::quitCalibrate()
{
    CD_WARNING("Not supported.\n");
    return false;
}

bool JointCalibrator::quitPark()
{
    CD_WARNING("Not supported.\n");
    return false;
}
