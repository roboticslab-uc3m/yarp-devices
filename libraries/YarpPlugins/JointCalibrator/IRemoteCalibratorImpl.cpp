// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <cmath>

#include <numeric>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr double MOTION_CHECK_INTERVAL = 0.1; // seconds
constexpr double POSITION_EPSILON = 0.01; // degrees

bool JointCalibrator::move(const std::vector<int> & joints, const MovementSpecs & specs)
{
    for (int joint : joints)
    {
        if (joint < 0 || joint > axes - 1)
        {
            yCError(JC) << "Invalid joint id: %d" << joint;
            return false;
        }
    }

    std::vector<double> encs(axes);

    if (!iEncoders->getEncoders(encs.data()))
    {
        yCError(JC) << "Unable to retrieve initial position";
        return false;
    }

    std::vector<int> ids;

    for (int joint : joints)
    {
        if (std::abs(encs[joint] - specs.pos[joint]) < POSITION_EPSILON)
        {
            yCInfo(JC) << "Joint" << joint << "already in target position";
            continue;
        }

        ids.push_back(joint);
    }

    if (ids.empty())
    {
        yCInfo(JC) << "All joints in target position, not moving";
        return true;
    }

    std::vector<double> initialRefSpeeds(ids.size());

    if (!iPositionControl->getRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
    {
        yCError(JC) << "Unable to retrieve initial reference speeds";
        return false;
    }

    std::vector<double> initialRefAccs(ids.size());

    if (!iPositionControl->getRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
    {
        yCError(JC) << "Unable to retrieve initial reference accelerations";
        return false;
    }

    std::vector<yarp::conf::vocab32_t> targetModes(ids.size(), VOCAB_CM_POSITION);

    if (!iControlMode->setControlModes(ids.size(), ids.data(), targetModes.data()))
    {
        yCError(JC) << "Unable to switch to position mode";
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
        yCError(JC) << "Unable to set new reference speeds";
        return false;
    }

    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), targetRefAccs.data()))
    {
        yCError(JC) << "Unable to set new reference accelerations";
        return false;
    }

    if (!iPositionControl->positionMove(ids.size(), ids.data(), targets.data()))
    {
        yCError(JC) << "Unable to move motors to new position";
        return false;
    }

    if (!isBlocking)
    {
        return true;
    }

    bool ok = true;
    bool done = false;

    do
    {
        yarp::os::SystemClock::delaySystem(MOTION_CHECK_INTERVAL);

        if (!iPositionControl->checkMotionDone(ids.size(), ids.data(), &done))
        {
            yCWarning(JC) << "Unable to check motion completion";
            ok = false;
            break;
        }
    }
    while (!done);

    if (!iEncoders->getEncoders(encs.data()))
    {
        yCWarning(JC) << "Unable to retrieve target position";
        ok = false;
    }

    for (int id : ids)
    {
        if (std::abs(encs[id] - specs.pos[id]) > POSITION_EPSILON)
        {
            yCWarning(JC) << "Joint" << id << "has not reached the desired position";
            ok = false;
        }
    }

    if (!iPositionControl->setRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
    {
        yCWarning(JC) << "Unable to restore initial reference speeds";
        ok = false;
    }

    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
    {
        yCWarning(JC) << "Unable to restore initial reference accelerations";
        ok = false;
    }

    return ok;
}

bool JointCalibrator::calibrateSingleJoint(int j)
{
    yCWarning(JC) << "calibrateSingleJoint() not supported";
    return false;
}

bool JointCalibrator::calibrateWholePart()
{
    yCWarning(JC) << "calibrateWholePart() not supported";
    return false;
}

bool JointCalibrator::homingSingleJoint(int j)
{
    yCInfo(JC) << "Performing homing procedure on joint" << j;
    std::vector<int> targets{j};
    return move(targets, homeSpecs);
}

bool JointCalibrator::homingWholePart()
{
    yCInfo(JC) << "Performing homing procedure on whole part";
    std::vector<int> targets(axes);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, homeSpecs);
}

bool JointCalibrator::parkSingleJoint(int j, bool wait)
{
    yCInfo(JC) << "Performing park procedure on joint" << j;
    std::vector<int> targets{j};
    return move(targets, parkSpecs);
}

bool JointCalibrator::parkWholePart()
{
    yCInfo(JC) << "Performing park procedure on whole part";
    std::vector<int> targets(axes);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, parkSpecs);
}

bool JointCalibrator::quitCalibrate()
{
    yCWarning(JC) << "quitCalibrate() not supported";
    return false;
}

bool JointCalibrator::quitPark()
{
    yCWarning(JC) << "quitPark() not supported";
    return false;
}
