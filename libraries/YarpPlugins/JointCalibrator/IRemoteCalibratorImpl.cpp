// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <cmath>

#include <numeric>

#include <yarp/os/LogStream.h>

using namespace roboticslab;

namespace
{
    constexpr double MOTION_CHECK_INTERVAL = 0.1; // seconds
    constexpr double POSITION_EPSILON = 0.01; // degrees
}

bool JointCalibrator::move(const std::vector<int> & joints, const MovementSpecs & specs)
{
    for (int joint : joints)
    {
        if (joint < 0 || joint > axes - 1)
        {
            yError() << "Invalid joint id: %d" << joint;
            return false;
        }
    }

    std::vector<double> encs(axes);

    if (!iEncoders->getEncoders(encs.data()))
    {
        yError() << "Unable to retrieve initial position";
        return false;
    }

    std::vector<int> ids;

    for (int joint : joints)
    {
        if (std::abs(encs[joint] - specs.pos[joint]) < POSITION_EPSILON)
        {
            yInfo() << "Joint" << joint << "already in target position";
            continue;
        }

        ids.push_back(joint);
    }

    if (ids.empty())
    {
        yInfo() << "All joints in target position, not moving";
        return true;
    }

    std::vector<double> initialRefSpeeds(ids.size());

    if (!iPositionControl->getRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
    {
        yError() << "Unable to retrieve initial reference speeds";
        return false;
    }

    std::vector<double> initialRefAccs(ids.size());

    if (!iPositionControl->getRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
    {
        yError() << "Unable to retrieve initial reference accelerations";
        return false;
    }

    std::vector<yarp::conf::vocab32_t> targetModes(ids.size(), VOCAB_CM_POSITION);

    if (!iControlMode->setControlModes(ids.size(), ids.data(), targetModes.data()))
    {
        yError() << "Unable to switch to position mode";
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
        yError() << "Unable to set new reference speeds";
        return false;
    }

    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), targetRefAccs.data()))
    {
        yError() << "Unable to set new reference accelerations";
        return false;
    }

    if (!iPositionControl->positionMove(ids.size(), ids.data(), targets.data()))
    {
        yError() << "Unable to move motors to new position";
        return false;
    }

    bool ok = true;
    bool done = false;

    do
    {
        yarp::os::Time::delay(MOTION_CHECK_INTERVAL);

        if (!iPositionControl->checkMotionDone(ids.size(), ids.data(), &done))
        {
            yWarning() << "Unable to check motion completion";
            ok = false;
            break;
        }
    }
    while (!done);

    if (!iEncoders->getEncoders(encs.data()))
    {
        yWarning() << "Unable to retrieve target position";
        ok = false;
    }

    for (int id : ids)
    {
        if (std::abs(encs[id] - specs.pos[id]) > POSITION_EPSILON)
        {
            yWarning() << "Joint" << id << "has not reached the desired position";
            ok = false;
        }
    }

    if (!iPositionControl->setRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
    {
        yWarning() << "Unable to restore initial reference speeds";
        ok = false;
    }

    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
    {
        yWarning() << "Unable to restore initial reference accelerations";
        ok = false;
    }

    return ok;
}

bool JointCalibrator::calibrateSingleJoint(int j)
{
    yWarning() << "calibrateSingleJoint() not supported";
    return false;
}

bool JointCalibrator::calibrateWholePart()
{
    yWarning() << "calibrateWholePart() not supported";
    return false;
}

bool JointCalibrator::homingSingleJoint(int j)
{
    yInfo() << "Performing homing procedure on joint" << j;
    std::vector<int> targets{j};
    return move(targets, homeSpecs);
}

bool JointCalibrator::homingWholePart()
{
    yInfo() << "Performing homing procedure on whole part";
    std::vector<int> targets(axes);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, homeSpecs);
}

bool JointCalibrator::parkSingleJoint(int j, bool wait)
{
    yInfo() << "Performing park procedure on joint" << j;
    std::vector<int> targets{j};
    return move(targets, parkSpecs);
}

bool JointCalibrator::parkWholePart()
{
    yInfo() << "Performing park procedure on whole part";
    std::vector<int> targets(axes);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, parkSpecs);
}

bool JointCalibrator::quitCalibrate()
{
    yWarning() << "quitCalibrate() not supported";
    return false;
}

bool JointCalibrator::quitPark()
{
    yWarning() << "quitPark() not supported";
    return false;
}
