// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <cmath> // std::abs

#include <numeric> // std::iota

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

constexpr double MOTION_CHECK_INTERVAL = 0.1; // seconds
constexpr double POSITION_EPSILON = 0.01; // degrees

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::move(const std::vector<int> & joints, const MovementSpecs & specs)
#else
bool JointCalibrator::move(const std::vector<int> & joints, const MovementSpecs & specs)
#endif
{
    for (int joint : joints)
    {
        if (joint < 0 || joint > m_joints - 1)
        {
            yCError(JC) << "Invalid joint id: %d" << joint;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return false;
#endif
        }
    }

    std::vector<double> encs(m_joints);

    if (!iEncoders->getEncoders(encs.data()))
    {
        yCError(JC) << "Unable to retrieve initial position";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
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
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    std::vector<double> initialRefSpeeds(ids.size());

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->getTrajSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
#else
    if (!iPositionControl->getRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
#endif
    {
        yCError(JC) << "Unable to retrieve initial reference speeds";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    std::vector<double> initialRefAccs(ids.size());

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->getTrajAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
#else
    if (!iPositionControl->getRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
#endif
    {
        yCError(JC) << "Unable to retrieve initial reference accelerations";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iControlMode->setControlModes(ids, std::vector(ids.size(), yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION)))
#else
    if (std::vector<yarp::conf::vocab32_t> targetModes(ids.size(), VOCAB_CM_POSITION);
        !iControlMode->setControlModes(ids.size(), ids.data(), targetModes.data()))
#endif
    {
        yCError(JC) << "Unable to switch to position mode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->setTrajSpeeds(ids.size(), ids.data(), targetRefSpeeds.data()))
#else
    if (!iPositionControl->setRefSpeeds(ids.size(), ids.data(), targetRefSpeeds.data()))
#endif
    {
        yCError(JC) << "Unable to set new reference speeds";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->setTrajAccelerations(ids.size(), ids.data(), targetRefAccs.data()))
#else
    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), targetRefAccs.data()))
#endif
    {
        yCError(JC) << "Unable to set new reference accelerations";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    if (!iPositionControl->positionMove(ids.size(), ids.data(), targets.data()))
    {
        yCError(JC) << "Unable to move motors to new position";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    if (!m_block)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    bool ok = true;
    bool done = false;

    do
    {
        yarp::os::SystemClock::delaySystem(MOTION_CHECK_INTERVAL);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        if (!iPositionControl->checkMotionDone(ids, done))
#else
        if (!iPositionControl->checkMotionDone(ids.size(), ids.data(), &done))
#endif
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->setTrajSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
#else
    if (!iPositionControl->setRefSpeeds(ids.size(), ids.data(), initialRefSpeeds.data()))
#endif
    {
        yCWarning(JC) << "Unable to restore initial reference speeds";
        ok = false;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->setTrajAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
#else
    if (!iPositionControl->setRefAccelerations(ids.size(), ids.data(), initialRefAccs.data()))
#endif
    {
        yCWarning(JC) << "Unable to restore initial reference accelerations";
        ok = false;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue::return_code::return_value_ok
              : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::calibrateSingleJoint(int j)
#else
bool JointCalibrator::calibrateSingleJoint(int j)
#endif
{
    yCWarning(JC) << "calibrateSingleJoint() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::calibrateWholePart()
#else
bool JointCalibrator::calibrateWholePart()
#endif
{
    yCWarning(JC) << "calibrateWholePart() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::homingSingleJoint(int j)
#else
bool JointCalibrator::homingSingleJoint(int j)
#endif
{
    yCInfo(JC) << "Performing homing procedure on joint" << j;
    std::vector<int> targets{j};
    return move(targets, homeSpecs);
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::homingWholePart()
#else
bool JointCalibrator::homingWholePart()
#endif
{
    yCInfo(JC) << "Performing homing procedure on whole part";
    std::vector<int> targets(m_joints);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, homeSpecs);
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::parkSingleJoint(int j, bool wait)
#else
bool JointCalibrator::parkSingleJoint(int j, bool wait)
#endif
{
    yCInfo(JC) << "Performing park procedure on joint" << j;
    std::vector<int> targets{j};
    return move(targets, parkSpecs);
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::parkWholePart()
#else
bool JointCalibrator::parkWholePart()
#endif
{
    yCInfo(JC) << "Performing park procedure on whole part";
    std::vector<int> targets(m_joints);
    std::iota(targets.begin(), targets.end(), 0);
    return move(targets, parkSpecs);
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::quitCalibrate()
#else
bool JointCalibrator::quitCalibrate()
#endif
{
    yCWarning(JC) << "quitCalibrate() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue JointCalibrator::quitPark()
#else
bool JointCalibrator::quitPark()
#endif
{
    yCWarning(JC) << "quitPark() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}
