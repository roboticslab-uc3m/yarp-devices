// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JOINT_CALIBRATOR_HPP__
#define __JOINT_CALIBRATOR_HPP__

#include <vector>

#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/WrapperSingle.h>

#include "JointCalibrator_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup JointCalibrator
 * @brief Contains JointCalibrator.
 */

/**
 * @ingroup JointCalibrator
 * @brief Stores movement specifications: position, velocity, acceleration.
 */
struct MovementSpecs
{
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> acc;
};

/**
 * @ingroup JointCalibrator
 * @brief Remote calibrator class for multi-joint homing and park.
 */
class JointCalibrator : public yarp::dev::DeviceDriver,
                        public yarp::dev::IRemoteCalibrator,
                        public yarp::dev::WrapperSingle,
                        public JointCalibrator_ParamsParser
{
public:
    bool calibrateSingleJoint(int j) override;
    bool calibrateWholePart() override;
    bool homingSingleJoint(int j) override;
    bool homingWholePart() override;
    bool parkSingleJoint(int j, bool wait) override;
    bool parkWholePart() override;
    bool quitCalibrate() override;
    bool quitPark() override;

    bool attach(yarp::dev::PolyDriver * poly) override;
    bool detach() override;

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    bool move(const std::vector<int> & joints, const MovementSpecs & specs);

    MovementSpecs homeSpecs;
    MovementSpecs parkSpecs;

    yarp::dev::IControlMode * iControlMode {nullptr};
    yarp::dev::IEncoders * iEncoders {nullptr};
    yarp::dev::IPositionControl * iPositionControl {nullptr};
};

#endif // __JOINT_CALIBRATOR_HPP__
