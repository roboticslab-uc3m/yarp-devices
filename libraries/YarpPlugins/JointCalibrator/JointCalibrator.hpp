// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JOINT_CALIBRATOR_HPP__
#define __JOINT_CALIBRATOR_HPP__

#include <vector>

#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>

#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR >= 3
# include <yarp/dev/IWrapper.h>
#else
# include <yarp/dev/Wrapper.h>
#endif

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup JointCalibrator
 * @brief Contains roboticslab::JointCalibrator.
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
                        public yarp::dev::IWrapper
{
public:
    JointCalibrator()
        : axes(0), iControlMode(nullptr), iEncoders(nullptr), iPositionControl(nullptr)
    { }

    virtual bool calibrateSingleJoint(int j) override;
    virtual bool calibrateWholePart() override;
    virtual bool homingSingleJoint(int j) override;
    virtual bool homingWholePart() override;
    virtual bool parkSingleJoint(int j, bool wait) override;
    virtual bool parkWholePart() override;
    virtual bool quitCalibrate() override;
    virtual bool quitPark() override;

    virtual bool attach(yarp::dev::PolyDriver * poly) override;
    virtual bool detach() override;

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

private:
    bool move(const std::vector<int> & joints, const MovementSpecs & specs);

    int axes;

    MovementSpecs homeSpecs;
    MovementSpecs parkSpecs;

    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionControl * iPositionControl;
};

}  // namespace roboticslab

#endif // __JOINT_CALIBRATOR_HPP__
