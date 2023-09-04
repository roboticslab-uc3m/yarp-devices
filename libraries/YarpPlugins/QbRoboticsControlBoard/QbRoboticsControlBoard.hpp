// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __QB_ROBOTICS_CONTROL_BOARD_HPP__
#define __QB_ROBOTICS_CONTROL_BOARD_HPP__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup QbRoboticsControlBoard
 * @brief Contains roboticslab::QbRoboticsControlBoard.
 */

/**
 * @ingroup QbRoboticsControlBoard
 * @brief Implements several motor interfaces.
 */
class QbRoboticsControlBoard : public yarp::dev::DeviceDriver,
                               public yarp::dev::IEncodersTimed,
                               public yarp::dev::IPositionControl,
                               public yarp::dev::IVelocityControl
{
public:

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    bool getAxes(int *ax) override;
    bool positionMove(int j, double ref) override;
    bool positionMove(const double *refs) override;
    bool relativeMove(int j, double delta) override;
    bool relativeMove(const double *deltas) override;
    bool checkMotionDone(int j, bool *flag) override;
    bool checkMotionDone(bool *flag) override;
    bool setRefSpeed(int j, double sp) override;
    bool setRefSpeeds(const double *spds) override;
    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double *accs) override;
    bool getRefSpeed(int j, double *ref) override;
    bool getRefSpeeds(double *spds) override;
    bool getRefAcceleration(int j, double *acc) override;
    bool getRefAccelerations(double *accs) override;
    bool stop(int j) override;
    bool stop() override;
    bool positionMove(int n_joint, const int *joints, const double *refs) override;
    bool relativeMove(int n_joint, const int *joints, const double *deltas) override;
    bool checkMotionDone(int n_joint, const int *joints, bool *flags) override;
    bool setRefSpeeds(int n_joint, const int *joints, const double *spds) override;
    bool setRefAccelerations(int n_joint, const int *joints, const double *accs) override;
    bool getRefSpeeds(int n_joint, const int *joints, double *spds) override;
    bool getRefAccelerations(int n_joint, const int *joints, double *accs) override;
    bool stop(int n_joint, const int *joints) override;
    bool getTargetPosition(int joint, double *ref) override;
    bool getTargetPositions(double *refs) override;
    bool getTargetPositions(int n_joint, const int *joints, double *refs) override;

    // ---------- IEncodersTimed Declarations. Implementation in IEncoderImpl.cpp ----------

    bool resetEncoder(int j) override;
    bool resetEncoders() override;
    bool setEncoder(int j, double val) override;
    bool setEncoders(const double *vals) override;
    bool getEncoder(int j, double *v) override;
    bool getEncoders(double *encs) override;
    bool getEncoderSpeed(int j, double *sp) override;
    bool getEncoderSpeeds(double *spds) override;
    bool getEncoderAcceleration(int j, double *spds) override;
    bool getEncoderAccelerations(double *accs) override;
    bool getEncodersTimed(double *encs, double *time) override;
    bool getEncoderTimed(int j, double *encs, double *time) override;

    // --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double *sp) override;
    bool velocityMove(int n_joint, const int *joints, const double *spds) override;
    bool getRefVelocity(int joint, double *vel) override;
    bool getRefVelocities(double *vels) override;
    bool getRefVelocities(int n_joint, const int *joints, double *vels) override;

private:
};

} // namespace roboticslab

#endif // __QB_ROBOTICS_CONTROL_BOARD_HPP__
