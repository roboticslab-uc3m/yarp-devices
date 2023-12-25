// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_HAND_HPP__
#define __TEXTILES_HAND_HPP__

#include <yarp/conf/version.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#if YARP_VERSION_COMPARE(<, 3, 9, 0)
# include <yarp/dev/IEncodersTimed.h>
# include <yarp/dev/IPositionControl.h>
# include <yarp/dev/IVelocityControl.h>
#endif
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ISerialDevice.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup TextilesHand
 * @brief Contains roboticslab::TextilesHand.
 */

/**
 * @ingroup TextilesHand
 * @brief Implementation for the custom UC3M Textiles Hand as a single CAN bus joint (control board raw interfaces).
 */
class TextilesHand : public yarp::dev::DeviceDriver,
                     public yarp::dev::IControlMode,
#if YARP_VERSION_COMPARE(<, 3, 9, 0)
                     public yarp::dev::IEncodersTimed,
                     public yarp::dev::IPositionControl,
                     public yarp::dev::IVelocityControl,
#endif
                     public yarp::dev::IPositionDirect
{
public:
    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

    bool getControlMode(int j, int * mode) override;
    bool getControlModes(int * modes) override;
    bool getControlModes(int n_joint, const int * joints, int * modes) override;
    bool setControlMode(int j, int mode) override;
    bool setControlModes(int * modes) override;
    bool setControlModes(int n_joint, const int * joints, int * modes) override;

#if YARP_VERSION_COMPARE(<, 3, 9, 0)
    //  ---------- IEncoders Declarations. Implementation in IEncodersImpl.cpp ----------

    //bool getAxes(int * ax) override;
    bool resetEncoder(int j) override { return false; }
    bool resetEncoders() override { return false; }
    bool setEncoder(int j, double val) override { return false; }
    bool setEncoders(const double * vals) override { return false; }
    bool getEncoder(int j, double * v) override { return false; }
    bool getEncoders(double * encs) override { return false; }
    bool getEncoderSpeed(int j, double * sp) override { return false; }
    bool getEncoderSpeeds(double * spds) override { return false; }
    bool getEncoderAcceleration(int j, double * spds) override { return false; }
    bool getEncoderAccelerations(double * accs) override { return false; }

    //  ---------- IEncodersTimed Declarations. Implementation in IEncodersImpl.cpp ----------

    bool getEncoderTimed(int j, double * encs, double * time) override { return false; }
    bool getEncodersTimed(double * encs, double * time) override { return false; }

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool positionMove(int j, double ref) override { return false; }
    bool positionMove(const double * refs) override { return false; }
    bool positionMove(int n_joint, const int * joints, const double * refs) override { return false; }
    bool relativeMove(int j, double delta) override { return false; }
    bool relativeMove(const double * deltas) override { return false; }
    bool relativeMove(int n_joint, const int * joints, const double * deltas) override { return false; }
    bool checkMotionDone(int j, bool * flag) override { return false; }
    bool checkMotionDone(bool * flag) override { return false; }
    bool checkMotionDone(int n_joint, const int * joints, bool * flag) override { return false; }
    bool setRefSpeed(int j, double sp) override { return false; }
    bool setRefSpeeds(const double * spds) override { return false; }
    bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override { return false; }
    bool setRefAcceleration(int j, double acc) override { return false; }
    bool setRefAccelerations(const double * accs) override { return false; }
    bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override { return false; }
    bool getRefSpeeds(int n_joint, const int * joints, double * spds) override { return false; }
    bool getRefSpeed(int j, double * ref) override { return false; }
    bool getRefSpeeds(double * spds) override { return false; }
    bool getRefAcceleration(int j, double * acc) override { return false; }
    bool getRefAccelerations(double * accs) override { return false; }
    bool getRefAccelerations(int n_joint, const int * joints, double * accs) override { return false; }
    bool stop(int j) override { return false; }
    bool stop() override { return false; }
    bool stop(int n_joint, const int *joints) override { return false; }
    bool getTargetPosition(int joint, double * ref) override { return false; }
    bool getTargetPositions(double * refs) override { return false; }
    bool getTargetPositions(int n_joint, const int * joints, double * refs) override { return false; }

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    //bool getAxes(int * ax) override;
    bool velocityMove(int j, double spd) override { return false; }
    bool velocityMove(const double * spds) override { return false; }
    bool velocityMove(int n_joint, const int * joints, const double * spds) override { return false; }
    bool getRefVelocity(int joint, double * vel) override { return false; }
    bool getRefVelocities(double * vels) override { return false; }
    bool getRefVelocities(int n_joint, const int * joints, double * vels) override { return false; }
    //bool setRefAcceleration(int j, double acc) override;
    //bool setRefAccelerations(const double * accs) override;
    //bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    //bool getRefAcceleration(int j, double * acc) override;
    //bool getRefAccelerations(double * accs) override;
    //bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    //bool stop(int j) override;
    //bool stop() override;
    //bool stop(int n_joint, const int *joints) override;
#endif // YARP_VERSION_COMPARE(<, 3, 9, 0)

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    bool getAxes(int * ax) override;
    bool setPosition(int j, double ref) override;
    bool setPositions(const double * refs) override;
    bool setPositions(int n_joint, const int * joints, const double * refs) override;
    bool getRefPosition(int joint, double * ref) override;
    bool getRefPositions(double * refs) override;
    bool getRefPositions(int n_joint, const int * joints, double * refs) override;

private:
    double lastTarget {0.0};

    yarp::dev::PolyDriver serialDevice;
    yarp::dev::ISerialDevice * iSerialDevice {nullptr};
};

} // namespace roboticslab

#endif // __TEXTILES_HAND_HPP__
