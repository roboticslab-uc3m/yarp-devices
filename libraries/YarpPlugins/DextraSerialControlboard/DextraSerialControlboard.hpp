// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_SERIAL_CONTROLBOARD_HPP__
#define __DEXTRA_SERIAL_CONTROLBOARD_HPP__

#include <mutex>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include "Synapse.hpp"

#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= Synapse::DATA_POINTS) return false; } while (0)
#define DEFAULT_PORT "/dev/ttyACM0" // also /dev/ttyUSB0

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup DextraSerialControlboard
 * @brief Contains roboticslab::DextraSerialControlboard.
 */

/**
 * @ingroup DextraSerialControlboard
 * @brief Implementation for the custom UC3M Dextra Hand controlboard interfaces.
 */
class DextraSerialControlboard : public yarp::dev::DeviceDriver,
                                 public yarp::dev::IAxisInfo,
                                 public yarp::dev::IControlLimits,
                                 public yarp::dev::IControlMode,
                                 public yarp::dev::IEncodersTimed,
                                 public yarp::dev::IPositionControl,
                                 public yarp::dev::IPositionDirect,
                                 public yarp::dev::IVelocityControl
{

public:

    DextraSerialControlboard();

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- IAxisInfo Declarations. Implementation in IAxisInfoImpl.cpp ---------
    virtual bool getAxisName(int axis, std::string &name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum &type);

    //  --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setVelLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double *min, double *max);

    //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------
    virtual bool getControlMode(int j, int *mode);
    virtual bool getControlModes(int *modes);
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModes(int *modes);

    //  ---------- IEncoders Declarations. Implementation in IEncodersTimedImpl.cpp ----------
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double *vals);
    virtual bool getEncoder(int j, double *v);
    virtual bool getEncoders(double *encs);
    virtual bool getEncoderSpeed(int j, double *sp);
    virtual bool getEncoderSpeeds(double *spds);
    virtual bool getEncoderAcceleration(int j, double *spds);
    virtual bool getEncoderAccelerations(double *accs);

    //  ---------- IEncodersTimed Declarations. Implementation in IEncodersTimedImpl.cpp ----------
    virtual bool getEncodersTimed(double *encs, double *time);
    virtual bool getEncoderTimed(int j, double *encs, double *time);

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------
    virtual bool getAxes(int *ax);
    virtual bool positionMove(int j, double ref);
    virtual bool positionMove(const double *refs);
    virtual bool relativeMove(int j, double delta);
    virtual bool relativeMove(const double *deltas);
    virtual bool checkMotionDone(int j, bool *flag);
    virtual bool checkMotionDone(bool *flag);
    virtual bool setRefSpeed(int j, double sp);
    virtual bool setRefSpeeds(const double *spds);
    virtual bool setRefAcceleration(int j, double acc);
    virtual bool setRefAccelerations(const double *accs);
    virtual bool getRefSpeed(int j, double *ref);
    virtual bool getRefSpeeds(double *spds);
    virtual bool getRefAcceleration(int j, double *acc);
    virtual bool getRefAccelerations(double *accs);
    virtual bool stop(int j);
    virtual bool stop();
    virtual bool positionMove(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);
    virtual bool getTargetPosition(const int joint, double *ref);
    virtual bool getTargetPositions(double *refs);
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs);

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------
    virtual bool setPosition(int j, double ref);
    virtual bool setPositions(const int n_joint, const int *joints, const double *refs);
    virtual bool setPositions(const double *refs);

    //  --------- IVelocityControl declarations and stub implementations. ---------
    virtual bool velocityMove(int j, double sp) { return false; }
    virtual bool velocityMove(const double *sp) { return false; }
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds) { return false; }
    virtual bool getRefVelocity(const int joint, double *vel) { return false; }
    virtual bool getRefVelocities(double *vels) { return false; }
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels) { return false; }

protected:

    double getSetpoint(int j);
    void getSetpoints(Synapse::Setpoints & setpoints);
    void setSetpoint(int j, Synapse::setpoint_t setpoint);
    void setSetpoints(const Synapse::Setpoints & setpoints);

    Synapse * synapse;
    yarp::dev::PolyDriver serialDevice;
    Synapse::Setpoints setpoints;
    mutable std::mutex setpointMutex;
};

}  // namespace roboticslab

#endif  // __DEXTRA_SERIAL_CONTROLBOARD_HPP__
