// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_SERIAL_CONTROLBOARD_HPP__
#define __DEXTRA_SERIAL_CONTROLBOARD_HPP__

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/SerialInterfaces.h>

#include "DextraRawControlboard.hpp"
#include "Synapse.hpp"

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
 */
class SerialSynapse : public Synapse
{
public:
    SerialSynapse(yarp::dev::ISerialDevice * iSerialDevice);

protected:
    virtual bool getMessage(unsigned char * msg, char stopByte, int size);
    virtual bool sendMessage(unsigned char * msg, int size);

private:
    yarp::dev::ISerialDevice * iSerialDevice;
};

/**
 * @ingroup DextraSerialControlboard
 * @brief Serial implementation for the custom UC3M Dextra Hand controlboard interfaces.
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

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- IAxisInfo Declarations ---------
    virtual bool getAxisName(int axis, std::string &name)
    { return raw.getAxisNameRaw(axis, name); }
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum &type)
    { return raw.getJointTypeRaw(axis, type); }

    //  --------- IControlLimits Declarations ---------
    virtual bool setLimits(int axis, double min, double max)
    { return raw.setLimitsRaw(axis, min, max); }
    virtual bool getLimits(int axis, double *min, double *max)
    { return raw.getLimitsRaw(axis, min, max); }
    virtual bool setVelLimits(int axis, double min, double max)
    { return raw.setVelLimitsRaw(axis, min, max); }
    virtual bool getVelLimits(int axis, double *min, double *max)
    { return raw.getVelLimitsRaw(axis, min, max); }

    //  --------- IControlMode Declarations ---------
    virtual bool getControlMode(int j, int *mode)
    { return raw.getControlModeRaw(j, mode); }
    virtual bool getControlModes(int *modes)
    { return raw.getControlModesRaw(modes); }
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes)
    { return raw.getControlModesRaw(n_joint, joints, modes); }
    virtual bool setControlMode(const int j, const int mode)
    { return raw.setControlModeRaw(j, mode); }
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes)
    { return raw.setControlModesRaw(n_joint, joints, modes); }
    virtual bool setControlModes(int *modes)
    { return raw.setControlModesRaw(modes); }

    //  ---------- IEncoders Declarations ----------
    virtual bool resetEncoder(int j)
    { return raw.resetEncoderRaw(j); }
    virtual bool resetEncoders()
    { return raw.resetEncodersRaw(); }
    virtual bool setEncoder(int j, double val)
    { return raw.setEncoderRaw(j, val); }
    virtual bool setEncoders(const double *vals)
    { return raw.setEncodersRaw(vals); }
    virtual bool getEncoder(int j, double *v)
    { return raw.getEncoderRaw(j, v); }
    virtual bool getEncoders(double *encs)
    { return raw.getEncodersRaw(encs); }
    virtual bool getEncoderSpeed(int j, double *sp)
    { return raw.getEncoderSpeedRaw(j, sp); }
    virtual bool getEncoderSpeeds(double *spds)
    { return raw.getEncoderSpeedsRaw(spds); }
    virtual bool getEncoderAcceleration(int j, double *spds)
    { return raw.getEncoderAccelerationRaw(j, spds); }
    virtual bool getEncoderAccelerations(double *accs)
    { return raw.getEncoderAccelerationsRaw(accs); }

    //  ---------- IEncodersTimed Declarations ----------
    virtual bool getEncodersTimed(double *encs, double *time)
    { return raw.getEncodersRaw(encs); }
    virtual bool getEncoderTimed(int j, double *encs, double *time)
    { return raw.getEncoderTimedRaw(j, encs, time); }

    // ------- IPositionControl declarations -------
    virtual bool getAxes(int *ax)
    { return raw.getAxes(ax); }
    virtual bool positionMove(int j, double ref)
    { return raw.positionMoveRaw(j, ref); }
    virtual bool positionMove(const double *refs)
    { return raw.positionMoveRaw(refs); }
    virtual bool relativeMove(int j, double delta)
    { return raw.relativeMoveRaw(j, delta); }
    virtual bool relativeMove(const double *deltas)
    { return raw.relativeMoveRaw(deltas); }
    virtual bool checkMotionDone(int j, bool *flag)
    { return raw.checkMotionDoneRaw(j, flag); }
    virtual bool checkMotionDone(bool *flag)
    { return raw.checkMotionDoneRaw(flag); }
    virtual bool setRefSpeed(int j, double sp)
    { return raw.setRefSpeedRaw(j, sp); }
    virtual bool setRefSpeeds(const double *spds)
    { return raw.setRefSpeedsRaw(spds); }
    virtual bool setRefAcceleration(int j, double acc)
    { return raw.setRefAccelerationRaw(j, acc); }
    virtual bool setRefAccelerations(const double *accs)
    { return raw.setRefAccelerationsRaw(accs); }
    virtual bool getRefSpeed(int j, double *ref)
    { return raw.getRefSpeedRaw(j, ref); }
    virtual bool getRefSpeeds(double *spds)
    { return raw.getRefSpeedsRaw(spds); }
    virtual bool getRefAcceleration(int j, double *acc)
    { return raw.getRefAccelerationRaw(j, acc); }
    virtual bool getRefAccelerations(double *accs)
    { return raw.getRefAccelerationsRaw(accs); }
    virtual bool stop(int j)
    { return raw.stopRaw(j); }
    virtual bool stop()
    { return raw.stopRaw(); }
    virtual bool positionMove(const int n_joint, const int *joints, const double *refs)
    { return raw.positionMoveRaw(n_joint, joints, refs); }
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas)
    { return raw.relativeMoveRaw(n_joint, joints, deltas); }
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags)
    { return raw.checkMotionDoneRaw(n_joint, joints, flags); }
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds)
    { return raw.setRefSpeedsRaw(n_joint, joints, spds); }
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs)
    { return raw.setRefAccelerationsRaw(n_joint, joints, accs); }
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds)
    { return raw.getRefSpeedsRaw(n_joint, joints, spds); }
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs)
    { return raw.getRefAccelerationsRaw(n_joint, joints, accs); }
    virtual bool stop(const int n_joint, const int *joints)
    { return raw.stopRaw(n_joint, joints); }
    virtual bool getTargetPosition(const int joint, double *ref)
    { return raw.getTargetPositionsRaw(joint, &joint, ref); }
    virtual bool getTargetPositions(double *refs)
    { return raw.getTargetPositionsRaw(refs); }
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs)
    { return raw.getTargetPositionsRaw(n_joint, joints, refs); }

    // ------- IPositionDirect declarations -------
    virtual bool setPosition(int j, double ref)
    { return raw.setPositionRaw(j, ref); }
    virtual bool setPositions(const int n_joint, const int *joints, const double *refs)
    { return raw.setPositionsRaw(n_joint, joints, refs); }
    virtual bool setPositions(const double *refs)
    { return raw.setPositionsRaw(refs); }

    //  --------- IVelocityControl declarations ---------
    virtual bool velocityMove(int j, double sp)
    { return raw.velocityMoveRaw(j, sp); }
    virtual bool velocityMove(const double *sp)
    { return raw.velocityMoveRaw(sp); }
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds)
    { return raw.velocityMoveRaw(n_joint, joints, spds); }
    virtual bool getRefVelocity(const int joint, double *vel)
    { return raw.getRefVelocityRaw(joint, vel); }
    virtual bool getRefVelocities(double *vels)
    { return raw.getRefVelocitiesRaw(vels); }
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels)
    { return raw.getRefVelocitiesRaw(n_joint, joints, vels); }

protected:

    DextraRawControlboard raw;
    yarp::dev::PolyDriver serialDevice;
};

}  // namespace roboticslab

#endif  // __DEXTRA_SERIAL_CONTROLBOARD_HPP__
