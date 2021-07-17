// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_SERIAL_CONTROLBOARD_HPP__
#define __DEXTRA_SERIAL_CONTROLBOARD_HPP__

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ISerialDevice.h>
#include <yarp/dev/PolyDriver.h>

#include "DextraRawControlboard.hpp"
#include "Synapse.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup DextraSerialControlboard
 * @brief Contains roboticslab::DextraSerialControlboard.
 */

/**
 * @ingroup DextraSerialControlboard
 * @brief Synapse interface for a serial bus.
 */
class SerialSynapse : public Synapse
{
public:
    //! Constructor.
    SerialSynapse(yarp::dev::ISerialDevice * iSerialDevice);

protected:
    bool getMessage(unsigned char * msg, char stopByte, int size) override;
    bool sendMessage(unsigned char * msg, int size) override;

private:
    yarp::dev::ISerialDevice * iSerialDevice;
};

/**
 * @ingroup DextraSerialControlboard
 * @brief Implementation of a serial-based stand-alone controlboard for a Dextra hand.
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

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- IAxisInfo declarations ---------

    virtual bool getAxisName(int axis, std::string & name) override
    { return raw.getAxisNameRaw(axis, name); }
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum & type) override
    { return raw.getJointTypeRaw(axis, type); }

    //  --------- IControlLimits declarations ---------

    virtual bool setLimits(int axis, double min, double max) override
    { return raw.setLimitsRaw(axis, min, max); }
    virtual bool getLimits(int axis, double * min, double * max) override
    { return raw.getLimitsRaw(axis, min, max); }
    virtual bool setVelLimits(int axis, double min, double max) override
    { return raw.setVelLimitsRaw(axis, min, max); }
    virtual bool getVelLimits(int axis, double * min, double * max) override
    { return raw.getVelLimitsRaw(axis, min, max); }

    //  --------- IControlMode declarations ---------

    virtual bool getControlMode(int j, int * mode) override
    { return raw.getControlModeRaw(j, mode); }
    virtual bool getControlModes(int * modes) override
    { return raw.getControlModesRaw(modes); }
    virtual bool getControlModes(int n_joint, const int * joints, int * modes) override
    { return raw.getControlModesRaw(n_joint, joints, modes); }
    virtual bool setControlMode(int j, int mode) override
    { return raw.setControlModeRaw(j, mode); }
    virtual bool setControlModes(int * modes) override
    { return raw.setControlModesRaw(modes); }
    virtual bool setControlModes(int n_joint, const int * joints, int * modes) override
    { return raw.setControlModesRaw(n_joint, joints, modes); }

    //  ---------- IEncoders declarations ----------

    virtual bool resetEncoder(int j) override
    { return raw.resetEncoderRaw(j); }
    virtual bool resetEncoders() override
    { return raw.resetEncodersRaw(); }
    virtual bool setEncoder(int j, double val) override
    { return raw.setEncoderRaw(j, val); }
    virtual bool setEncoders(const double *vals) override
    { return raw.setEncodersRaw(vals); }
    virtual bool getEncoder(int j, double * v) override
    { return raw.getEncoderRaw(j, v); }
    virtual bool getEncoders(double *encs) override
    { return raw.getEncodersRaw(encs); }
    virtual bool getEncoderSpeed(int j, double * sp) override
    { return raw.getEncoderSpeedRaw(j, sp); }
    virtual bool getEncoderSpeeds(double * spds) override
    { return raw.getEncoderSpeedsRaw(spds); }
    virtual bool getEncoderAcceleration(int j, double * spds) override
    { return raw.getEncoderAccelerationRaw(j, spds); }
    virtual bool getEncoderAccelerations(double * accs) override
    { return raw.getEncoderAccelerationsRaw(accs); }

    //  ---------- IEncodersTimed declarations ----------

    virtual bool getEncoderTimed(int j, double * encs, double * time) override
    { return raw.getEncoderTimedRaw(j, encs, time); }
    virtual bool getEncodersTimed(double * encs, double * time) override
    { return raw.getEncodersRaw(encs); }

    // ------- IPositionControl declarations -------

    virtual bool getAxes(int * ax) override
    { return raw.getAxes(ax); }
    virtual bool positionMove(int j, double ref) override
    { return raw.positionMoveRaw(j, ref); }
    virtual bool positionMove(const double * refs) override
    { return raw.positionMoveRaw(refs); }
    virtual bool positionMove(int n_joint, const int * joints, const double * refs) override
    { return raw.positionMoveRaw(n_joint, joints, refs); }
    virtual bool relativeMove(int j, double delta) override
    { return raw.relativeMoveRaw(j, delta); }
    virtual bool relativeMove(const double * deltas) override
    { return raw.relativeMoveRaw(deltas); }
    virtual bool relativeMove(int n_joint, const int * joints, const double * deltas) override
    { return raw.relativeMoveRaw(n_joint, joints, deltas); }
    virtual bool checkMotionDone(int j, bool * flag) override
    { return raw.checkMotionDoneRaw(j, flag); }
    virtual bool checkMotionDone(bool * flag) override
    { return raw.checkMotionDoneRaw(flag); }
    virtual bool checkMotionDone(int n_joint, const int * joints, bool * flag) override
    { return raw.checkMotionDoneRaw(n_joint, joints, flag); }
    virtual bool setRefSpeed(int j, double sp) override
    { return raw.setRefSpeedRaw(j, sp); }
    virtual bool setRefSpeeds(const double * spds) override
    { return raw.setRefSpeedsRaw(spds); }
    virtual bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override
    { return raw.setRefSpeedsRaw(n_joint, joints, spds); }
    virtual bool setRefAcceleration(int j, double acc) override
    { return raw.setRefAccelerationRaw(j, acc); }
    virtual bool setRefAccelerations(const double * accs) override
    { return raw.setRefAccelerationsRaw(accs); }
    virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override
    { return raw.setRefAccelerationsRaw(n_joint, joints, accs); }
    virtual bool getRefSpeed(int j, double * ref) override
    { return raw.getRefSpeedRaw(j, ref); }
    virtual bool getRefSpeeds(double * spds) override
    { return raw.getRefSpeedsRaw(spds); }
    virtual bool getRefSpeeds(int n_joint, const int * joints, double * spds) override
    { return raw.getRefSpeedsRaw(n_joint, joints, spds); }
    virtual bool getRefAcceleration(int j, double * acc) override
    { return raw.getRefAccelerationRaw(j, acc); }
    virtual bool getRefAccelerations(double * accs) override
    { return raw.getRefAccelerationsRaw(accs); }
    virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override
    { return raw.getRefAccelerationsRaw(n_joint, joints, accs); }
    virtual bool stop(int j) override
    { return raw.stopRaw(j); }
    virtual bool stop() override
    { return raw.stopRaw(); }
    virtual bool stop(int n_joint, const int * joints) override
    { return raw.stopRaw(n_joint, joints); }
    virtual bool getTargetPosition(int joint, double * ref) override
    { return raw.getTargetPositionsRaw(joint, &joint, ref); }
    virtual bool getTargetPositions(double * refs) override
    { return raw.getTargetPositionsRaw(refs); }
    virtual bool getTargetPositions(int n_joint, const int * joints, double * refs) override
    { return raw.getTargetPositionsRaw(n_joint, joints, refs); }

    // ------- IPositionDirect declarations -------

    virtual bool setPosition(int j, double ref) override
    { return raw.setPositionRaw(j, ref); }
    virtual bool setPositions(const double * refs) override
    { return raw.setPositionsRaw(refs); }
    virtual bool setPositions(int n_joint, const int * joints, const double * refs) override
    { return raw.setPositionsRaw(n_joint, joints, refs); }

    //  --------- IVelocityControl declarations ---------

    virtual bool velocityMove(int j, double sp) override
    { return raw.velocityMoveRaw(j, sp); }
    virtual bool velocityMove(const double * sp) override
    { return raw.velocityMoveRaw(sp); }
    virtual bool velocityMove(int n_joint, const int * joints, const double * spds) override
    { return raw.velocityMoveRaw(n_joint, joints, spds); }
    virtual bool getRefVelocity(int joint, double * vel) override
    { return raw.getRefVelocityRaw(joint, vel); }
    virtual bool getRefVelocities(double * vels) override
    { return raw.getRefVelocitiesRaw(vels); }
    virtual bool getRefVelocities(int n_joint, const int * joints, double * vels) override
    { return raw.getRefVelocitiesRaw(n_joint, joints, vels); }

protected:

    DextraRawControlboard raw;
    yarp::dev::PolyDriver serialDevice;
};

} // namespace roboticslab

#endif // __DEXTRA_SERIAL_CONTROLBOARD_HPP__
