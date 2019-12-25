// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FAKE_JOINT_HPP__
#define __FAKE_JOINT_HPP__

#include <yarp/dev/ControlBoardInterfaces.h>

#include "ICanBusSharer.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup FakeJoint
 * @brief Contains roboticslab::FakeJoint.
 */

/**
 * @ingroup FakeJoint
 * @brief Implementation for a fake joint Raw(instant movement) as a single CAN bus joint Raw(controlboard raw interfaces).
 */
class FakeJoint : public yarp::dev::DeviceDriver,
                  public yarp::dev::IControlLimitsRaw,
                  public yarp::dev::IControlModeRaw,
                  public yarp::dev::ICurrentControlRaw,
                  public yarp::dev::IEncodersTimedRaw,
                  public yarp::dev::IInteractionModeRaw,
                  public yarp::dev::IPositionControlRaw,
                  public yarp::dev::IPositionDirectRaw,
                  public yarp::dev::IRemoteVariablesRaw,
                  public yarp::dev::ITorqueControlRaw,
                  public yarp::dev::IVelocityControlRaw,
                  public ICanBusSharer
{
public:

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override
    { return true; }
    virtual bool close() override
    { return true; }

    //  --------- ICanBusSharer eclarations ---------

    virtual unsigned int getId() override
    { return 0; }
    virtual bool interpretMessage(const yarp::dev::CanMessage & message) override
    { return true; }
    virtual bool initialize() override
    { return true; }
    virtual bool finalize() override
    { return true; }
    virtual bool registerSender(CanSenderDelegate * sender) override
    { return true; }

    //  --------- IControlLimitsRaw declarations ---------

    virtual bool setLimitsRaw(int axis, double min, double max) override
    { return true; }
    virtual bool getLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return true; }
    virtual bool setVelLimitsRaw(int axis, double min, double max) override
    { return true; }
    virtual bool getVelLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return true; }

    //  --------- IControlModeRaw declarations ---------

    virtual bool getControlModeRaw(int j, int * mode) override
    { *mode = VOCAB_CM_CONFIGURED; return true; }
    virtual bool getControlModesRaw(int * modes) override
    { return getControlModeRaw(0, &modes[0]); }
    virtual bool getControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return getControlModeRaw(0, &modes[0]); }
    virtual bool setControlModeRaw(int j, int mode) override
    { return true; }
    virtual bool setControlModesRaw(int * modes) override
    { return true; }
    virtual bool setControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return true; }

    //  --------- ICurrentControlRaw declarations ---------

    virtual bool getNumberOfMotorsRaw(int * ax) override // TODO
    { return getAxes(ax); }
    virtual bool getCurrentRaw(int m, double * curr) override
    { *curr = 0.0; return true; }
    virtual bool getCurrentsRaw(double * currs) override
    { return getCurrentRaw(0, &currs[0]); }
    virtual bool getCurrentRangeRaw(int m, double * min, double * max) override
    { *min = *max = 0.0; return true; }
    virtual bool getCurrentRangesRaw(double * mins, double * maxs) override
    { return getCurrentRangeRaw(0, &mins[0], &maxs[0]); }
    virtual bool setRefCurrentRaw(int m, double curr) override
    { return true; }
    virtual bool setRefCurrentsRaw(const double * currs) override
    { return true; }
    virtual bool setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override
    { return true; }
    virtual bool getRefCurrentRaw(int m, double * curr) override
    { *curr = 0.0; return true; }
    virtual bool getRefCurrentsRaw(double * currs) override
    { return getRefCurrentRaw(0, &currs[0]); }

    //  ---------- IEncodersRaw declarations ----------.

    virtual bool getAxes(int * ax) override
    { *ax = 1; return true; }
    virtual bool resetEncoderRaw(int j) override
    { return true; }
    virtual bool resetEncodersRaw() override
    { return true; }
    virtual bool setEncoderRaw(int j, double val) override
    { return true; }
    virtual bool setEncodersRaw(const double * vals) override
    { return true; }
    virtual bool getEncoderRaw(int j, double * v) override
    { *v = 0.0; return true; }
    virtual bool getEncodersRaw(double * encs) override
    { return getEncoderRaw(0, &encs[0]); }
    virtual bool getEncoderSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return true; }
    virtual bool getEncoderSpeedsRaw(double * spds) override
    { return getEncoderRaw(0, &spds[0]); }
    virtual bool getEncoderAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return true; }
    virtual bool getEncoderAccelerationsRaw(double * accs) override
    { return getEncoderAccelerationRaw(0, &accs[0]); }

    //  ---------- IEncodersTimedRaw declarations ----------

    virtual bool getEncoderTimedRaw(int j, double * enc, double * time) override
    { *enc = *time = 0.0; return true; }
    virtual bool getEncodersTimedRaw(double * encs, double * times) override
    { return getEncoderTimedRaw(0, &encs[0], &times[0]); }

    // ------- IInteractionModeRaw declarations -------

    virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override
    { *mode = yarp::dev::InteractionModeEnum::VOCAB_IM_UNKNOWN; return true; }
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }
    virtual bool getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }
    virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { return true; }
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return true; }
    virtual bool setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return true; }

    // ------- IPositionControlRaw declarations -------

    //virtual bool getAxesRaw(int * ax) override;
    virtual bool positionMoveRaw(int j, double ref) override
    { return true; }
    virtual bool positionMoveRaw(const double * refs) override
    { return true; }
    virtual bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override
    { return true; }
    virtual bool relativeMoveRaw(int j, double delta) override
    { return true; }
    virtual bool relativeMoveRaw(const double * deltas) override
    { return true; }
    virtual bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override
    { return true; }
    virtual bool checkMotionDoneRaw(int j, bool * flag) override
    { *flag = true; return true; }
    virtual bool checkMotionDoneRaw(bool * flag) override
    { return checkMotionDoneRaw(flag); }
    virtual bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override
    { return checkMotionDoneRaw(flag); }
    virtual bool setRefSpeedRaw(int j, double sp) override
    { return true; }
    virtual bool setRefSpeedsRaw(const double * spds) override
    { return true; }
    virtual bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return true; }
    virtual bool setRefAccelerationRaw(int j, double acc) override
    { return true; }
    virtual bool setRefAccelerationsRaw(const double * accs) override
    { return true; }
    virtual bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return true; }
    virtual bool getRefSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return true; }
    virtual bool getRefSpeedsRaw(double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }
    virtual bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }
    virtual bool getRefAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return true; }
    virtual bool getRefAccelerationsRaw(double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }
    virtual bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }
    virtual bool stopRaw(int j) override
    { return true; }
    virtual bool stopRaw() override
    { return true; }
    virtual bool stopRaw(int n_joint, const int *joints) override
    { return true; }
    virtual bool getTargetPositionRaw(int joint, double * ref) override
    { *ref = 0.0; return true; }
    virtual bool getTargetPositionsRaw(double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }
    virtual bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }

    // ------- IPositionDirectRaw declarations -------

    //virtual bool getAxesRaw(int * ax) override;
    virtual bool setPositionRaw(int j, double ref) override
    { return true; }
    virtual bool setPositionsRaw(const double * refs) override
    { return true; }
    virtual bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return true; }
    virtual bool getRefPositionRaw(int joint, double * ref) override
    { *ref = 0.0; return true; }
    virtual bool getRefPositionsRaw(double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }
    virtual bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }

    // ------- IRemoteVariablesRaw declarations -------

    virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override
    { return true; }
    virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override
    { return true; }
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override
    { return true; }

    // -------- ITorqueControlRaw declarations --------

    //virtual bool getAxesRaw(int * ax) override;
    virtual bool getRefTorqueRaw(int j, double * t) override
    { *t = 0.0; return true; }
    virtual bool getRefTorquesRaw(double * t) override
    { return getRefTorqueRaw(0, &t[0]); }
    virtual bool setRefTorqueRaw(int j, double t) override
    { return true; }
    virtual bool setRefTorquesRaw(const double * t) override
    { return true; }
    virtual bool setRefTorquesRaw(int n_joint, const int * joints, const double * t) override
    { return true; }
    virtual bool getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override
    { return true; }
    virtual bool setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override
    { return true; }
    virtual bool getTorqueRaw(int j, double * t) override
    { *t = 0.0; return true; }
    virtual bool getTorquesRaw(double * t) override
    { return getTorqueRaw(0, &t[0]); }
    virtual bool getTorqueRangeRaw(int j, double * min, double * max) override
    { *min = *max = 0.0; return true; }
    virtual bool getTorqueRangesRaw(double * mins, double * maxs) override
    { return getTorqueRangeRaw(0, &mins[0], &maxs[0]); }

    //  --------- IVelocityControl declarations ---------

    //virtual bool getAxesRaw(int * ax) override;
    virtual bool velocityMoveRaw(int j, double spd) override
    { return true; }
    virtual bool velocityMoveRaw(const double * spds) override
    { return true; }
    virtual bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override
    { return true; }
    virtual bool getRefVelocityRaw(int joint, double * vel) override
    { *vel = 0.0; return true; }
    virtual bool getRefVelocitiesRaw(double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }
    virtual bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }
    //virtual bool setRefAccelerationRaw(int j, double acc) override;
    //virtual bool setRefAccelerationsRaw(const double * accs) override;
    //virtual bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    //virtual bool getRefAccelerationRaw(int j, double * acc) override;
    //virtual bool getRefAccelerationsRaw(double * accs) override;
    //virtual bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    //virtual bool stopRaw(int j) override;
    //virtual bool stopRaw() override;
    //virtual bool stopRaw(int n_joint, const int *joints) override;
};

} // namespace roboticslab

#endif // __FAKE_JOINT_HPP__
