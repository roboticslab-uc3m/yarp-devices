// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_RAW_CONTROLBOARD_HPP__
#define __DEXTRA_RAW_CONTROLBOARD_HPP__

#include <mutex>

#include <yarp/dev/ControlBoardInterfaces.h>

#include "Synapse.hpp"

#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= Synapse::DATA_POINTS) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * \defgroup DextraRawControlboard
 * @brief Contains roboticslab::DextraRawControlboard.
 */

/**
 * @ingroup DextraRawControlboard
 * @brief Implementation for the custom UC3M Dextra Hand controlboard interfaces.
 */
class DextraRawControlboard : public yarp::dev::IAxisInfoRaw,
                              public yarp::dev::IControlLimitsRaw,
                              public yarp::dev::IControlModeRaw,
                              public yarp::dev::IEncodersTimedRaw,
                              public yarp::dev::IPositionControlRaw,
                              public yarp::dev::IPositionDirectRaw,
                              public yarp::dev::IVelocityControlRaw
{
public:

    DextraRawControlboard();

    void acquireSynapseHandle(Synapse * synapse);
    void destroySynapse();

    //  --------- IAxisInfoRaw Declarations. Implementation in IAxisInfoRawImpl.cpp ---------
    virtual bool getAxisNameRaw(int axis, std::string &name);
    virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum &type);

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    virtual bool getControlModeRaw(int j, int *mode);
    virtual bool getControlModesRaw(int *modes);
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);

    //  ---------- IEncodersRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals);
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs);
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds);
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs);

    //  ---------- IEncodersTimedRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
    virtual bool getEncodersTimedRaw(double *encs, double *time);
    virtual bool getEncoderTimedRaw(int j, double *encs, double *time);

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------
    virtual bool getAxes(int *ax);
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs);
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas);
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool checkMotionDoneRaw(bool *flag);
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds);
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs);
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds);
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs);
    virtual bool stopRaw(int j);
    virtual bool stopRaw();
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    virtual bool stopRaw(const int n_joint, const int *joints);
    virtual bool getTargetPositionRaw(const int joint, double *ref);
    virtual bool getTargetPositionsRaw(double *refs);
    virtual bool getTargetPositionsRaw(const int n_joint, const int *joints, double *refs);

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------
    virtual bool setPositionRaw(int j, double ref);
    virtual bool setPositionsRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool setPositionsRaw(const double *refs);

    //  --------- IVelocityControlRaw declarations and stub implementations. ---------
    virtual bool velocityMoveRaw(int j, double sp) { return false; }
    virtual bool velocityMoveRaw(const double *sp) { return false; }
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds) { return false; }
    virtual bool getRefVelocityRaw(const int joint, double *vel) { return false; }
    virtual bool getRefVelocitiesRaw(double *vels) { return false; }
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels) { return false; }

protected:

    Synapse * synapse;

private:

    double getSetpoint(int j);
    void getSetpoints(Synapse::Setpoints & setpoints);
    void setSetpoint(int j, Synapse::setpoint_t setpoint);
    void setSetpoints(const Synapse::Setpoints & setpoints);

    Synapse::Setpoints setpoints;
    mutable std::mutex setpointMutex;
};

}  // namespace roboticslab

#endif  // __DEXTRA_RAW_CONTROLBOARD_HPP__
