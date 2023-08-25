// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __AMOR_CONTROL_BOARD_HPP__
#define __AMOR_CONTROL_BOARD_HPP__

#include <mutex>
#include <vector>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <amor.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup AmorControlBoard
 * @brief Contains roboticslab::AmorControlBoard.
 */

/**
* @ingroup AmorControlBoard
* @brief Implements several yarp::dev:: control board interfaces.
*/
class AmorControlBoard : public yarp::dev::DeviceDriver,
                         public yarp::dev::IAxisInfo,
                         public yarp::dev::IControlLimits,
                         public yarp::dev::IControlMode,
                         public yarp::dev::ICurrentControl,
                         public yarp::dev::IEncodersTimed,
                         public yarp::dev::IPositionControl,
                         public yarp::dev::IVelocityControl
{
public:

    ~AmorControlBoard() override
    { close(); }

    // -------- DeviceDriver declarations. Implementation in IDeviceDriverImpl.cpp --------

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

    // ---------- IEncoders declarations. Implementation in IEncodersImpl.cpp ----------

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

    // --------- IEncodersTimed declarations. Implementation in IEncodersTimedImpl.cpp ---------

   bool getEncodersTimed(double *encs, double *time) override;
   bool getEncoderTimed(int j, double *encs, double *time) override;

    // --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double *sp) override;
    bool velocityMove(const int n_joint, const int *joints, const double *spds) override;
    bool getRefVelocity(int joint, double *vel) override;
    bool getRefVelocities(double *vels) override;
    bool getRefVelocities(int n_joint, const int *joints, double *vels) override;

    // --------- IControlLimits declarations. Implementation in IControlLimitsImpl.cpp ---------

    bool setLimits(int axis, double min, double max) override;
    bool getLimits(int axis, double *min, double *max) override;
    bool setVelLimits(int axis, double min, double max) override;
    bool getVelLimits(int axis, double *min, double *max) override;

    // --------- IControlMode declarations. Implementation in IControlModeImpl.cpp ---------

    bool getControlMode(int j, int *mode) override;
    bool getControlModes(int *modes) override;
    bool getControlModes(int n_joint, const int *joints, int *modes) override;
    bool setControlMode(int j, int mode) override;
    bool setControlModes(int n_joint, const int *joints, int *modes) override;
    bool setControlModes(int *modes) override;

    // -------- IAxisInfo declarations. Implementation in IAxisInfoImpl.cpp --------

    bool getAxisName(int axis, std::string& name) override;
    bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    // --------- ICurrentControl Declarations. Implementation in ICurrentControlImpl.cpp ---------

    bool getNumberOfMotors(int *ax) override;
    bool getCurrent(int m, double *curr) override;
    bool getCurrents(double *currs) override;
    bool getCurrentRange(int m, double *min, double *max) override;
    bool getCurrentRanges(double *min, double *max) override;
    bool setRefCurrents(const double *currs) override;
    bool setRefCurrent(int m, double curr) override;
    bool setRefCurrents(int n_motor, const int *motors, const double *currs) override;
    bool getRefCurrents(double *currs) override;
    bool getRefCurrent(int m, double *curr) override;

    // ------------------------------- Protected -------------------------------------

protected:

    /**
     * Check if index is within range (referred to driver vector size).
     * @param idx index to check.
     * @return true/false on success/failure.
     */
    bool indexWithinRange(const int& idx);

    /**
     * Check if number of joints is within range.
     * @param n_joint index to check.
     * @return true/false on success/failure.
     */
    bool batchWithinRange(const int& n_joint);

    /**
     * Convert from radians to degrees.
     * @param rad radians
     * @return degrees
     */
    static double toDeg(double rad);

    /**
     * Convert from degrees to radians.
     * @param deg degrees
     * @return radians
     */
    static double toRad(double deg);

private:

    AMOR_HANDLE handle {AMOR_INVALID_HANDLE};
    mutable std::mutex handleMutex;
    yarp::dev::PolyDriver cartesianControllerDevice;
    bool usingCartesianController {false};
    int controlMode {VOCAB_CM_POSITION};
};

} // namespace roboticslab

#endif // __AMOR_CONTROL_BOARD_HPP__
