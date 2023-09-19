// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EMULATED_CONTROL_BOARD_HPP__
#define __EMULATED_CONTROL_BOARD_HPP__

#include <vector>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup EmulatedControlBoard
 * @brief Contains roboticslab::EmulatedControlBoard.
 */

/**
 * @ingroup EmulatedControlBoard
 * @brief Implements several motor interfaces.
 */
class EmulatedControlBoard : public yarp::dev::DeviceDriver,
                             public yarp::dev::IControlLimits,
                             public yarp::dev::IControlMode,
                             public yarp::dev::IEncodersTimed,
                             public yarp::dev::IPositionControl,
                             public yarp::dev::IPositionDirect,
                             public yarp::dev::IVelocityControl,
                             public yarp::os::PeriodicThread
{
public:

    // Set the thread period in the class constructor
    EmulatedControlBoard() : PeriodicThread(1.0) {} // In seconds

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

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    bool setPosition(int j, double ref) override;
    bool setPositions(int n_joint, const int *joints, const double *refs) override;
    bool setPositions(const double *refs) override;
    bool getRefPosition(int joint, double *ref) override;
    bool getRefPositions(double *refs) override;
    bool getRefPositions(int n_joint, const int *joints, double *refs) override;

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

    // --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------

    bool setLimits(int axis, double min, double max) override;
    bool getLimits(int axis, double *min, double *max) override;
    bool setVelLimits(int axis, double min, double max) override;
    bool getVelLimits(int axis, double *min, double *max) override;

    // --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

    bool getControlMode(int j, int *mode) override;
    bool getControlModes(int *modes) override;
    bool getControlModes(int n_joint, const int *joints, int *modes) override;
    bool setControlMode(int j, const int mode) override;
    bool setControlModes(int n_joint, const int *joints, int *modes) override;
    bool setControlModes(int *modes) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------

    bool threadInit() override;
    void run() override;

    // ----- Shared Area Funcion declarations. Implementation in SharedArea.cpp -----

    void setEncRaw(const int index, const double position);
    void setEncsRaw(const std::vector<double> & positions);

    double getEncRaw(const int index);
    std::vector<double> getEncsRaw();

    double getEncExposed(const int index);
    std::vector<double> getEncsExposed();

    // ------------------------------- Private -------------------------------------

private:

    enum jmc_state { NOT_CONTROLLING, POSITION_MOVE, RELATIVE_MOVE, VELOCITY_MOVE };
    enum jmc_mode { POSITION_MODE, VELOCITY_MODE, POSITION_DIRECT_MODE };

    bool setPositionMode(int j);
    bool setVelocityMode(int j);
    bool setTorqueMode(int j);
    bool setPositionDirectMode(int j);

    // General Joint Motion Controller parameters //
    unsigned int axes;
    double jmcMs;
    jmc_mode controlMode;
    double lastTime;

    yarp::os::Semaphore encRawMutex;  // SharedArea

    std::vector<jmc_state> jointStatus;

    std::vector<double> encRaw;
    std::vector<double> encRawExposed;  // For conversion.
    std::vector<double> initPos;  // Exposed.
    std::vector<double> jointTol;  // Exposed.
    std::vector<double> maxLimit;  // Exposed.
    std::vector<double> minLimit;  // Exposed.
    std::vector<double> refAcc;  // Exposed.
    std::vector<double> refSpeed;  // Exposed.
    std::vector<double> targetExposed;  // Exposed.
    std::vector<double> velRawExposed;  // For conversion.
    std::vector<double> velRaw;
};

} // namespace roboticslab

#endif // __EMULATED_CONTROL_BOARD_HPP__
