// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EMULATED_CONTROL_BOARD_HPP__
#define __EMULATED_CONTROL_BOARD_HPP__

#include <mutex>
#include <vector>

#include <yarp/conf/version.h>

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include "EmulatedControlBoard_ParamsParser.h"


/**
 * @ingroup YarpPlugins
 * @defgroup EmulatedControlBoard
 * @brief Contains EmulatedControlBoard.
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
                             public yarp::os::PeriodicThread,
                             public EmulatedControlBoard_ParamsParser
{
public:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    using return_t = yarp::dev::ReturnValue;
#else
    using return_t = bool;
#endif

    // Set the thread period in the class constructor
    EmulatedControlBoard() : yarp::os::PeriodicThread(1.0) {} // In seconds

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAxes(std::size_t & ax) override;
#else
    return_t getAxes(int * ax) override;
#endif
    return_t positionMove(int j, double ref) override;
    return_t positionMove(const double * refs) override;
    return_t positionMove(int n_joint, const int * joints, const double * refs) override;
    return_t relativeMove(int j, double delta) override;
    return_t relativeMove(const double * deltas) override;
    return_t relativeMove(int n_joint, const int * joints, const double * deltas) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t checkMotionDone(int j, bool & flag) override;
    return_t checkMotionDone(bool & flag) override;
    return_t checkMotionDone(const std::vector<int> & joints, bool & flag) override;
    return_t setTrajSpeed(int j, double sp) override;
    return_t setTrajSpeeds(const double * spds) override;
    return_t setTrajSpeeds(int n_joint, const int * joints, const double * spds) override;
    return_t getTrajSpeed(int j, double * ref) override;
    return_t getTrajSpeeds(double * spds) override;
    return_t getTrajSpeeds(int n_joint, const int * joints, double * spds) override;
    return_t setTrajAcceleration(int j, double acc) override;
    return_t setTrajAccelerations(const double * accs) override;
    return_t setTrajAccelerations(int n_joint, const int * joints, const double * accs) override;
    return_t getTrajAcceleration(int j, double * acc) override;
    return_t getTrajAccelerations(double * accs) override;
    return_t getTrajAccelerations(int n_joint, const int * joints, double * accs) override;
#else
    return_t checkMotionDone(int j, bool * flag) override;
    return_t checkMotionDone(bool * flag) override;
    return_t checkMotionDone(int n_joint, const int * joints, bool * flags) override;
    return_t setRefSpeed(int j, double sp) override;
    return_t setRefSpeeds(const double * spds) override;
    return_t setRefSpeeds(int n_joint, const int * joints, const double * spds) override;
    return_t getRefSpeed(int j, double * ref) override;
    return_t getRefSpeeds(double * spds) override;
    return_t getRefSpeeds(int n_joint, const int * joints, double * spds) override;
    return_t setRefAcceleration(int j, double acc) override;
    return_t setRefAccelerations(const double * accs) override;
    return_t setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    return_t getRefAcceleration(int j, double * acc) override;
    return_t getRefAccelerations(double * accs) override;
    return_t getRefAccelerations(int n_joint, const int * joints, double * accs) override;
#endif
    return_t stop(int j) override;
    return_t stop() override;
    return_t stop(int n_joint, const int * joints) override;
    return_t getTargetPosition(int joint, double * ref) override;
    return_t getTargetPositions(double * refs) override;
    return_t getTargetPositions(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------
    return_t setPosition(int j, double ref) override;
    return_t setPositions(int n_joint, const int * joints, const double * refs) override;
    return_t setPositions(const double * refs) override;
    return_t getRefPosition(int joint, double * ref) override;
    return_t getRefPositions(double * refs) override;
    return_t getRefPositions(int n_joint, const int * joints, double * refs) override;

    // ---------- IEncodersTimed Declarations. Implementation in IEncoderImpl.cpp ----------
    return_t resetEncoder(int j) override;
    return_t resetEncoders() override;
    return_t setEncoder(int j, double val) override;
    return_t setEncoders(const double * vals) override;
    return_t getEncoder(int j, double * v) override;
    return_t getEncoders(double * encs) override;
    return_t getEncodersTimed(double * encs, double * time) override;
    return_t getEncoderTimed(int j, double * encs, double * time) override;
    return_t getEncoderSpeed(int j, double * sp) override;
    return_t getEncoderSpeeds(double * spds) override;
    return_t getEncoderAcceleration(int j, double * spds) override;
    return_t getEncoderAccelerations(double * accs) override;

    // --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------
    return_t velocityMove(int j, double sp) override;
    return_t velocityMove(const double * sp) override;
    return_t velocityMove(int n_joint, const int * joints, const double * spds) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getTargetVelocity(int joint, double * vel) override;
    return_t getTargetVelocities(double * vels) override;
    return_t getTargetVelocities(int n_joint, const int * joints, double * vels) override;
#else
    return_t getRefVelocity(int joint, double * vel) override;
    return_t getRefVelocities(double * vels) override;
    return_t getRefVelocities(int n_joint, const int * joints, double * vels) override;
#endif

    // --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPosLimits(int axis, double min, double max) override;
    return_t getPosLimits(int axis, double * min, double * max) override;
#else
    return_t setLimits(int axis, double min, double max) override;
    return_t getLimits(int axis, double * min, double * max) override;
#endif
    return_t setVelLimits(int axis, double min, double max) override;
    return_t getVelLimits(int axis, double * min, double * max) override;

    // --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override;
    return_t getControlMode(int j, yarp::dev::ControlModeEnum & mode) override;
    return_t getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes) override;
    return_t getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override;
    return_t setControlMode(int j, yarp::dev::SelectableControlModeEnum mode) override;
    return_t setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    return_t setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
#else
    return_t getControlMode(int j, int * mode) override;
    return_t getControlModes(int * modes) override;
    return_t getControlModes(int n_joint, const int * joints, int * modes) override;
    return_t setControlMode(int j, const int mode) override;
    return_t setControlModes(int n_joint, const int * joints, int * modes) override;
    return_t setControlModes(int * modes) override;
#endif

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------
    bool threadInit() override;
    void run() override;

protected:
    // ----- Shared Area Funcion declarations. Implementation in SharedArea.cpp -----
    void setEncRaw(const int index, const double position);
    void setEncsRaw(const std::vector<double> & positions);

    double getEncRaw(const int index);
    std::vector<double> getEncsRaw();

    double getEncExposed(const int index);
    std::vector<double> getEncsExposed();

private:
    enum jmc_state { NOT_CONTROLLING, POSITION_MOVE, RELATIVE_MOVE, VELOCITY_MOVE };
    enum jmc_mode { POSITION_MODE, VELOCITY_MODE, POSITION_DIRECT_MODE, UNKNOWN_MODE };

    // General Joint Motion Controller parameters //
    jmc_mode controlMode {UNKNOWN_MODE};
    double lastTime {0.0};

    std::mutex encRawMutex; // SharedArea

    std::vector<jmc_state> jointStatus;

    std::vector<double> encRaw;
    std::vector<double> refAcc; // Exposed.
    std::vector<double> targetExposed; // Exposed.
    std::vector<double> velRaw;
};

#endif // __EMULATED_CONTROL_BOARD_HPP__
