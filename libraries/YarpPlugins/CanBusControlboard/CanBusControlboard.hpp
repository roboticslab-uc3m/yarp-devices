// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_CONTROLBOARD_HPP__
#define __CAN_BUS_CONTROLBOARD_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/CanBusInterface.h>

#include <stdlib.h>  //-- Just for ::exit()
#include <fcntl.h>  //-- Just for O_RDWR
#include <vector>
#include <map>
#include <list>
#include <sstream>

#include "PositionDirectThread.hpp"

// -- Pause
#include <stdlib.h>
#include <stdio.h>

//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be managed from father CMake.
#include "ColorDebug.h"

#include "DeviceMapper.hpp"
#include "CanRxTxThreads.hpp"
#include "ICanBusSharer.hpp"

#define CHECK_JOINT(j) do { int n = deviceMapper.getControlledAxes(); if ((j) < 0 || (j) > n - 1) return false; } while (0)

#define DEFAULT_MODE "position"
#define DEFAULT_CUI_TIMEOUT 1.0
#define DEFAULT_CAN_BUS "CanBusHico"
#define DEFAULT_LIN_INTERP_PERIOD_MS 50
#define DEFAULT_LIN_INTERP_BUFFER_SIZE 1
#define DEFAULT_LIN_INTERP_MODE "pt"

#define DEFAULT_CAN_RX_BUFFER_SIZE 500
#define DEFAULT_CAN_TX_BUFFER_SIZE 500
#define DEFAULT_CAN_RX_PERIOD_MS -1
#define DEFAULT_CAN_TX_PERIOD_MS 1.0
#define DEFAULT_CAN_SDO_TIMEOUT_MS 25.0
#define DEFAULT_CAN_DRIVE_STATE_TIMEOUT 2.5

namespace roboticslab
{

/**
 *
 * @ingroup YarpPlugins
 * \defgroup CanBusControlboard
 * @brief Contains roboticslab::CanBusControlboard.
 */

/**
* @ingroup CanBusControlboard
* @brief Implements IControlLimits, IControlMode, IEncodersTimed, IPositionControl, IPositionDirect,
* ITorqueControl, IVelocityControl interface yarp::dev class member functions, linking to roboticslab::TechnosoftIpos,
* roboticslab::LacqueyFetch and/or roboticslab::FakeJoint raw implementations.
*
*/
class CanBusControlboard : public yarp::dev::DeviceDriver,
                           public yarp::dev::IControlLimits,
                           public yarp::dev::IControlMode,
                           public yarp::dev::ICurrentControl,
                           public yarp::dev::IEncodersTimed,
                           public yarp::dev::IInteractionMode,
                           public yarp::dev::IPositionControl,
                           public yarp::dev::IPositionDirect,
                           public yarp::dev::IRemoteVariables,
                           public yarp::dev::ITorqueControl,
                           public yarp::dev::IVelocityControl
{
public:

    //  --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------

    virtual bool setLimits(int axis, double min, double max) override;
    virtual bool getLimits(int axis, double * min, double * max) override;
    virtual bool setVelLimits(int axis, double min, double max) override;
    virtual bool getVelLimits(int axis, double * min, double * max) override;

    //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

    virtual bool getControlMode(int j, int * mode) override;
    virtual bool getControlModes(int * modes) override;
    virtual bool getControlModes(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlMode(int j, const int mode) override;
    virtual bool setControlModes(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlModes(int * modes) override;

    //  --------- ICurrentControl Declarations. Implementation in ICurrentControlImpl.cpp ---------

    virtual bool getNumberOfMotors(int * ax) override;
    virtual bool getCurrent(int m, double * curr) override;
    virtual bool getCurrents(double * currs) override;
    virtual bool getCurrentRange(int m, double * min, double * max) override;
    virtual bool getCurrentRanges(double * min, double * max) override;
    virtual bool setRefCurrent(int m, double curr) override;
    virtual bool setRefCurrents(const double * currs) override;
    virtual bool setRefCurrents(int n_motor, const int * motors, const double * currs) override;
    virtual bool getRefCurrents(double * currs) override;
    virtual bool getRefCurrent(int m, double *curr) override;

    //  ---------- IEncoders Declarations. Implementation in IEncodersImpl.cpp ----------

    virtual bool getAxes(int * ax) override;
    virtual bool resetEncoder(int j) override;
    virtual bool resetEncoders() override;
    virtual bool setEncoder(int j, double val) override;
    virtual bool setEncoders(const double * vals) override;
    virtual bool getEncoder(int j, double * v) override;
    virtual bool getEncoders(double * encs) override;
    virtual bool getEncoderSpeed(int j, double * sp) override;
    virtual bool getEncoderSpeeds(double * spds) override;
    virtual bool getEncoderAcceleration(int j, double * spds) override;
    virtual bool getEncoderAccelerations(double * accs) override;

    //  ---------- IEncodersTimed Declarations. Implementation in IEncodersImpl.cpp ----------

    virtual bool getEncoderTimed(int j, double * encs, double * time) override;
    virtual bool getEncodersTimed(double * encs, double * time) override;

    // -----------IInteractionMode Declarations. Implementation in IInteractionModeImpl.cpp --------------

    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode) override;
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    virtual bool getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    virtual bool setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool positionMove(int j, double ref) override;
    virtual bool positionMove(const double * refs) override;
    virtual bool positionMove(int n_joint, const int * joints, const double * refs) override;
    virtual bool relativeMove(int j, double delta) override;
    virtual bool relativeMove(const double * deltas) override;
    virtual bool relativeMove(int n_joint, const int * joints, const double * deltas) override;
    virtual bool checkMotionDone(int j, bool * flag) override;
    virtual bool checkMotionDone(bool * flag) override;
    virtual bool checkMotionDone(int n_joint, const int * joints, bool * flags) override;
    virtual bool setRefSpeed(int j, double sp) override;
    virtual bool setRefSpeeds(const double * spds) override;
    virtual bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override;
    virtual bool setRefAcceleration(int j, double acc) override;
    virtual bool setRefAccelerations(const double * accs) override;
    virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    virtual bool getRefSpeeds(int n_joint, const int * joints, double * spds) override;
    virtual bool getRefSpeed(int j, double * ref) override;
    virtual bool getRefSpeeds(double * spds) override;
    virtual bool getRefAcceleration(int j, double * acc) override;
    virtual bool getRefAccelerations(double * accs) override;
    virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    virtual bool stop(int j) override;
    virtual bool stop() override;
    virtual bool stop(int n_joint, const int *joints) override;
    virtual bool getTargetPosition(int joint, double * ref) override;
    virtual bool getTargetPositions(double * refs) override;
    virtual bool getTargetPositions(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool setPosition(int j, double ref) override;
    virtual bool setPositions(const double * refs) override;
    virtual bool setPositions(int n_joint, const int * joints, const double * refs) override;
    virtual bool getRefPosition(int joint, double * ref) override;
    virtual bool getRefPositions(double *refs) override;
    virtual bool getRefPositions(int n_joint, const int * joints, double * refs) override;

    // -----------IRemoteVariables Declarations. Implementation in IRemoteVariablesImpl.cpp --------------

    virtual bool getRemoteVariable(std::string key, yarp::os::Bottle & val) override;
    virtual bool setRemoteVariable(std::string key, const yarp::os::Bottle & val) override;
    virtual bool getRemoteVariablesList(yarp::os::Bottle * listOfKeys) override;

    // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------

    //virtual bool getAxes(int * ax) override;
    virtual bool getRefTorques(double *t);
    virtual bool getRefTorque(int j, double *t);
    virtual bool setRefTorques(const double *t);
    virtual bool setRefTorque(int j, double t);
    virtual bool setRefTorques(const int n_joint, const int *joints, const double *t);
    virtual bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params);
    virtual bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params);
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    //virtual bool getAxes(int * ax) override;
    virtual bool velocityMove(int j, double sp) override;
    virtual bool velocityMove(const double * sp) override;
    virtual bool velocityMove(int n_joint, const int * joints, const double * spds) override;
    virtual bool getRefVelocity(int joint, double * vel) override;
    virtual bool getRefVelocities(double * vels) override;
    virtual bool getRefVelocities(int n_joint, const int * joints, double * vels) override;
    //virtual bool setRefAcceleration(int j, double acc) override;
    //virtual bool setRefAccelerations(const double * accs) override;
    //virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    //virtual bool getRefAcceleration(int j, double * acc) override;
    //virtual bool getRefAccelerations(double * accs) override;
    //virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    //virtual bool stop(int j) override;
    //virtual bool stop() override;
    //virtual bool stop(int n_joint, const int *joints) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

private:

    yarp::dev::PolyDriver canBusDevice;
    yarp::dev::ICanBus* iCanBus;

    yarp::dev::PolyDriverList nodes;
    std::vector< ICanBusSharer* > iCanBusSharer;

    DeviceMapper deviceMapper;

    std::vector< int > motorIds;
    std::map< int, int > idxFromCanId;

    CanReaderThread * canReaderThread;
    CanWriterThread * canWriterThread;

    PositionDirectThread * posdThread;
    int linInterpPeriodMs;
    int linInterpBufferSize;
    std::string linInterpMode;
};

} // namespace roboticslab

#endif //  __CAN_BUS_CONTROLBOARD_HPP__
