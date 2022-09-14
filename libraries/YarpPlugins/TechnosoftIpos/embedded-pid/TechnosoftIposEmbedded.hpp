// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_EMBEDDED_HPP__
#define __TECHNOSOFT_IPOS_EMBEDDED_HPP__

#include <atomic>
#include <bitset>

#include "TechnosoftIposBase.hpp"
#include "embedded-pid/InterpolatedPositionBuffer.hpp"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief A TechnosoftIposBase implementation using the firmware-embedded PID.
 */
class TechnosoftIposEmbedded : public TechnosoftIposBase
{
public:
    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    bool synchronize() override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool positionMoveRaw(int j, double ref) override;
    bool positionMoveRaw(const double * refs) override;
    bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    bool relativeMoveRaw(int j, double delta) override;
    bool relativeMoveRaw(const double * deltas) override;
    bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    bool checkMotionDoneRaw(int j, bool * flag) override;
    bool checkMotionDoneRaw(bool * flag) override;
    bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override;
    bool setRefSpeedRaw(int j, double sp) override;
    bool setRefSpeedsRaw(const double * spds) override;
    bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    bool setRefAccelerationRaw(int j, double acc) override;
    bool setRefAccelerationsRaw(const double * accs) override;
    bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    bool getRefSpeedRaw(int j, double * ref) override;
    bool getRefSpeedsRaw(double * spds) override;
    bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    bool getRefAccelerationRaw(int j, double * acc) override;
    bool getRefAccelerationsRaw(double * accs) override;
    bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    bool stopRaw(int j) override;
    bool stopRaw() override;
    bool stopRaw(int n_joint, const int * joints) override;
    bool getTargetPositionRaw(int joint, double * ref) override;
    bool getTargetPositionsRaw(double * refs) override;
    bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool setPositionRaw(int j, double ref) override;
    bool setPositionsRaw(const double * refs) override;
    bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override;
    bool getRefPositionRaw(int joint, double * ref) override;
    bool getRefPositionsRaw(double * refs) override;
    bool getRefPositionsRaw(int n_joint, const int *joints, double * refs) override;

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override;
    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override;
    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    //bool getAxes(int * ax) override;
    bool velocityMoveRaw(int j, double sp) override;
    bool velocityMoveRaw(const double * sp) override;
    bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override;
    bool getRefVelocityRaw(int joint, double * vel) override;
    bool getRefVelocitiesRaw(double * vels) override;
    bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override;
    //bool setRefAccelerationRaw(int j, double acc) override;
    //bool setRefAccelerationsRaw(const double * accs) override;
    //bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    //bool getRefAccelerationRaw(int j, double * acc) override;
    //bool getRefAccelerationsRaw(double * accs) override;
    //bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    //bool stopRaw(int j) override;
    //bool stopRaw() override;
    //bool stopRaw(int n_joint, const int *joints) override;

private:
    void interpretModesOfOperation(std::int8_t modesOfOperation) override;
    void interpretIpStatus(std::uint16_t ipStatus) override;
    void reset() override;

    InterpolatedPositionBuffer * ipBuffer;

    std::bitset<16> ipStatus;

    std::atomic<bool> ipMotionStarted {false};
    std::atomic<bool> ipBufferFilled {false};
    std::atomic<bool> ipBufferEnabled {false};

    std::atomic<bool> enableSync {false};
    std::atomic<bool> enableCsv {false};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_EMBEDDED_HPP__
