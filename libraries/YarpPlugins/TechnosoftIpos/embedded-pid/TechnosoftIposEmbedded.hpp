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

    bool synchronize(double timestamp) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool setControlModeRaw(int j, int mode) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    bool positionMoveRaw(int j, double ref) override;
    bool relativeMoveRaw(int j, double delta) override;
    bool checkMotionDoneRaw(int j, bool * flag) override;
    bool setRefSpeedRaw(int j, double sp) override;
    bool setRefAccelerationRaw(int j, double acc) override;
    bool getRefSpeedRaw(int j, double * ref) override;
    bool getRefAccelerationRaw(int j, double * acc) override;
    bool stopRaw(int j) override;
    bool getTargetPositionRaw(int joint, double * ref) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    bool setPositionRaw(int j, double ref) override;
    bool getRefPositionRaw(int joint, double * ref) override;

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override;
    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override;
    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    bool velocityMoveRaw(int j, double sp) override;
    bool getRefVelocityRaw(int joint, double * vel) override;

private:
    void interpretModesOfOperation(std::int8_t modesOfOperation) override;
    void interpretIpStatus(std::uint16_t ipStatus) override;
    void reset() override;

    InterpolatedPositionBuffer * ipBuffer {nullptr};
    std::string ipMode {"pt"};
    int ipPeriodMs {50};

    std::bitset<16> ipStatus;

    std::atomic<bool> ipMotionStarted {false};
    std::atomic<bool> ipBufferFilled {false};
    std::atomic<bool> ipBufferEnabled {false};

    std::atomic<bool> enableSync {false};
    std::atomic<bool> enableCsv {false};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_EMBEDDED_HPP__
