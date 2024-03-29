// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RawDevice.hpp"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

namespace roboticslab // gcc <7 bug
{

struct RawDevice::Private
{
    // control board interfaces
    yarp::dev::IAmplifierControlRaw * iAmplifierControlRaw;
    yarp::dev::IAxisInfoRaw * iAxisInfoRaw;
    yarp::dev::IControlCalibrationRaw * iControlCalibrationRaw;
    yarp::dev::IControlLimitsRaw * iControlLimitsRaw;
    yarp::dev::IControlModeRaw * iControlModeRaw;
    yarp::dev::ICurrentControlRaw * iCurrentControlRaw;
    yarp::dev::IEncodersTimedRaw * iEncodersTimedRaw;
    yarp::dev::IImpedanceControlRaw * iImpedanceControlRaw;
    yarp::dev::IInteractionModeRaw * iInteractionModeRaw;
    yarp::dev::IJointFaultRaw * iJointFaultRaw;
    yarp::dev::IMotorRaw * iMotorRaw;
    yarp::dev::IMotorEncodersRaw * iMotorEncodersRaw;
    yarp::dev::IPidControlRaw * iPidControlRaw;
    yarp::dev::IPositionControlRaw * iPositionControlRaw;
    yarp::dev::IPositionDirectRaw * iPositionDirectRaw;
    yarp::dev::IPWMControlRaw * iPWMControlRaw;
    yarp::dev::IRemoteVariablesRaw * iRemoteVariablesRaw;
    yarp::dev::IVelocityControlRaw * iVelocityControlRaw;
    yarp::dev::ITorqueControlRaw * iTorqueControlRaw;

    // multiple analog sensors interfaces
    yarp::dev::IThreeAxisGyroscopes * iThreeAxisGyroscopes;
    yarp::dev::IThreeAxisLinearAccelerometers * iThreeAxisLinearAccelerometers;
    yarp::dev::IThreeAxisMagnetometers * iThreeAxisMagnetometers;
    yarp::dev::IOrientationSensors * iOrientationSensors;
    yarp::dev::ITemperatureSensors * iTemperatureSensors;
    yarp::dev::ISixAxisForceTorqueSensors * iSixAxisForceTorqueSensors;
    yarp::dev::IContactLoadCellArrays * iContactLoadCellArrays;
    yarp::dev::IEncoderArrays * iEncoderArrays;
    yarp::dev::ISkinPatches * iSkinPatches;
    yarp::dev::IPositionSensors * iPositionSensors;
};

RawDevice::RawDevice(yarp::dev::PolyDriver * _driver)
    : priv(std::make_unique<Private>()),
      driver(_driver)
{
    if (driver)
    {
        auto * impl = driver->getImplementation();

        // control board interfaces
        valid |= impl->view(priv->iAmplifierControlRaw);
        valid |= impl->view(priv->iAxisInfoRaw);
        valid |= impl->view(priv->iControlCalibrationRaw);
        valid |= impl->view(priv->iControlLimitsRaw);
        valid |= impl->view(priv->iControlModeRaw);
        valid |= impl->view(priv->iCurrentControlRaw);
        valid |= impl->view(priv->iEncodersTimedRaw);
        valid |= impl->view(priv->iImpedanceControlRaw);
        valid |= impl->view(priv->iInteractionModeRaw);
        valid |= impl->view(priv->iJointFaultRaw);
        valid |= impl->view(priv->iMotorRaw);
        valid |= impl->view(priv->iMotorEncodersRaw);
        valid |= impl->view(priv->iPidControlRaw);
        valid |= impl->view(priv->iPositionControlRaw);
        valid |= impl->view(priv->iPositionDirectRaw);
        valid |= impl->view(priv->iPWMControlRaw);
        valid |= impl->view(priv->iRemoteVariablesRaw);
        valid |= impl->view(priv->iVelocityControlRaw);
        valid |= impl->view(priv->iTorqueControlRaw);

        // multiple analog sensors interfaces
        valid |= impl->view(priv->iThreeAxisGyroscopes);
        valid |= impl->view(priv->iThreeAxisLinearAccelerometers);
        valid |= impl->view(priv->iThreeAxisMagnetometers);
        valid |= impl->view(priv->iOrientationSensors);
        valid |= impl->view(priv->iTemperatureSensors);
        valid |= impl->view(priv->iSixAxisForceTorqueSensors);
        valid |= impl->view(priv->iContactLoadCellArrays);
        valid |= impl->view(priv->iEncoderArrays);
        valid |= impl->view(priv->iSkinPatches);
        valid |= impl->view(priv->iPositionSensors);
    }
}

RawDevice::~RawDevice() = default;

// explicit (full) specializations, no explicit instantiations required

// control board interfaces

template<>
yarp::dev::IAmplifierControlRaw * RawDevice::getHandle() const
{ return priv->iAmplifierControlRaw; }

template<>
yarp::dev::IAxisInfoRaw * RawDevice::getHandle() const
{ return priv->iAxisInfoRaw; }

template<>
yarp::dev::IControlCalibrationRaw * RawDevice::getHandle() const
{ return priv->iControlCalibrationRaw; }

template<>
yarp::dev::IControlLimitsRaw * RawDevice::getHandle() const
{ return priv->iControlLimitsRaw; }

template<>
yarp::dev::IControlModeRaw * RawDevice::getHandle() const
{ return priv->iControlModeRaw; }

template<>
yarp::dev::ICurrentControlRaw * RawDevice::getHandle() const
{ return priv->iCurrentControlRaw; }

template<>
yarp::dev::IEncodersRaw * RawDevice::getHandle() const
{ return priv->iEncodersTimedRaw; }

template<>
yarp::dev::IEncodersTimedRaw * RawDevice::getHandle() const
{ return priv->iEncodersTimedRaw; }

template<>
yarp::dev::IImpedanceControlRaw * RawDevice::getHandle() const
{ return priv->iImpedanceControlRaw; }

template<>
yarp::dev::IInteractionModeRaw * RawDevice::getHandle() const
{ return priv->iInteractionModeRaw; }

template<>
yarp::dev::IJointFaultRaw * RawDevice::getHandle() const
{ return priv->iJointFaultRaw; }

template<>
yarp::dev::IMotorRaw * RawDevice::getHandle() const
{ return priv->iMotorRaw; }

template<>
yarp::dev::IMotorEncodersRaw * RawDevice::getHandle() const
{ return priv->iMotorEncodersRaw; }

template<>
yarp::dev::IPidControlRaw * RawDevice::getHandle() const
{ return priv->iPidControlRaw; }

template<>
yarp::dev::IPositionControlRaw * RawDevice::getHandle() const
{ return priv->iPositionControlRaw; }

template<>
yarp::dev::IPositionDirectRaw * RawDevice::getHandle() const
{ return priv->iPositionDirectRaw; }

template<>
yarp::dev::IPWMControlRaw * RawDevice::getHandle() const
{ return priv->iPWMControlRaw; }

template<>
yarp::dev::IRemoteVariablesRaw * RawDevice::getHandle() const
{ return priv->iRemoteVariablesRaw; }

template<>
yarp::dev::IVelocityControlRaw * RawDevice::getHandle() const
{ return priv->iVelocityControlRaw; }

template<>
yarp::dev::ITorqueControlRaw * RawDevice::getHandle() const
{ return priv->iTorqueControlRaw; }

// multiple analog sensors interfaces

template<>
yarp::dev::IThreeAxisGyroscopes * RawDevice::getHandle() const
{ return priv->iThreeAxisGyroscopes; }

template<>
yarp::dev::IThreeAxisLinearAccelerometers * RawDevice::getHandle() const
{ return priv->iThreeAxisLinearAccelerometers; }

template<>
yarp::dev::IThreeAxisMagnetometers * RawDevice::getHandle() const
{ return priv->iThreeAxisMagnetometers; }

template<>
yarp::dev::IOrientationSensors * RawDevice::getHandle() const
{ return priv->iOrientationSensors; }

template<>
yarp::dev::ITemperatureSensors * RawDevice::getHandle() const
{ return priv->iTemperatureSensors; }

template<>
yarp::dev::ISixAxisForceTorqueSensors * RawDevice::getHandle() const
{ return priv->iSixAxisForceTorqueSensors; }

template<>
yarp::dev::IContactLoadCellArrays * RawDevice::getHandle() const
{ return priv->iContactLoadCellArrays; }

template<>
yarp::dev::IEncoderArrays * RawDevice::getHandle() const
{ return priv->iEncoderArrays; }

template<>
yarp::dev::ISkinPatches * RawDevice::getHandle() const
{ return priv->iSkinPatches; }

template<>
yarp::dev::IPositionSensors * RawDevice::getHandle() const
{ return priv->iPositionSensors; }

} // namespace roboticslab
