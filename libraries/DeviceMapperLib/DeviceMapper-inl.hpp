// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEVICE_MAPPER_INL_HPP__
#define __DEVICE_MAPPER_INL_HPP__

namespace roboticslab // gcc <7 bug
{

template<>
inline yarp::dev::IAmplifierControlRaw * RawDevice::getHandle() const
{ return iAmplifierControlRaw; }

template<>
inline yarp::dev::IAxisInfoRaw * RawDevice::getHandle() const
{ return iAxisInfoRaw; }

template<>
inline yarp::dev::IControlCalibrationRaw * RawDevice::getHandle() const
{ return iControlCalibrationRaw; }

template<>
inline yarp::dev::IControlLimitsRaw * RawDevice::getHandle() const
{ return iControlLimitsRaw; }

template<>
inline yarp::dev::IControlModeRaw * RawDevice::getHandle() const
{ return iControlModeRaw; }

template<>
inline yarp::dev::ICurrentControlRaw * RawDevice::getHandle() const
{ return iCurrentControlRaw; }

template<>
inline yarp::dev::IEncodersTimedRaw * RawDevice::getHandle() const
{ return iEncodersTimedRaw; }

template<>
inline yarp::dev::IImpedanceControlRaw * RawDevice::getHandle() const
{ return iImpedanceControlRaw; }

template<>
inline yarp::dev::IInteractionModeRaw * RawDevice::getHandle() const
{ return iInteractionModeRaw; }

template<>
inline yarp::dev::IMotorRaw * RawDevice::getHandle() const
{ return iMotorRaw; }

template<>
inline yarp::dev::IMotorEncodersRaw * RawDevice::getHandle() const
{ return iMotorEncodersRaw; }

template<>
inline yarp::dev::IPidControlRaw * RawDevice::getHandle() const
{ return iPidControlRaw; }

template<>
inline yarp::dev::IPositionControlRaw * RawDevice::getHandle() const
{ return iPositionControlRaw; }

template<>
inline yarp::dev::IPositionDirectRaw * RawDevice::getHandle() const
{ return iPositionDirectRaw; }

template<>
inline yarp::dev::IPWMControlRaw * RawDevice::getHandle() const
{ return iPWMControlRaw; }

template<>
inline yarp::dev::IRemoteVariablesRaw * RawDevice::getHandle() const
{ return iRemoteVariablesRaw; }

template<>
inline yarp::dev::IVelocityControlRaw * RawDevice::getHandle() const
{ return iVelocityControlRaw; }

template<>
inline yarp::dev::ITorqueControlRaw * RawDevice::getHandle() const
{ return iTorqueControlRaw; }

} // namespace roboticslab

#endif // __DEVICE_MAPPER_INL_HPP__
