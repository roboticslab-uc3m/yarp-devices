// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_UTILS_HPP__
#define __CAN_UTILS_HPP__

#include <stdint.h>

#include <string>

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{
namespace CanUtils
{

std::string msgToStr(uint32_t id, uint32_t cob, unsigned char len, const uint8_t * msgData);

inline std::string msgToStr(const yarp::dev::CanMessage & message)
{ return msgToStr(message.getId() & 0x7F, message.getId() & 0xFF80, message.getLen(), message.getData()); }

template<typename T_int, typename T_frac>
void encodeFixedPoint(double value, T_int * integer, T_frac * fractional)
{
    *integer = static_cast<T_int>(value);
    *fractional = std::abs(value - *integer) * (1 << 8 * sizeof(T_frac));
}

template<typename T_int, typename T_frac>
double decodeFixedPoint(T_int integer, T_frac fractional)
{
    double frac = static_cast<double>(fractional) / (1 << 8 * sizeof(T_frac));
    return integer + (integer >= 0 ? frac : -frac);
}

} // namespace CanUtils
} // namespace roboticslab

#endif // __CAN_UTILS_HPP__
