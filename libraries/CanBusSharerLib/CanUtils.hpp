// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_UTILS_HPP__
#define __CAN_UTILS_HPP__

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <string>
#include <type_traits>

namespace roboticslab
{
namespace CanUtils
{

std::string msgToStr(std::uint8_t id, std::uint16_t cob, std::size_t len, const std::uint8_t * data);

inline std::string msgToStr(std::uint16_t cobId, std::size_t len, const std::uint8_t * data)
{ return msgToStr(cobId & 0x7F, cobId & 0xFF80, len, data); }

template<typename T_int, typename T_frac>
void encodeFixedPoint(double value, T_int * integer, T_frac * fractional)
{
    static_assert(std::is_integral<T_int>::value && std::is_integral<T_frac>::value, "Integral required.");
    *integer = static_cast<T_int>(value);
    *fractional = std::abs(value - *integer) * (1 << 8 * sizeof(T_frac));
}

template<typename T_int, typename T_frac>
double decodeFixedPoint(T_int integer, T_frac fractional)
{
    static_assert(std::is_integral<T_int>::value && std::is_integral<T_frac>::value, "Integral required.");
    double frac = static_cast<double>(fractional) / (1 << 8 * sizeof(T_frac));
    return integer + (integer >= 0 ? frac : -frac);
}

} // namespace CanUtils
} // namespace roboticslab

#endif // __CAN_UTILS_HPP__
