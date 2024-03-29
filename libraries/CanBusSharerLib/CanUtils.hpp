// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_UTILS_HPP__
#define __CAN_UTILS_HPP__

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <string>
#include <type_traits>

#include "CanMessage.hpp"

//! Contains CAN-related utilities.
namespace roboticslab::CanUtils
{

/**
 * @ingroup CanBusSharerLib
 * @brief Create a string representation of the given CAN message details.
 */
std::string msgToStr(std::uint8_t id, std::uint16_t cob, std::size_t len, const std::uint8_t * data);

/**
 * @ingroup CanBusSharerLib
 * @brief Create a string representation of the given CAN message data payload.
 */
std::string msgToStr(std::size_t len, const std::uint8_t * data);

/**
 * @ingroup CanBusSharerLib
 * @brief Create a string representation of the given CAN message details.
 */
inline std::string msgToStr(std::uint16_t cobId, std::size_t len, const std::uint8_t * data)
{ return msgToStr(cobId & 0x7F, cobId & 0xFF80, len, data); }

/**
 * @ingroup CanBusSharerLib
 * @brief Create a string representation of the given CAN message details.
 */
inline std::string msgToStr(const can_message & msg)
{ return msgToStr(msg.id & 0x7F, msg.id & 0xFF80, msg.len, msg.data); }

/**
 * @ingroup CanBusSharerLib
 * @brief Obtain a fixed-point representation given a float value.
 * @tparam T_int Integer part, integral type expected.
 * @tparam T_frac Fractional part, integral type expected.
 */
template<typename T_int, typename T_frac>
void encodeFixedPoint(double value, T_int * integer, T_frac * fractional)
{
    static_assert(std::is_integral_v<T_int> && std::is_integral_v<T_frac>, "Integral required.");
    static_assert(std::is_unsigned_v<T_frac>, "Unsigned fractional type required.");
    double _int;
    *fractional = std::round(std::modf(value, &_int) * (1 << 8 * sizeof(T_frac)));
    *integer = static_cast<T_int>(_int);
}

/**
 * @ingroup CanBusSharerLib
 * @brief Obtain a float representation of a fixed-point value.
 * @tparam T_int Integer part, integral type expected.
 * @tparam T_frac Fractional part, integral type expected.
 */
template<typename T_int, typename T_frac>
double decodeFixedPoint(T_int integer, T_frac fractional)
{
    static_assert(std::is_integral_v<T_int> && std::is_integral_v<T_frac>, "Integral required.");
    static_assert(std::is_unsigned_v<T_frac>, "Unsigned fractional type required.");
    double frac = static_cast<double>(fractional) / (1 << 8 * sizeof(T_frac));
    return integer + (integer >= 0 ? frac : -frac);
}

} // namespace roboticslab::CanUtils

#endif // __CAN_UTILS_HPP__
