// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanUtils.hpp"

#include <iomanip>
#include <sstream>

using namespace roboticslab;

std::string CanUtils::msgToStr(std::uint8_t id, std::uint16_t cob, std::size_t len, const std::uint8_t * data)
{
    std::stringstream tmp;

    for (std::size_t i = 0; i < len - 1; i++)
    {
        tmp << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(data[i]) << " ";
    }

    tmp << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(data[len - 1]);
    tmp << ". canId(";
    tmp << std::dec << static_cast<int>(id);
    tmp << ") via(0x";
    tmp << std::hex << cob;
    tmp << ").";

    return tmp.str();
}
