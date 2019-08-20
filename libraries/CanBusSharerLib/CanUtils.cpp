// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanUtils.hpp"

#include <iomanip>
#include <sstream>

using namespace roboticslab::CanUtils;

std::string msgToStr(uint8_t id, uint16_t cob, size_t len, const uint8_t * msgData)
{
    std::stringstream tmp;

    for (size_t i = 0; i < len - 1; i++)
    {
        tmp << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(msgData[i]) << " ";
    }

    tmp << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(msgData[len - 1]);
    tmp << ". canId(";
    tmp << std::dec << id;
    tmp << ") via(";
    tmp << std::hex << cob;
    tmp << ").";

    return tmp.str();
}
