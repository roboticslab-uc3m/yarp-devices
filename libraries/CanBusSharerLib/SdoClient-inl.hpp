// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_INL_HPP__
#define __SDO_CLIENT_INL_HPP__

#include "SdoClient.hpp"

template<>
inline bool roboticslab::SdoClient::upload(const std::string & name, std::string * data, uint16_t index, uint8_t subindex)
{
    const size_t maxLen = 100; // arbitrary high value
    char buf[maxLen] = {0};

    if (!uploadInternal(name, buf, maxLen, index, subindex))
    {
        return false;
    }

    * data = std::string(buf);
    return true;
}

#endif // __SDO_CLIENT_INL_HPP__
