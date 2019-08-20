// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_INL_HPP__
#define __SDO_CLIENT_INL_HPP__

#include "SdoClient.hpp"

namespace roboticslab // gcc <7 bug
{

template<>
inline bool SdoClient::upload(const std::string & name, std::string * s, uint16_t index, uint8_t subindex)
{
    const uint32_t maxLen = 100; // arbitrary high value
    char buf[maxLen] = {0};

    if (!uploadInternal(name, buf, maxLen, index, subindex))
    {
        return false;
    }

    *s = std::string(buf);
    return true;
}

template<>
inline bool SdoClient::upload(const std::string & name, std::function<void(std::string * s)> fn, uint16_t index, uint8_t subindex)
{
    std::string s;

    if (!upload(name, &s, index, subindex))
    {
        return false;
    }

    fn(&s);
    return true;
}

template<>
inline bool SdoClient::download(const std::string & name, const std::string & s, uint16_t index, uint8_t subindex)
{
    char * buf = new char[s.size()];
    s.copy(buf, s.size());
    bool res = downloadInternal(name, buf, s.size(), index, subindex);
    delete[] buf;
    return res;
}

} // namespace roboticslab

#endif // __SDO_CLIENT_INL_HPP__
