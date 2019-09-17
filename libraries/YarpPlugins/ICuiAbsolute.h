// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CUI_ABSOLUTE_HPP__
#define __I_CUI_ABSOLUTE_HPP__

#include <cstdint>

namespace roboticslab
{

/**
 * @brief Abstract base for a CuiAbsolute.
 */
class ICuiAbsolute
{
public:
    virtual ~ICuiAbsolute() {}

    virtual bool startContinuousPublishing(std::uint8_t time) = 0;
    virtual bool startPullPublishing() = 0;
    virtual bool stopPublishingMessages() = 0;
};

}  // namespace roboticslab

#endif // __I_CUI_ABSOLUTE_HPP__
