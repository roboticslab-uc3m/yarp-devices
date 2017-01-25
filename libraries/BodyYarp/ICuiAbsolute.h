// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CUI_ABSOLUTE__
#define __I_CUI_ABSOLUTE__

#include <stdint.h>

namespace teo
{

/**
 *
 * @brief Abstract base for a CuiAbsolute.
 *
 */
class ICuiAbsolute
{
public:
    /**
     * Destructor.
     */
    virtual ~ICuiAbsolute() {}

    virtual bool startContinuousPublishing(uint8_t time) = 0;
    virtual bool startPullPublishing() = 0;
    virtual bool stopPublishingMessages() = 0;
    virtual bool HasFirstReached() = 0;

};

}  // namespace teo

#endif  //  __I_CUI_ABSOLUTE__
