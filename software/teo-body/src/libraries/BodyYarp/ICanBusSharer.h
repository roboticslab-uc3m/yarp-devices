// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CAN_BUS_SHARER__
#define __I_CAN_BUS_SHARER__

#include "bodybot/CanBusHico.hpp"

namespace teo
{

class ICanBusSharer
{
public:
    /**
     * Destructor.
     */
    virtual ~ICanBusSharer() {}

    /**
     * Interpret a can bus message.
     * @return true/false.
     */
    virtual bool interpretMessage( can_msg * message) = 0;

    virtual bool setCanBusPtr(CanBusHico *canDevicePtr) = 0;

};

}  // namespace teo

#endif  //  __I_CAN_BUS_SHARER__
