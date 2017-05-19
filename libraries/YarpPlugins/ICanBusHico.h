// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CAN_BUS_HICO__
#define __I_CAN_BUS_HICO__

#include "hico_api.h"

#define DEFAULT_CAN_DEVICE "/dev/can0"
#define DEFAULT_CAN_BITRATE BITRATE_1000k

#define DELAY 0.001  // [s] Required when using same driver.

namespace roboticslab
{

/**
 *
 * @brief Abstract base for a CAN bus hico.
 *
 */
class ICanBusHico
{
public:
    /**
     * Destructor.
     */
    virtual ~ICanBusHico() {}

    /**
     * Write message to the CAN buffer.
     * @param id Message's COB-id
     * @param len Data field length
     * @param msgData Data to send
     * @return true/false on success/failure.
     */
    virtual bool sendRaw(uint32_t id, uint16_t len, uint8_t * msgData) = 0;

    /** Read data.
     * @return Number on got, 0 on timeout, and errno on fail. */
    virtual int read_timeout(struct can_msg *buf, unsigned int timeout) = 0;

};

}  // namespace roboticslab

#endif  //  __I_CAN_BUS_HICO__
