// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_FAKE__
#define __CAN_BUS_FAKE__

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>  // just for ::write
#include <err.h>
#include <errno.h>
#include <assert.h>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ICanBusHico.h"

#include "ColorDebug.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusFake
 * @brief Contains roboticslab::CanBusFake.
 */

/**
 * @ingroup CanBusFake
 * @brief Specifies the HicoCan (hcanpci) behaviour and specifications.
 */
class CanBusFake : public yarp::dev::DeviceDriver, public ICanBusHico
{

public:

    /** Initialize the CAN device.
     * @param device is the device path, such as "/dev/can0".
     * @param bitrate is the bitrate, such as BITRATE_100k.
     * @return true/false on success/failure.
     */
    virtual bool open(yarp::os::Searchable& config);

    /** Close the CAN device. */
    virtual bool close();

    /**
     * Write message to the CAN buffer.
     * @param id Message's COB-id
     * @param len Data field length
     * @param msgData Data to send
     * @return true/false on success/failure.
     */
    virtual bool sendRaw(uint32_t id, uint16_t len, uint8_t * msgData);

    /** Read data.
     * @return Number on got, 0 on timeout, and errno on fail. */
    virtual int read_timeout(struct can_msg *buf, unsigned int timeout);

private:

};

}  // namespace roboticslab

#endif  // __CAN_BUS_FAKE__

