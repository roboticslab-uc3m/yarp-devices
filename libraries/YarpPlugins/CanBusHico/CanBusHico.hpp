// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_HICO__
#define __CAN_BUS_HICO__

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

#include "hico_api.h"
#include "ICanBusHico.h"
#include "HicoCanMessage.hpp"
#include "ColorDebug.hpp"

namespace roboticslab
{

/**
 *
 * @ingroup CanBusHico
 * @brief Specifies the HicoCan (hcanpci) behaviour and specifications.
 *
 */
class CanBusHico : public ICanBusHico,
                   public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   private yarp::dev::ImplementCanBufferFactory<HicoCanMessage, can_msg>
{

public:

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    /** Initialize the CAN device.
     * @param device is the device path, such as "/dev/can0".
     * @param bitrate is the bitrate, such as BITRATE_100k.
     * @return true/false on success/failure.
     */
    virtual bool open(yarp::os::Searchable& config);

    /** Close the CAN device. */
    virtual bool close();

    //  --------- ICanBusHico declarations. Implementation in CanBusHico.cpp ---------

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

    //  --------- ICanBus declarations. Implementation in ICanBusImpl.cpp ---------

    virtual bool canSetBaudRate(unsigned int rate);

    virtual bool canGetBaudRate(unsigned int * rate);

    virtual bool canIdAdd(unsigned int id);

    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false);

    virtual bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false);

protected:

    /** CAN file descriptor */
    int fileDescriptor;

    yarp::os::Semaphore canBusReady;

};

}  // namespace roboticslab

#endif  // __CAN_BUS_HICO__
