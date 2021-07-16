// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_SOCKET__
#define __CAN_BUS_SOCKET__

#include <string.h>  // bug, missing in yarp/dev/CanBusInterface.h as of YARP v2.3.70.2
#include <linux/can.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "SocketCanMessage.hpp"

namespace roboticslab
{

/**
 *
 * @ingroup CanBusSocket
 * @brief Specifies the SocketCan behaviour and specifications.
 *
 */
class CanBusSocket : public yarp::dev::DeviceDriver,
                     public yarp::dev::ICanBus,
                     private yarp::dev::ImplementCanBufferFactory<SocketCanMessage, struct can_frame>
{

public:

    CanBusSocket() {}

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    /** Initialize the CAN device.
     * @param device is the device path, such as "/dev/can0".
     * @param bitrate is the bitrate, such as BITRATE_100k.
     * @return true/false on success/failure.
     */
    virtual bool open(yarp::os::Searchable& config);

    /** Close the CAN device. */
    virtual bool close();

    //  --------- ICanBus declarations. Implementation in ICanBusImpl.cpp ---------

    virtual bool canSetBaudRate(unsigned int rate);

    virtual bool canGetBaudRate(unsigned int * rate);

    virtual bool canIdAdd(unsigned int id);

    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false);

    virtual bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false);

protected:

};

}  // namespace roboticslab

#endif  // __CAN_BUS_SOCKET__
