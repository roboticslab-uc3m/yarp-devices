// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_SOCKET_HPP__
#define __CAN_BUS_SOCKET_HPP__

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "SocketCanMessage.hpp"
#include "CanBusSocket_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusSocket
 * @brief Contains CanBusSocket.
 *
 * See <a href="https://www.kernel.org/doc/html/latest/networking/can.html">documentation</a>.
 */
class CanBusSocket : public yarp::dev::DeviceDriver,
                     public yarp::dev::ICanBus,
                     public yarp::dev::ImplementCanBufferFactory<roboticslab::SocketCanMessage, struct can_frame>,
                     public CanBusSocket_ParamsParser
{
public:
    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- ICanBus declarations. Implementation in ICanBusImpl.cpp ---------
    bool canSetBaudRate(unsigned int rate) override;
    bool canGetBaudRate(unsigned int * rate) override;
    bool canIdAdd(unsigned int id) override;
    bool canIdDelete(unsigned int id) override;
    bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false) override;
    bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false) override;

private:
    enum io_operation { READ, WRITE };

    bool waitUntilTimeout(io_operation op, bool * bufferReady);
    void interpretErrorFrame(const struct can_frame * msg);

    int socketDescriptor {0};
    std::vector<struct can_filter> filters;
};

#endif // __CAN_BUS_SOCKET_HPP__
