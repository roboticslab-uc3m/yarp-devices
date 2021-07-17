// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_SOCKET_HPP__
#define __CAN_BUS_SOCKET_HPP__

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "SocketCanMessage.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusSocket
 * @brief Contains roboticslab::CanBusSocket.
 *
 * See <a href="https://www.kernel.org/doc/html/latest/networking/can.html">documentation</a>.
 */
class CanBusSocket : public yarp::dev::DeviceDriver,
                     public yarp::dev::ICanBus,
                     public yarp::dev::ICanBusErrors,
                     private yarp::dev::ImplementCanBufferFactory<SocketCanMessage, struct can_frame>
{
public:
    ~CanBusSocket() override
    { close(); }

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

    //  --------- ICanBusErrors declarations. Implementation in ICanBusErrorsImpl.cpp ---------
    bool canGetErrors(yarp::dev::CanErrors & err) override;

private:
    std::string iface;
    int s {0};
};

} // namespace roboticslab

#endif // __CAN_BUS_SOCKET_HPP__
