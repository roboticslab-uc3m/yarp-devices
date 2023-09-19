// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_FAKE__
#define __CAN_BUS_FAKE__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "FakeCanMessage.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusFake
 * @brief Contains roboticslab::CanBusFake.
 */

/**
 * @ingroup CanBusFake
 * @brief Fake CanBus driver, e.g. for testing CanBusBroker with pure USB devices.
 */
class CanBusFake : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   public yarp::dev::ICanBusErrors,
                   public yarp::dev::ImplementCanBufferFactory<FakeCanMessage, struct fake_can_msg>
{
public:

    //  --------- DeviceDriver declarations ---------

    virtual bool open(yarp::os::Searchable & config) override
    { return true; }

    virtual bool close() override
    { return true; }

    //  --------- ICanBus declarations ---------

    virtual bool canSetBaudRate(unsigned int rate) override
    { return true; }

    virtual bool canGetBaudRate(unsigned int * rate) override
    { *rate = 0; return true; }

    virtual bool canIdAdd(unsigned int id) override
    { return true; }

    virtual bool canIdDelete(unsigned int id) override
    { return true; }

    virtual bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false) override
    { return true; }

    virtual bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false) override
    { return true; }

    //  --------- ICanBusErrors declarations ---------

    virtual bool canGetErrors(yarp::dev::CanErrors & err) override
    { return true; }
};

} // namespace roboticslab

#endif // __CAN_BUS_FAKE__
