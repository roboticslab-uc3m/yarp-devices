// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_BROKER_HPP__
#define __CAN_BUS_BROKER_HPP__

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Searchable.h>

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>

#include "CanRxTxThreads.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief ...
 *
 * TODO: https://whatis.techtarget.com/definition/message-broker
 */
class CanBusBroker final
{
public:
    CanBusBroker(const std::string & name);
    ~CanBusBroker();

    bool configure(const yarp::os::Searchable & config);
    bool registerDevice(yarp::dev::PolyDriver * driver);
    bool createPorts(const std::string & name);
    bool addFilters();
    bool clearFilters();
    bool startThreads();
    bool stopThreads();

    CanReaderThread * getReader() const
    { return reader; }

    CanWriterThread * getWriter() const
    { return writer; }

    std::string getName() const
    { return name; }

private:
    std::string name;

    CanReaderThread * reader;
    CanWriterThread * writer;

    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBusErrors * iCanBusErrors;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;

    yarp::os::BufferedPort<yarp::os::Bottle> dumpPort;
};

} // namespace roboticslab

#endif // __CAN_BUS_BROKER_HPP__
