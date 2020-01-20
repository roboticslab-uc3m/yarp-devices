// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

CanBusBroker::CanBusBroker(const std::string & _name)
    : name(_name),
      reader(nullptr),
      writer(nullptr),
      iCanBus(nullptr),
      iCanBusErrors(nullptr),
      iCanBufferFactory(nullptr)
{ }

// -----------------------------------------------------------------------------

CanBusBroker::~CanBusBroker()
{
    delete reader;
    delete writer;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::configure(const yarp::os::Searchable & config)
{
    int rxBufferSize = config.check("rxBufferSize", yarp::os::Value(0), "CAN bus RX buffer size").asInt32();
    int txBufferSize = config.check("txBufferSize", yarp::os::Value(0), "CAN bus TX buffer size").asInt32();

    double rxDelay = config.check("rxDelay", yarp::os::Value(0.0), "CAN bus RX delay (seconds)").asFloat64();
    double txDelay = config.check("txDelay", yarp::os::Value(0.0), "CAN bus TX delay (seconds)").asFloat64();

    if (rxBufferSize <= 0 || txBufferSize <= 0 || rxDelay <= 0.0 || txDelay <= 0.0)
    {
        CD_WARNING("Illegal CAN bus buffer size or delay options.\n");
        return false;
    }

    reader = new CanReaderThread(name, rxDelay, rxBufferSize);
    writer = new CanWriterThread(name, txDelay, txBufferSize);

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::registerDevice(yarp::dev::PolyDriver * driver)
{
    if (!driver->view(iCanBus))
    {
        CD_WARNING("Cannot view ICanBus interface.\n");
        return false;
    }

    if (!driver->view(iCanBusErrors))
    {
        CD_WARNING("Cannot view ICanBusErrors interface.\n");
        return false;
    }

    if (!driver->view(iCanBufferFactory))
    {
        CD_WARNING("Cannot view ICanBufferFactory interface.\n");
        return false;
    }

    if (reader)
    {
        reader->setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory);
    }

    if (writer)
    {
        writer->setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::createPorts(const std::string & name)
{
    if (!dumpPort.open(name + "/dump:o"))
    {
        CD_WARNING("Cannot open dump port.\n");
        return false;
    }

    if (reader)
    {
        reader->attachDumpPort(&dumpPort);
    }

    if (writer)
    {
        writer->attachDumpPort(&dumpPort);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::addFilters()
{
    if (!iCanBus || !reader)
    {
        return false;
    }

    for (const auto & entry : reader->getHandleMap())
    {
        if (!iCanBus->canIdAdd(entry.first))
        {
            CD_WARNING("Cannot add acceptance filter ID %d.\n", entry.first);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::clearFilters()
{
    if (!iCanBus)
    {
        return false;
    }

    // Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
    if (!iCanBus->canIdDelete(0))
    {
        CD_WARNING("CAN filters on bus %s may be preserved on the next run.\n", name.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::startThreads()
{
    if (!reader || !reader->start())
    {
        CD_WARNING("Cannot start reader thread.\n");
        return false;
    }

    if (!writer || !writer->start())
    {
        CD_WARNING("Cannot start writer thread.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stopThreads()
{
    bool ok = true;

    if (reader && reader->isRunning() && !reader->stop())
    {
        CD_WARNING("Cannot stop reader thread.\n");
        ok = false;
    }

    if (writer && writer->isRunning() && !writer->stop())
    {
        CD_WARNING("Cannot stop writer thread.\n");
        ok = false;
    }

    return ok;
}

// -----------------------------------------------------------------------------
