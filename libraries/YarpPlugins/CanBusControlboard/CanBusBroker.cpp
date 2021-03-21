// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <memory>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

CanBusBroker::CanBusBroker(const std::string & _name)
    : name(_name),
      readerThread(nullptr),
      writerThread(nullptr),
      iCanBus(nullptr),
      iCanBusErrors(nullptr),
      iCanBufferFactory(nullptr),
      busLoadMonitor(nullptr)
{ }

// -----------------------------------------------------------------------------

CanBusBroker::~CanBusBroker()
{
    stopThreads();

    dumpPort.close();
    sendPort.close();
    sdoPort.close();
    busLoadPort.close();

    delete busLoadMonitor;
    delete readerThread;
    delete writerThread;
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

    if (config.check("busLoadPeriod", "CAN bus load monitor period (seconds)"))
    {
        double busLoadPeriod = config.find("busLoadPeriod").asFloat64();

        if (busLoadPeriod <= 0.0)
        {
            CD_WARNING("Illegal CAN bus load monitor option period: %f.\n", busLoadPeriod);
            return false;
        }

        busLoadMonitor = new BusLoadMonitor(busLoadPeriod);
    }

    readerThread = new CanReaderThread(name, rxDelay, rxBufferSize);
    writerThread = new CanWriterThread(name, txDelay, txBufferSize);

    if (config.check("name", "YARP port prefix for remote CAN interface"))
    {
        return createPorts(config.find("name").asString());
    }

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

    if (busLoadMonitor)
    {
        unsigned int bitrate;

        if (!iCanBus->canGetBaudRate(&bitrate))
        {
            CD_WARNING("Cannot get bitrate.\n");
            return false;
        }

        busLoadMonitor->setBitrate(bitrate);
    }

    if (readerThread)
    {
        readerThread->setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory);
    }

    if (writerThread)
    {
        writerThread->setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::createPorts(const std::string & prefix)
{
    if (!dumpPort.open(prefix + "/dump:o"))
    {
        CD_WARNING("Cannot open dump port.\n");
        return false;
    }

    if (!sendPort.open(prefix + "/send:i"))
    {
        CD_WARNING("Cannot open send port.\n");
        return false;
    }

    if (!sdoPort.open(prefix + "/sdo:s"))
    {
        CD_WARNING("Cannot open SDO port.\n");
        return false;
    }

    if (busLoadMonitor && !busLoadPort.open(prefix + "/load:o"))
    {
        CD_WARNING("Cannot open bus load port.\n");
        return false;
    }

    if (readerThread)
    {
        readerThread->attachDumpWriter(&dumpPort, &dumpWriter, &dumpMutex);
        readerThread->attachCanNotifier(&sdoReplier);
        readerThread->attachBusLoadMonitor(busLoadMonitor->getReadMonitor());
    }

    if (writerThread)
    {
        writerThread->attachDumpWriter(&dumpPort, &dumpWriter, &dumpMutex);
        writerThread->attachBusLoadMonitor(busLoadMonitor->getWriteMonitor());
        sdoReplier.configureSender(writerThread->getDelegate());
    }

    dumpPort.setWriteOnly();
    dumpWriter.attach(dumpPort);

    sendPort.setOutputMode(false);
    commandReader.attach(sendPort);
    commandReader.useCallback(*this);

    sdoPort.setReader(sdoReplier);

    if (busLoadMonitor)
    {
        busLoadPort.setInputMode(false);
        busLoadMonitor->attach(busLoadPort);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::addFilters()
{
    if (!iCanBus || !readerThread)
    {
        return false;
    }

    for (auto * handle : readerThread->getHandles())
    {
        auto ids = handle->getAdditionalIds();
        ids.push_back(handle->getId());

        for (auto id : ids)
        {
            if (!iCanBus->canIdAdd(id))
            {
                CD_WARNING("Cannot add acceptance filter ID %d.\n", id);
                return false;
            }
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
    if (busLoadMonitor && !busLoadMonitor->start())
    {
        CD_WARNING("Cannot start bus load monitor thread.\n");
        return false;
    }

    if (!readerThread || !readerThread->start())
    {
        CD_WARNING("Cannot start reader thread.\n");
        return false;
    }

    if (!writerThread || !writerThread->start())
    {
        CD_WARNING("Cannot start writer thread.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stopThreads()
{
    sendPort.interrupt();
    commandReader.disableCallback();
    sdoPort.interrupt();

    if (busLoadMonitor && busLoadMonitor->isRunning())
    {
        busLoadMonitor->stop();
    }

    bool ok = true;

    if (readerThread && readerThread->isRunning() && !readerThread->stop())
    {
        CD_WARNING("Cannot stop reader thread.\n");
        ok = false;
    }

    if (writerThread && writerThread->isRunning() && !writerThread->stop())
    {
        CD_WARNING("Cannot stop writer thread.\n");
        ok = false;
    }

    // keep out ports last to avoid deadlock (happened sometimes with dumpPort)
    dumpPort.interrupt();
    busLoadPort.interrupt();

    return ok;
}

// -----------------------------------------------------------------------------

void CanBusBroker::onRead(yarp::os::Bottle & b)
{
    if (b.size() != 1 && b.size() != 2)
    {
        CD_WARNING("Illegal size %zu, expected [1,2].\n", b.size());
        return;
    }

    unsigned int id = b.get(0).asInt32();

    if (id > 0x7FF)
    {
        CD_WARNING("Illegal COB-ID: 0x%x.\n", id);
        return;
    }

    unsigned int size = 0;
    std::unique_ptr<std::uint8_t[]> raw;

    if (b.size() == 2)
    {
        if (!b.get(1).isList())
        {
            CD_WARNING("Second element is not a list.\n");
            return;
        }

        const yarp::os::Bottle * data = b.get(1).asList();
        size = data->size();

        if (size == 0 || size > 8)
        {
            CD_WARNING("Empty data or size exceeds 8 elements: %d.\n", size);
            return;
        }

        raw = std::make_unique<std::uint8_t[]>(size);

        for (int i = 0; i < size; i++)
        {
            raw[i] = data->get(i).asInt8();
        }
    }

    can_message msg {id, size, raw.get()};
    writerThread->getDelegate()->prepareMessage(msg);
    CD_INFO("Remote command: %s\n", CanUtils::msgToStr(msg).c_str());
}

// -----------------------------------------------------------------------------
