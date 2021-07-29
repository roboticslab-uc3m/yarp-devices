// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <memory>

#include <yarp/os/LogStream.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

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

    busLoadMonitor = nullptr;
    readerThread = nullptr;
    writerThread = nullptr;
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
        yCWarning(CBCB) << "Illegal CAN bus buffer size or delay options";
        return false;
    }

    readerThread = new CanReaderThread(name, rxDelay, rxBufferSize);
    writerThread = new CanWriterThread(name, txDelay, txBufferSize);

    if (config.check("name", "YARP port prefix for remote CAN interface"))
    {
        if (config.check("busLoadPeriod", "CAN bus load monitor period (seconds)"))
        {
            double busLoadPeriod = config.find("busLoadPeriod").asFloat64();

            if (busLoadPeriod <= 0.0)
            {
                yCWarning(CBCB) << "Illegal CAN bus load monitor option period:" << busLoadPeriod;
                return false;
            }

            busLoadMonitor = new BusLoadMonitor(busLoadPeriod);
        }

        return createPorts(config.find("name").asString());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::registerDevice(yarp::dev::PolyDriver * driver)
{
    if (!driver->view(iCanBus))
    {
        yCWarning(CBCB) << "Cannot view ICanBus interface";
        return false;
    }

    if (!driver->view(iCanBusErrors))
    {
        yCWarning(CBCB) << "Cannot view ICanBusErrors interface";
        return false;
    }

    if (!driver->view(iCanBufferFactory))
    {
        yCWarning(CBCB) << "Cannot view ICanBufferFactory interface";
        return false;
    }

    if (busLoadMonitor)
    {
        unsigned int bitrate;

        if (!iCanBus->canGetBaudRate(&bitrate))
        {
            yCWarning(CBCB) << "Cannot get bitrate";
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
        yCWarning(CBCB) << "Cannot open dump port";
        return false;
    }

    if (!sendPort.open(prefix + "/send:i"))
    {
        yCWarning(CBCB) << "Cannot open send port";
        return false;
    }

    if (!sdoPort.open(prefix + "/sdo:s"))
    {
        yCWarning(CBCB) << "Cannot open SDO port";
        return false;
    }

    if (busLoadMonitor && !busLoadPort.open(prefix + "/load:o"))
    {
        yCWarning(CBCB) << "Cannot open bus load port";
        return false;
    }

    if (readerThread)
    {
        readerThread->attachDumpWriter(&dumpPort, &dumpWriter, &dumpMutex);
        readerThread->attachCanNotifier(&sdoReplier);

        if (busLoadMonitor)
        {
            readerThread->attachBusLoadMonitor(busLoadMonitor->getReadMonitor());
        }
    }

    if (writerThread)
    {
        writerThread->attachDumpWriter(&dumpPort, &dumpWriter, &dumpMutex);

        if (busLoadMonitor)
        {
            writerThread->attachBusLoadMonitor(busLoadMonitor->getWriteMonitor());
        }

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
                yCWarning(CBCB) << "Cannot add acceptance filter ID" << id;
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
        yCWarning(CBCB) << "CAN filters on bus" << name << "may be preserved on the next run";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::startThreads()
{
    if (busLoadMonitor && !busLoadMonitor->start())
    {
        yCWarning(CBCB) << "Cannot start bus load monitor thread";
        return false;
    }

    if (!readerThread || !readerThread->start())
    {
        yCWarning(CBCB) << "Cannot start reader thread";
        return false;
    }

    if (!writerThread || !writerThread->start())
    {
        yCWarning(CBCB) << "Cannot start writer thread";
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
        yCWarning(CBCB) << "Cannot stop reader thread";
        ok = false;
    }

    if (writerThread && writerThread->isRunning() && !writerThread->stop())
    {
        yCWarning(CBCB) << "Cannot stop writer thread";
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
        yCWarning(CBCB, "Illegal size %zu, expected [1,2]", b.size());
        return;
    }

    unsigned int id = b.get(0).asInt32();

    if (id > 0x7FF)
    {
        yCWarning(CBCB, "Illegal COB-ID: 0x%x", id);
        return;
    }

    unsigned int size = 0;
    std::unique_ptr<std::uint8_t[]> raw;

    if (b.size() == 2)
    {
        if (!b.get(1).isList())
        {
            yCWarning(CBCB, "Second element is not a list");
            return;
        }

        const yarp::os::Bottle * data = b.get(1).asList();
        size = data->size();

        if (size == 0 || size > 8)
        {
            yCWarning(CBCB, "Empty data or size exceeds 8 elements: %d", size);
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
    yCInfo(CBCB, "Remote command: %s", CanUtils::msgToStr(msg).c_str());
}

// -----------------------------------------------------------------------------
