// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <cstdint>

#include <ios>
#include <iomanip>
#include <memory>
#include <sstream>

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
      iCanBufferFactory(nullptr)
{ }

// -----------------------------------------------------------------------------

CanBusBroker::~CanBusBroker()
{
    stopThreads();

    dumpPort.close();
    sendPort.close();

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

    readerThread = new CanReaderThread(name, rxDelay, rxBufferSize);
    writerThread = new CanWriterThread(name, txDelay, txBufferSize);

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

bool CanBusBroker::createPorts(const std::string & name)
{
    if (!dumpPort.open(name + "/dump:o"))
    {
        CD_WARNING("Cannot open dump port.\n");
        return false;
    }

    if (!sendPort.open(name + "/send:i"))
    {
        CD_WARNING("Cannot open send port.\n");
        return false;
    }

    if (!sdoPort.open(name + "/sdo:s"))
    {
        CD_WARNING("Cannot open SDO port.\n");
        return false;
    }

    dumpPort.setInputMode(false);
    dumpWriter.attach(dumpPort);

    sendPort.setOutputMode(false);
    commandReader.attach(sendPort);
    commandReader.useCallback(*this);

    sdoPort.setReader(*this);

    if (readerThread)
    {
        readerThread->attachDumpWriter(&dumpWriter, &dumpMutex);
        readerThread->attachSdoResponder(this);
    }

    if (writerThread)
    {
        writerThread->attachDumpWriter(&dumpWriter, &dumpMutex);
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

    for (const auto & entry : readerThread->getHandleMap())
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
    dumpPort.interrupt();
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

    return ok;
}

// -----------------------------------------------------------------------------

void CanBusBroker::onRead(yarp::os::Bottle & b)
{
    if (b.size() != 1 && b.size() != 2)
    {
        CD_WARNING("Illegal size %d, expected [1,2].\n", b.size());
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

        raw = std::unique_ptr<std::uint8_t[]>(new std::uint8_t[size]);

        for (int i = 0; i < size; i++)
        {
            raw[i] = data->get(i).asInt8();
        }
    }

    writerThread->getDelegate()->prepareMessage({id, size, raw.get()});
    CD_INFO("Remote command: %s\n", CanUtils::msgToStr(id, size, raw.get()).c_str());
}

// -----------------------------------------------------------------------------

bool CanBusBroker::read(yarp::os::ConnectionReader & reader)
{
    if (this->sdoRequest)
    {
        return false;
    }

    yarp::os::Bottle request;
    yarp::os::Bottle response;

    if (!request.read(reader))
    {
        return false;
    }

    if (request.size() < 5 || !request.get(0).isVocab() || !request.get(1).isInt32()
            || !request.get(2).isInt32() || !request.get(3).isInt32() || !request.get(4).isVocab())
    {
        return false;
    }

    sdo_direction dir = static_cast<sdo_direction>(request.get(0).asVocab());
    unsigned int id = request.get(1).asInt8();
    unsigned int index = request.get(2).asInt16();
    unsigned int subindex = request.get(3).asInt8();
    data_type type = static_cast<data_type>(request.get(4).asVocab());

    if (dir == sdo_direction::DOWNLOAD && request.size() < 6)
    {
        return false;
    }

    std::unique_ptr<SdoClient> sdoRequest(this->sdoRequest = allocateClient(id));
    sdoRequest->configureSender(writerThread->getDelegate());

    if (dir == sdo_direction::UPLOAD)
    {
        yarp::os::ConnectionWriter * writer = reader.getWriter();

        if (!writer)
        {
            return false;
        }

        std::stringstream ss;
        bool ok = false;

        switch (type)
        {
        case data_type::INTEGER_8:
        {
            std::int8_t int8data;

            if (sdoRequest->upload("Remote request", &int8data, index, subindex))
            {
                ss << std::setw(2) << std::hex << std::showbase << (static_cast<long>(int8data) & 0xFF);
                response.addInt8(int8data);
                ok = true;
            }

            break;
        }
        case data_type::UNSIGNED_INTEGER_8:
        {
            std::uint8_t uint8data;

            if (sdoRequest->upload("Remote request", &uint8data, index, subindex))
            {
                ss << std::setw(2) << std::hex << std::showbase << (static_cast<unsigned long>(uint8data) & 0xFF);
                response.addInt16(uint8data);
                ok = true;
            }

            break;
        }
        case data_type::INTEGER_16:
        {
            std::int16_t int16data;

            if (sdoRequest->upload("Remote request", &int16data, index, subindex))
            {
                ss << std::setw(4) << std::hex << std::showbase << (static_cast<long>(int16data) & 0xFFFF);
                response.addInt16(int16data);
                ok = true;
            }

            break;
        }
        case data_type::UNSIGNED_INTEGER_16:
        {
            std::uint16_t uint16data;

            if (sdoRequest->upload("Remote request", &uint16data, index, subindex))
            {
                ss << std::setw(4) << std::hex << std::showbase << (static_cast<unsigned long>(uint16data) & 0xFFFF);
                response.addInt32(uint16data);
                ok = true;
            }

            break;
        }
        case data_type::INTEGER_32:
        {
            std::int32_t int32data;

            if (sdoRequest->upload("Remote request", &int32data, index, subindex))
            {
                ss << std::setw(8) << std::hex << std::showbase << (static_cast<long>(int32data) & 0xFFFFFFFF);
                response.addInt32(int32data);
                ok = true;
            }

            break;
        }
        case data_type::UNSIGNED_INTEGER_32:
        {
            std::uint32_t uint32data;

            if (sdoRequest->upload("Remote request", &uint32data, index, subindex))
            {
                ss << std::setw(8) << std::hex << std::showbase << (static_cast<unsigned long>(uint32data) & 0xFFFFFFFF);
                response.addInt64(uint32data);
                ok = true;
            }

            break;
        }
        case data_type::STRING:
        {
            std::string strData;

            if (sdoRequest->upload("Remote request", strData, index, subindex))
            {
                ss << strData;
                ok = true;
            }

            break;
        }
        default:
            return false;
        }

        if (!ok)
        {
            return false;
        }

        response.addString(ss.str());
        return response.write(*writer);
    }
    else if (dir == sdo_direction::DOWNLOAD)
    {
        switch (type)
        {
        case data_type::INTEGER_8:
        case data_type::UNSIGNED_INTEGER_8:
            return sdoRequest->download("Remote indication", request.get(5).asInt8(), index, subindex);
        case data_type::INTEGER_16:
        case data_type::UNSIGNED_INTEGER_16:
            return sdoRequest->download("Remote indication", request.get(5).asInt16(), index, subindex);
        case data_type::INTEGER_32:
        case data_type::UNSIGNED_INTEGER_32:
            return sdoRequest->download("Remote indication", request.get(5).asInt32(), index, subindex);
        case data_type::STRING:
            return sdoRequest->download("Remote indication", request.get(5).asString(), index, subindex);
        default:
            return false;
        }
    }
    else
    {
        return false;
    }
}

// -----------------------------------------------------------------------------

bool CanBusBroker::notify(unsigned int cobId, const unsigned char * data)
{
    if (sdoRequest && sdoRequest->getCobIdTx() == cobId)
    {
        return sdoRequest->notify(data);
    }

    return false;
}

// -----------------------------------------------------------------------------
