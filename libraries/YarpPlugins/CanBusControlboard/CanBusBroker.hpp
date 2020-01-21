// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_BROKER_HPP__
#define __CAN_BUS_BROKER_HPP__

#include <mutex>
#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/PortWriterBuffer.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/TypedReaderCallback.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>

#include "CanRxTxThreads.hpp"
#include "SdoClient.hpp"

namespace roboticslab
{

constexpr yarp::conf::vocab32_t VOCAB_SDO_UPLOAD = yarp::os::createVocab('s', 'd', 'o', 'u');
constexpr yarp::conf::vocab32_t VOCAB_SDO_DOWNLOAD = yarp::os::createVocab('s', 'd', 'o', 'd');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I8_TYPE = yarp::os::createVocab('i', '8');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI8_TYPE = yarp::os::createVocab('u', 'i', '8');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I16_TYPE = yarp::os::createVocab('i', '1', '6');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI16_TYPE = yarp::os::createVocab('u', 'i', '1', '6');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I32_TYPE = yarp::os::createVocab('i', '3', '2');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI32_TYPE = yarp::os::createVocab('u', 'i', '3', '2');
constexpr yarp::conf::vocab32_t VOCAB_SDO_STRING_TYPE = yarp::os::createVocab('s', 't', 'r');

/**
 * @ingroup CanBusControlboard
 * @brief ...
 *
 * TODO
 */
class SdoResponder
{
public:
    SdoResponder() : sdoRequest(nullptr)
    { }

    virtual ~SdoResponder()
    { delete sdoRequest; }

    virtual bool notify(unsigned int id, const unsigned char * data) = 0;

protected:
    enum class sdo_direction : yarp::conf::vocab32_t
    {
        UPLOAD = VOCAB_SDO_UPLOAD,
        DOWNLOAD = VOCAB_SDO_DOWNLOAD
    };

    enum class data_type : yarp::conf::vocab32_t
    {
        INTEGER_8 = VOCAB_SDO_I8_TYPE,
        UNSIGNED_INTEGER_8 = VOCAB_SDO_UI8_TYPE,
        INTEGER_16 = VOCAB_SDO_I16_TYPE,
        UNSIGNED_INTEGER_16 = VOCAB_SDO_UI16_TYPE,
        INTEGER_32 = VOCAB_SDO_I32_TYPE,
        UNSIGNED_INTEGER_32 = VOCAB_SDO_UI32_TYPE,
        STRING = VOCAB_SDO_STRING_TYPE
    };

    static SdoClient * allocateClient(unsigned int id)
    { return new SdoClient(id, SDO_RX, SDO_TX, SDO_TIMEOUT); }

    SdoClient * sdoRequest;

private:
    static constexpr unsigned int SDO_RX = 0x600;
    static constexpr unsigned int SDO_TX = 0x580;
    static constexpr double SDO_TIMEOUT = 0.25; // seconds
};

/**
 * @ingroup CanBusControlboard
 * @brief ...
 *
 * TODO: https://whatis.techtarget.com/definition/message-broker
 */
class CanBusBroker final : public yarp::os::PortReader,
                           public yarp::os::TypedReaderCallback<yarp::os::Bottle>,
                           public SdoResponder
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
    { return readerThread; }

    CanWriterThread * getWriter() const
    { return writerThread; }

    std::string getName() const
    { return name; }

    virtual void onRead(yarp::os::Bottle & b) override;
    virtual bool read(yarp::os::ConnectionReader & reader) override;

    virtual bool notify(unsigned int cobId, const unsigned char * data) override;

private:
    std::string name;

    CanReaderThread * readerThread;
    CanWriterThread * writerThread;

    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBusErrors * iCanBusErrors;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;

    yarp::os::Port dumpPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> dumpWriter;
    std::mutex dumpMutex;

    yarp::os::Port sendPort;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> commandReader;

    yarp::os::RpcServer sdoPort;
};

} // namespace roboticslab

#endif // __CAN_BUS_BROKER_HPP__
