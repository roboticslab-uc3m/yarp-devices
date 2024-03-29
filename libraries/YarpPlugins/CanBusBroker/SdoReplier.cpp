// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoReplier.hpp"

#include <iomanip>
#include <ios>
#include <memory>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"
#include "SdoClient.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

constexpr yarp::conf::vocab32_t VOCAB_SDO_UPLOAD = yarp::os::createVocab32('s', 'd', 'o', 'u');
constexpr yarp::conf::vocab32_t VOCAB_SDO_DOWNLOAD = yarp::os::createVocab32('s', 'd', 'o', 'd');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I8_TYPE = yarp::os::createVocab32('i', '8');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI8_TYPE = yarp::os::createVocab32('u', 'i', '8');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I16_TYPE = yarp::os::createVocab32('i', '1', '6');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI16_TYPE = yarp::os::createVocab32('u', 'i', '1', '6');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I32_TYPE = yarp::os::createVocab32('i', '3', '2');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI32_TYPE = yarp::os::createVocab32('u', 'i', '3', '2');
constexpr yarp::conf::vocab32_t VOCAB_SDO_STRING_TYPE = yarp::os::createVocab32('s', 't', 'r');
constexpr yarp::conf::vocab32_t VOCAB_SDO_OK = yarp::os::createVocab32('o', 'k');
constexpr yarp::conf::vocab32_t VOCAB_SDO_FAIL = yarp::os::createVocab32('f', 'a', 'i', 'l');
constexpr yarp::conf::vocab32_t VOCAB_SDO_HELP = yarp::os::createVocab32('h', 'e', 'l', 'p');

namespace
{
    yarp::os::Bottle makeUsage()
    {
        return {
            yarp::os::Value("format: <direction> <id> <index> <subindex> <type> [<data (only download)>]"),
            yarp::os::Value("> direction ([u]pload: request from drive; [d]ownload: send to drive):"),
            yarp::os::Value(VOCAB_SDO_UPLOAD, true),
            yarp::os::Value(VOCAB_SDO_DOWNLOAD, true),
            yarp::os::Value("> available types ([i]nteger, [u]nsigned integer, [str]ing):"),
            yarp::os::Value(VOCAB_SDO_I8_TYPE, true),
            yarp::os::Value(VOCAB_SDO_UI8_TYPE, true),
            yarp::os::Value(VOCAB_SDO_I16_TYPE, true),
            yarp::os::Value(VOCAB_SDO_UI16_TYPE, true),
            yarp::os::Value(VOCAB_SDO_I32_TYPE, true),
            yarp::os::Value(VOCAB_SDO_UI32_TYPE, true),
            yarp::os::Value(VOCAB_SDO_STRING_TYPE, true)
        };
    }

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

    class ConnectionGuard
    {
    public:
        ConnectionGuard(yarp::os::Bottle * _response, yarp::os::ConnectionWriter * _writer)
            : response(_response), writer(_writer), sdo(nullptr), success(false)
        { }

        ~ConnectionGuard()
        {
            if (sdo)
            {
                delete *sdo;
                *sdo = nullptr;
            }

            if (response && writer)
            {
                response->addVocab32(success ? VOCAB_SDO_OK : VOCAB_SDO_FAIL);
                response->write(*writer);
            }
        }

        void attachSdoHandle(SdoClient ** sdo)
        { this->sdo = sdo; }

        bool inhibit()
        { response = nullptr; writer = nullptr; return true; }

        bool flip()
        { success = true; return true; }

    private:
        yarp::os::Bottle * response;
        yarp::os::ConnectionWriter * writer;
        SdoClient ** sdo;
        bool success;
    };
}

// -----------------------------------------------------------------------------

class SdoReplier::Private
{
public:
    SdoClient ** allocate(unsigned int id, ICanSenderDelegate * sender)
    {
        sdoClient = new SdoClient(id, SDO_COB_RX, SDO_COB_TX, SDO_TIMEOUT, sender);
        return &sdoClient;
    }

    SdoClient * sdo()
    { return sdoClient; }

private:
    SdoClient * sdoClient = nullptr;

    static constexpr unsigned int SDO_COB_RX = 0x600;
    static constexpr unsigned int SDO_COB_TX = 0x580;
    static constexpr double SDO_TIMEOUT = 0.25; // seconds
};

// -----------------------------------------------------------------------------

SdoReplier::SdoReplier()
    : priv(new Private),
      sender(nullptr)
{ }

// -----------------------------------------------------------------------------

SdoReplier::~SdoReplier()
{
    delete priv;
}

// -----------------------------------------------------------------------------

bool SdoReplier::read(yarp::os::ConnectionReader & reader)
{
    yarp::os::ConnectionWriter * writer = reader.getWriter();

    yarp::os::Bottle request;
    yarp::os::Bottle response;

    if (!writer || !request.read(reader))
    {
        return false;
    }

    if (request.size() == 1 && request.get(0).asVocab32() == VOCAB_SDO_HELP)
    {
        static auto usage = makeUsage();
        yarp::os::Bottle reply;
        reply.addVocab32('m', 'a', 'n', 'y');
        reply.append(usage);
        return reply.write(*writer);
    }

    ConnectionGuard guard(&response, writer);

    if (request.size() < 5)
    {
        yCWarning(CBB) << "SDO requests require at least 5 elements, got" << request.size();
        return false;
    }

    sdo_direction dir = static_cast<sdo_direction>(request.get(0).asVocab32());
    unsigned int id = request.get(1).asInt8();
    unsigned int index = request.get(2).asInt16();
    unsigned int subindex = request.get(3).asInt8();
    data_type type = static_cast<data_type>(request.get(4).asVocab32());

    guard.attachSdoHandle(priv->allocate(id, sender));

    if (dir == sdo_direction::UPLOAD)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::internal << std::hex << std::showbase;
        bool ok = false;

        switch (type)
        {
        case data_type::INTEGER_8:
        {
            std::int8_t int8data;

            if (priv->sdo()->upload("Remote request", &int8data, index, subindex))
            {
                ss << std::setw(4) << (static_cast<long>(int8data) & 0xFF);
                response.addInt8(int8data);
                ok = true;
            }

            break;
        }
        case data_type::UNSIGNED_INTEGER_8:
        {
            std::uint8_t uint8data;

            if (priv->sdo()->upload("Remote request", &uint8data, index, subindex))
            {
                ss << std::setw(4) << (static_cast<unsigned long>(uint8data) & 0xFF);
                response.addInt16(uint8data);
                ok = true;
            }

            break;
        }
        case data_type::INTEGER_16:
        {
            std::int16_t int16data;

            if (priv->sdo()->upload("Remote request", &int16data, index, subindex))
            {
                ss << std::setw(6) << (static_cast<long>(int16data) & 0xFFFF);
                response.addInt16(int16data);
                ok = true;
            }

            break;
        }
        case data_type::UNSIGNED_INTEGER_16:
        {
            std::uint16_t uint16data;

            if (priv->sdo()->upload("Remote request", &uint16data, index, subindex))
            {
                ss << std::setw(6) << (static_cast<unsigned long>(uint16data) & 0xFFFF);
                response.addInt32(uint16data);
                ok = true;
            }

            break;
        }
        case data_type::INTEGER_32:
        {
            std::int32_t int32data;

            if (priv->sdo()->upload("Remote request", &int32data, index, subindex))
            {
                ss << std::setw(10) << (static_cast<long>(int32data) & 0xFFFFFFFF);
                response.addInt32(int32data);
                ok = true;
            }

            break;
        }
        case data_type::UNSIGNED_INTEGER_32:
        {
            std::uint32_t uint32data;

            if (priv->sdo()->upload("Remote request", &uint32data, index, subindex))
            {
                ss << std::setw(10) << (static_cast<unsigned long>(uint32data) & 0xFFFFFFFF);
                response.addInt64(uint32data);
                ok = true;
            }

            break;
        }
        case data_type::STRING:
        {
            std::string strData;

            if (priv->sdo()->upload("Remote request", strData, index, subindex))
            {
                ss << strData;
                ok = true;
            }

            break;
        }
        default:
            yCWarning(CBB) << "Invalid data type" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(type));
            return false;
        }

        if (!ok)
        {
            return false;
        }

        response.addString(ss.str());
        return response.write(*writer) && guard.inhibit();
    }
    else if (dir == sdo_direction::DOWNLOAD)
    {
        if (request.size() != 6)
        {
            yCWarning(CBB) << "Download SDO requires exactly 6 elements, got" << request.size();
            return false;
        }

        const yarp::os::Value & data = request.get(5);

        switch (type)
        {
        case data_type::INTEGER_8:
        case data_type::UNSIGNED_INTEGER_8:
            return priv->sdo()->download("Remote indication", data.asInt8(), index, subindex) && guard.flip();
        case data_type::INTEGER_16:
        case data_type::UNSIGNED_INTEGER_16:
            return priv->sdo()->download("Remote indication", data.asInt16(), index, subindex) && guard.flip();
        case data_type::INTEGER_32:
        case data_type::UNSIGNED_INTEGER_32:
            return priv->sdo()->download("Remote indication", data.asInt32(), index, subindex) && guard.flip();
        case data_type::STRING:
            return priv->sdo()->download("Remote indication", data.asString(), index, subindex) && guard.flip();
        default:
            yCWarning(CBB) << "Invalid data type" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(type));
            return false;
        }
    }
    else
    {
        yCWarning(CBB) << "Invalid SDO direction" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(dir));
        return false;
    }
}

// -----------------------------------------------------------------------------

bool SdoReplier::notifyMessage(const can_message & msg)
{
    if (priv->sdo() && priv->sdo()->getCobIdTx() == msg.id)
    {
        return priv->sdo()->notify(msg.data);
    }

    return false;
}

// -----------------------------------------------------------------------------
