// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoReplier.hpp"

#include <iomanip>
#include <ios>
#include <memory>
#include <sstream>

#include <yarp/conf/version.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"
#include "SdoClient.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_MINOR >= 5
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
#else
constexpr yarp::conf::vocab32_t VOCAB_SDO_UPLOAD = yarp::os::createVocab('s', 'd', 'o', 'u');
constexpr yarp::conf::vocab32_t VOCAB_SDO_DOWNLOAD = yarp::os::createVocab('s', 'd', 'o', 'd');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I8_TYPE = yarp::os::createVocab('i', '8');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI8_TYPE = yarp::os::createVocab('u', 'i', '8');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I16_TYPE = yarp::os::createVocab('i', '1', '6');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI16_TYPE = yarp::os::createVocab('u', 'i', '1', '6');
constexpr yarp::conf::vocab32_t VOCAB_SDO_I32_TYPE = yarp::os::createVocab('i', '3', '2');
constexpr yarp::conf::vocab32_t VOCAB_SDO_UI32_TYPE = yarp::os::createVocab('u', 'i', '3', '2');
constexpr yarp::conf::vocab32_t VOCAB_SDO_STRING_TYPE = yarp::os::createVocab('s', 't', 'r');
constexpr yarp::conf::vocab32_t VOCAB_SDO_OK = yarp::os::createVocab('o', 'k');
constexpr yarp::conf::vocab32_t VOCAB_SDO_FAIL = yarp::os::createVocab('f', 'a', 'i', 'l');
#endif

namespace
{
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
            delete *sdo;
            *sdo = nullptr;

            if (response && writer)
            {
#if YARP_VERSION_MINOR >= 5
                response->addVocab32(success ? VOCAB_SDO_OK : VOCAB_SDO_FAIL);
#else
                response->addVocab(success ? VOCAB_SDO_OK : VOCAB_SDO_FAIL);
#endif
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
    SdoClient ** allocate(unsigned int id, CanSenderDelegate * sender)
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

    ConnectionGuard guard(&response, writer);

    if (request.size() < 5)
    {
        yCWarning(CBCB, "SDO requests require at least 5 elements, got %zu", request.size());
        return false;
    }

#if YARP_VERSION_MINOR >= 5
    sdo_direction dir = static_cast<sdo_direction>(request.get(0).asVocab32());
#else
    sdo_direction dir = static_cast<sdo_direction>(request.get(0).asVocab());
#endif
    unsigned int id = request.get(1).asInt8();
    unsigned int index = request.get(2).asInt16();
    unsigned int subindex = request.get(3).asInt8();
#if YARP_VERSION_MINOR >= 5
    data_type type = static_cast<data_type>(request.get(4).asVocab32());
#else
    data_type type = static_cast<data_type>(request.get(4).asVocab());
#endif

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
#if YARP_VERSION_MINOR >= 5
            yCWarning(CBCB, "Invalid data type %s", yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(type)).c_str());
#else
            yCWarning(CBCB, "Invalid data type %s", yarp::os::Vocab::decode(static_cast<yarp::conf::vocab32_t>(type)).c_str());
#endif
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
            yCWarning(CBCB, "Download SDO requires exactly 6 elements, got %zu", request.size());
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
#if YARP_VERSION_MINOR >= 5
            yCWarning(CBCB, "Invalid data type %s", yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(type)).c_str());
#else
            yCWarning(CBCB, "Invalid data type %s", yarp::os::Vocab::decode(static_cast<yarp::conf::vocab32_t>(type)).c_str());
#endif
            return false;
        }
    }
    else
    {
#if YARP_VERSION_MINOR >= 5
        yCWarning(CBCB, "Invalid SDO direction %s", yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(dir)).c_str());
#else
        yCWarning(CBCB, "Invalid SDO direction %s", yarp::os::Vocab::decode(static_cast<yarp::conf::vocab32_t>(dir)).c_str());
#endif
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
