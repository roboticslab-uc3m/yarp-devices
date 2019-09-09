// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PDO_PROTOCOL_HPP__
#define __PDO_PROTOCOL_HPP__

#include <cstdint>

#include <functional>
#include <type_traits>
#include <utility> // std::forward

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"

namespace roboticslab
{

class PdoProtocol;

class PdoConfiguration final
{
    friend PdoProtocol;

public:
    PdoConfiguration();
    ~PdoConfiguration();
    PdoConfiguration(const PdoConfiguration &);
    PdoConfiguration & operator=(const PdoConfiguration &);

    PdoConfiguration & setValid(bool value);
    PdoConfiguration & setRtr(bool value);
    PdoConfiguration & setTransmissionType(std::uint8_t value);
    PdoConfiguration & setInhibitTime(std::uint16_t value);
    PdoConfiguration & setEventTimer(std::uint16_t value);
    PdoConfiguration & setSyncStartValue(std::uint8_t value);

    template<typename T>
    PdoConfiguration & addMapping(std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        static_assert(sizeof(T) <= sizeof(std::uint32_t), "Size exceeds 4 bytes.");
        addMappingInternal((index << 16) + (subindex << 8) + sizeof(T) * 8);
        return *this;
    }

private:
    void addMappingInternal(std::uint32_t value);

    class Private;
    Private * priv;
};

class PdoProtocol
{
public:
    PdoProtocol(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo)
        : id(id), cob(cob), n(n), sdo(sdo), sender(nullptr)
    { }

    virtual ~PdoProtocol()
    { }

    std::uint16_t getCobId() const
    { return cob + id; };

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    virtual bool configure(const PdoConfiguration & config);

protected:
    enum class PdoType { RPDO, TPDO };

    virtual PdoType getType() const = 0;

    // https://stackoverflow.com/a/38776200
    template<typename T1 = std::uint8_t, typename... Tn>
    static constexpr std::size_t size()
    { return sizeof...(Tn) == 0 ? sizeof(T1) : sizeof(T1) + size<Tn...>(); }

    std::uint8_t id;
    std::uint16_t cob;
    unsigned int n;

    SdoClient * sdo;
    CanSenderDelegate * sender;
};

class ReceivePdo final : public PdoProtocol
{
public:
    ReceivePdo(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo)
        : PdoProtocol(id, cob, n, sdo)
    { }

    template<typename... Ts>
    bool write(Ts... data)
    {
        // TODO: check against PDO mapping configuration?
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        std::uint8_t raw[size<Ts...>()]; std::size_t count = 0;
        ordered_call{(pack(&data, raw, &count), 1)...}; // https://w.wiki/7M$
        return writeInternal(raw, count);
    }

protected:
    virtual PdoType getType() const override
    { return PdoType::RPDO; }

private:
    struct ordered_call
    { template<typename... Ts> ordered_call(Ts...) { } };

    template<typename T>
    void pack(const T * data, std::uint8_t * buff, std::size_t * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        packInternal(buff + *count, data, sizeof(T));
        *count += sizeof(T);
    }

    void packInternal(std::uint8_t * buff, const void * data, std::size_t size);
    bool writeInternal(const std::uint8_t * data, std::size_t size);
};

class TransmitPdo final : public PdoProtocol
{
public:
    TransmitPdo(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo)
        : PdoProtocol(id, cob, n, sdo)
    { }

    bool accept(const std::uint8_t * data, std::size_t size)
    { return callback(data, size); }

    template<typename... Ts, typename Fn>
    void registerHandler(const Fn & fn)
    {
        // TODO: check against PDO mapping configuration?
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        callback = [&](const std::uint8_t * raw, std::size_t len)
            { std::size_t count = 0;
              return size<Ts...>() == len && (ordered_call{fn, unpack<Ts>(raw, &count)...}, true); };
    }

protected:
    virtual PdoType getType() const override
    { return PdoType::TPDO; }

private:
    typedef std::function<bool(const std::uint8_t * data, std::size_t size)> HandlerFn;

    // https://stackoverflow.com/a/14058638
    struct ordered_call
    {
        template<typename Fn, typename... Ts>
        ordered_call(const Fn & fn, Ts &&... ts)
        { fn(std::forward<Ts>(ts)...); }
    };

    template<typename T>
    T unpack(const std::uint8_t * buff, std::size_t * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        T data;
        unpackInternal(&data, buff + *count, sizeof(T));
        *count += sizeof(T);
        return data;
    }

    void unpackInternal(void * data, const std::uint8_t * buff, std::size_t size);

    HandlerFn callback;
};

} // namespace roboticslab

#endif // __PDO_PROTOCOL_HPP__
