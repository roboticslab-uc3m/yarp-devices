// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PDO_PROTOCOL_HPP__
#define __PDO_PROTOCOL_HPP__

#include <cstddef>
#include <cstdint>

#include <functional>
#include <type_traits>
#include <utility> // std::forward

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"

namespace roboticslab
{

// https://stackoverflow.com/a/53284026
class PdoTransmissionType final
{
public:
    enum transmission_type : std::uint8_t
    {
        SYNCHRONOUS_ACYCLIC = 0x00,
        SYNCHRONOUS_CYCLIC = 0x01,
        RTR_SYNCHRONOUS = 0xFC,
        RTR_EVENT_DRIVEN = 0xFD,
        EVENT_DRIVEN_MANUFACTURER = 0xFE,
        EVENT_DRIVEN_DEVICE_APP_PROFILE = 0xFF
    };

    constexpr PdoTransmissionType() : type(SYNCHRONOUS_ACYCLIC)
    { }

    constexpr PdoTransmissionType(transmission_type type) : type(type)
    { }

    constexpr operator std::uint8_t()
    { return type; }

    static constexpr PdoTransmissionType SYNCHRONOUS_CYCLIC_N(std::uint8_t n)
    { return static_cast<transmission_type>(n); }

private:
    transmission_type type;
};

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
    PdoConfiguration & setTransmissionType(PdoTransmissionType value);
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
        : id(id), cob(cob), n(n), sdo(sdo)
    { }

    virtual ~PdoProtocol()
    { }

    std::uint16_t getCobId() const
    { return cob + id; };

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
};

class ReceivePdo final : public PdoProtocol
{
public:
    ReceivePdo(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo, CanSenderDelegate * sender = nullptr)
        : PdoProtocol(id, cob, n, sdo), sender(sender)
    { }

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    template<typename... Ts>
    bool write(Ts... data)
    {
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        std::uint8_t raw[size<Ts...>()]; unsigned int count = 0;
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
    void pack(const T * data, std::uint8_t * buff, unsigned int * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        packInternal(buff + *count, data, sizeof(T));
        *count += sizeof(T);
    }

    void packInternal(std::uint8_t * buff, const void * data, unsigned int size);
    bool writeInternal(const std::uint8_t * data, unsigned int size);

    CanSenderDelegate * sender;
};

class TransmitPdo final : public PdoProtocol
{
public:
    TransmitPdo(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo)
        : PdoProtocol(id, cob, n, sdo)
    { }

    bool accept(const std::uint8_t * data, unsigned int size)
    { return (bool)callback && callback(data, size); }

    template<typename... Ts, typename Fn>
    void registerHandler(Fn && fn)
    {
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        callback = [&](const std::uint8_t * raw, unsigned int len)
            { unsigned int count = 0;
              return size<Ts...>() == len && (ordered_call{std::forward<Fn>(fn), unpack<Ts>(raw, &count)...}, true); };
    }

    void unregisterHandler()
    { callback = HandlerFn(); }

protected:
    virtual PdoType getType() const override
    { return PdoType::TPDO; }

private:
    typedef std::function<bool(const std::uint8_t * data, unsigned int size)> HandlerFn;

    // https://stackoverflow.com/a/14058638
    struct ordered_call
    {
        template<typename Fn, typename... Ts>
        ordered_call(Fn && fn, Ts &&... ts)
        { std::forward<Fn>(fn)(std::forward<Ts>(ts)...); }
    };

    template<typename T>
    T unpack(const std::uint8_t * buff, unsigned int * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        T data;
        unpackInternal(&data, buff + *count, sizeof(T));
        *count += sizeof(T);
        return data;
    }

    void unpackInternal(void * data, const std::uint8_t * buff, unsigned int size);

    HandlerFn callback;
};

} // namespace roboticslab

#endif // __PDO_PROTOCOL_HPP__
