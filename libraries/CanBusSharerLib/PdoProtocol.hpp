// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PDO_PROTOCOL_HPP__
#define __PDO_PROTOCOL_HPP__

#include <cstdint>
#include <cstring>

#include <functional>
#include <set>
#include <type_traits>
#include <utility>

#include "CanOpen.hpp"
#include "CanSenderDelegate.hpp"
#include "nonstd/optional.hpp"

namespace roboticslab
{

class CanOpen;

class PdoConfiguration final
{
    friend CanOpen;

public:
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
        mappings.insert((index << 16) + (subindex << 8) + sizeof(T) * 8);
        return *this;
    }

private:
    nonstd::optional<bool> valid;
    nonstd::optional<bool> rtr;
    nonstd::optional<std::uint8_t> transmissionType;
    nonstd::optional<std::uint16_t> inhibitTime;
    nonstd::optional<std::uint16_t> eventTimer;
    nonstd::optional<std::uint8_t> syncStartValue;
    std::set<std::uint32_t> mappings;
};

class PdoProtocol
{
public:
    virtual ~PdoProtocol()
    { }

    virtual std::uint16_t getCob() const
    { return 0; };

protected:
    // https://stackoverflow.com/a/38776200
    template<typename T1 = std::uint8_t, typename... Tn>
    static constexpr std::size_t size()
    { return sizeof...(Tn) == 0 ? sizeof(T1) : sizeof(T1) + size<Tn...>(); }
};

class ReceivePdo : public PdoProtocol
{
public:
    template<typename... Ts>
    bool write(Ts... data)
    {
        // TODO: check against PDO mapping configuration?
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        std::uint8_t raw[size<Ts...>()]; std::size_t count = 0;
        ordered_call{(pack(&data, raw, &count), 1)...}; // https://w.wiki/7M$
        return writeInternal(raw, count);
    }

    virtual void configureSender(CanSenderDelegate * sender) = 0;

protected:
    virtual bool writeInternal(const std::uint8_t * data, std::size_t size) = 0;

private:
    struct ordered_call
    { template<typename... Ts> ordered_call(Ts...) { } };

    template<typename T>
    void pack(const T * data, std::uint8_t * buff, std::size_t * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        std::memcpy(buff + *count, data, sizeof(T));
        *count += sizeof(T);
    }
};

class ConcreteReceivePdo : public ReceivePdo
{
public:
    ConcreteReceivePdo(std::uint8_t id, std::uint16_t cob) : id(id), cob(cob), sender(0)
    { }

    virtual void configureSender(CanSenderDelegate * sender) override
    { this->sender = sender; }

    virtual std::uint16_t getCob() const override
    { return cob; }

protected:
    virtual bool writeInternal(const std::uint8_t * data, std::size_t size) override;

private:
    std::uint8_t id;
    std::uint16_t cob;
    CanSenderDelegate * sender;
};

class InvalidReceivePdo : public ReceivePdo
{
public:
    virtual void configureSender(CanSenderDelegate * sender)
    { }

protected:
    virtual bool writeInternal(const std::uint8_t * data, std::size_t size);
};

class TransmitPdo : public PdoProtocol
{
public:
    typedef std::function<bool(const std::uint8_t * data, std::size_t size)> HandlerFn;

    bool accept(const std::uint8_t * data, std::size_t size)
    { return callback(data, size); }

    template<typename... Ts, typename Fn>
    void registerHandler(Fn fn)
    {
        // TODO: check against PDO mapping configuration?
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        callback = [&](const std::uint8_t * raw, std::size_t len)
            { std::size_t count = 0;
              return size<Ts...>() == len && (ordered_call{fn, unpack<Ts>(raw, &count)...}, true); };
    }

private:
    // https://stackoverflow.com/a/14058638
    struct ordered_call
    {
        template<typename Fn, typename... Ts>
        ordered_call(Fn fn, Ts &&... ts)
        { fn(std::forward<Ts>(ts)...); }
    };

    template<typename T>
    T unpack(const std::uint8_t * buff, std::size_t * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        T data;
        std::memcpy(&data, buff + *count, sizeof(T));
        *count += sizeof(T);
        return data;
    }

    HandlerFn callback;
};

class ConcreteTransmitPdo : public TransmitPdo
{
public:
    ConcreteTransmitPdo(std::uint8_t id, std::uint16_t cob) : id(id), cob(cob)
    { }

    virtual std::uint16_t getCob() const override
    { return cob; }

private:
    std::uint8_t id;
    std::uint16_t cob;
};

class InvalidTransmitPdo : public TransmitPdo
{};

}  // namespace roboticslab

#endif // __PDO_PROTOCOL_HPP__
