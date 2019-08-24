// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PDO_PROTOCOL_HPP__
#define __PDO_PROTOCOL_HPP__

#include <cstdint>
#include <cstring>

#include <functional>
#include <type_traits>
#include <utility>

#include "CanSenderDelegate.hpp"

namespace roboticslab
{

class PdoProtocol
{
public:
    virtual ~PdoProtocol()
    { }

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
    ConcreteReceivePdo(unsigned int id) : id(id), sender(0)
    { }

    virtual void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

protected:
    virtual bool writeInternal(const std::uint8_t * data, std::size_t size);

private:
    unsigned int id;
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
    ConcreteTransmitPdo(unsigned int id) : id(id)
    { }

private:
    unsigned int id;
};

class InvalidTransmitPdo : public TransmitPdo
{};

}  // namespace roboticslab

#endif // __PDO_PROTOCOL_HPP__
