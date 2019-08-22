// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PDO_PROTOCOL_HPP__
#define __PDO_PROTOCOL_HPP__

#include <cstdint>
#include <cstring>

#include <stdexcept>
#include <type_traits>

#include "CanSenderDelegate.hpp"

namespace roboticslab
{

class PdoProtocol
{
public:
    virtual ~PdoProtocol()
    { }

    virtual void configureSender(CanSenderDelegate * sender) = 0;
};

class ReceivePdo : public PdoProtocol
{
public:
    template<typename T, typename... Ts>
    bool write(Ts... data)
    {
        static_assert(sizeof...(Ts) > 0, "Empty list of parameters.");
        std::uint8_t buffer[accumulateSize(data...)]; std::size_t count = 0;
        pass{(addData(buffer, &count, data), 1)...}; // https://w.wiki/7M$
        return count <= 8 && writeInternal(buffer, count);
    }

protected:
    virtual bool writeInternal(const std::uint8_t * data, std::size_t size) = 0;

private:
    void write() { }

    static std::size_t accumulateSizes()
    { return 0; }

    // pity this won't work: https://stackoverflow.com/a/8626450
    template<typename T1, typename... Tn>
    static constexpr std::size_t accumulateSize(T1 t1, Tn... tn)
    { return sizeof(T1) + accumulateSizes(tn...); }

    struct pass
    { template<typename... T> pass(T...) { } };

    template<typename T>
    void addData(std::uint8_t * buff, std::size_t * count, T data)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        std::memcpy(buff + *count, &data, sizeof(T));
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
    bool read();
};

}  // namespace roboticslab

#endif // __PDO_PROTOCOL_HPP__
