// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_HPP__
#define __SDO_CLIENT_HPP__

#include <cstdint>

#include <functional>
#include <string>
#include <type_traits>

#include "CanSenderDelegate.hpp"
#include "SdoSemaphore.hpp"

namespace roboticslab
{

class SdoClient final
{
public:
    SdoClient(std::uint8_t id, std::uint16_t cobRx, std::uint16_t cobTx, double timeout)
        : id(id), cobRx(cobRx), cobTx(cobTx), sender(0), sdoSemaphore(timeout)
    {}

    std::uint16_t getCobIdRx() const
    { return cobRx + id; }

    std::uint16_t getCobIdTx() const
    { return cobTx + id; }

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    void notify(const std::uint8_t * raw)
    { sdoSemaphore.notify(raw); }

    template<typename T>
    bool upload(const std::string & name, T * data, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return uploadInternal(name, data, sizeof(T), index, subindex);
    }

    template<typename T>
    bool upload(const std::string & name, std::function<void(T * data)> fn, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        T data; bool res;
        return res = upload(name, &data, index, subindex), fn(&data), res;
    }

    template<typename T>
    bool download(const std::string & name, T data, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return downloadInternal(name, &data, sizeof(T), index, subindex);
    }

private:
    bool send(const std::uint8_t * msg);
    std::string msgToStr(std::uint16_t cob, const std::uint8_t * msgData);

    bool uploadInternal(const std::string & name, void * data, std::uint32_t size, std::uint16_t index, std::uint8_t subindex);
    bool downloadInternal(const std::string & name, const void * data, std::uint32_t size, std::uint16_t index, std::uint8_t subindex);
    bool performTransfer(const std::string & name, const std::uint8_t * req, std::uint8_t * resp);

    std::uint8_t id;
    std::uint16_t cobRx;
    std::uint16_t cobTx;

    CanSenderDelegate * sender;
    SdoSemaphore sdoSemaphore;
};

} // namespace roboticslab

// template specializations
#include "SdoClient-inl.hpp"

#endif // __SDO_CLIENT_HPP__
