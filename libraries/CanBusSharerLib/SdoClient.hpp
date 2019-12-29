// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_HPP__
#define __SDO_CLIENT_HPP__

#include <cstdint>

#include <string>
#include <type_traits>
#include <utility>

#include "CanSenderDelegate.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

class SdoClient final
{
public:
    SdoClient(std::uint8_t id, std::uint16_t cobRx, std::uint16_t cobTx, double timeout, CanSenderDelegate * sender = nullptr)
        : id(id), cobRx(cobRx), cobTx(cobTx), sender(sender), stateObserver(timeout)
    {}

    std::uint16_t getCobIdRx() const
    { return cobRx + id; }

    std::uint16_t getCobIdTx() const
    { return cobTx + id; }

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    bool notify(const std::uint8_t * raw)
    { return stateObserver.notify(raw, 8); }

    template<typename T>
    bool upload(const std::string & name, T * data, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return uploadInternal(name, data, sizeof(T), index, subindex);
    }

    template<typename T, typename Fn>
    bool upload(const std::string & name, Fn && fn, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        T data;
        return upload(name, &data, index, subindex) && (std::forward<Fn>(fn)(data), true);
    }

    template<typename T>
    bool download(const std::string & name, T data, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return downloadInternal(name, &data, sizeof(T), index, subindex);
    }

    bool upload(const std::string & name, std::string & s, std::uint16_t index, std::uint8_t subindex = 0x00);

    template<typename Fn>
    bool upload(const std::string & name, Fn && fn, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        std::string s;
        return upload(name, s, index, subindex) && (std::forward<Fn>(fn)(s), true);
    }

    bool download(const std::string & name, const std::string & s, std::uint16_t index, std::uint8_t subindex = 0x00);

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
    TypedStateObserver<std::uint8_t[]> stateObserver;
};

} // namespace roboticslab

#endif // __SDO_CLIENT_HPP__
