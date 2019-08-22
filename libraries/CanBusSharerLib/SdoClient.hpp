// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_HPP__
#define __SDO_CLIENT_HPP__

#include <stdint.h>

#include <functional>
#include <string>
#include <type_traits>

#include "CanSenderDelegate.hpp"
#include "SdoSemaphore.hpp"

namespace roboticslab
{

class SdoClient
{
public:
    virtual ~SdoClient() { }

    virtual void configureSender(CanSenderDelegate * sender) = 0;

    virtual void notify(const uint8_t * raw) = 0;

    template<typename T>
    bool upload(const std::string & name, T * data, uint16_t index, uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return uploadInternal(name, data, sizeof(T), index, subindex);
    }

    template<typename T>
    bool upload(const std::string & name, std::function<void(T * data)> fn, uint16_t index, uint8_t subindex = 0x00)
    {
        T data; bool res;
        return res = upload(name, &data, index, subindex), fn(&data), res;
    }

    template<typename T>
    bool download(const std::string & name, T data, uint16_t index, uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return downloadInternal(name, &data, sizeof(T), index, subindex);
    }

protected:
    virtual bool uploadInternal(const std::string & name, void * data, uint32_t size, uint16_t index, uint8_t subindex) = 0;
    virtual bool downloadInternal(const std::string & name, const void * data, uint32_t size, uint16_t index, uint8_t subindex) = 0;
};

class ConcreteSdoClient : public SdoClient
{
public:
    ConcreteSdoClient(unsigned int id, double timeout) : id(id), sender(0), sdoSemaphore(timeout)
    {}

    virtual void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    virtual void notify(const uint8_t * raw)
    { sdoSemaphore.notify(raw); }

private:
    bool send(const uint8_t * msg);
    std::string msgToStr(uint16_t cob, const uint8_t * msgData);

    virtual bool uploadInternal(const std::string & name, void * data, uint32_t size, uint16_t index, uint8_t subindex);
    virtual bool downloadInternal(const std::string & name, const void * data, uint32_t size, uint16_t index, uint8_t subindex);
    bool performTransfer(const std::string & name, const uint8_t * req, uint8_t * resp);

    unsigned int id;
    CanSenderDelegate * sender;
    SdoSemaphore sdoSemaphore;

    static const uint16_t COB_D = 0x600;
    static const uint16_t COB_U = 0x580;
};

class InvalidSdoClient : public SdoClient
{
public:
    virtual void configureSender(CanSenderDelegate * sender)
    { }

    virtual void notify(const uint8_t * raw)
    { }

private:
    virtual bool uploadInternal(const std::string & name, void * data, uint32_t size, uint16_t index, uint8_t subindex);
    virtual bool downloadInternal(const std::string & name, const void * data, uint32_t size, uint16_t index, uint8_t subindex);
};

} // namespace roboticslab

// template specializations
#include "SdoClient-inl.hpp"

#endif // __SDO_CLIENT_HPP__
