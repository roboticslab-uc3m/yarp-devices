// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_HPP__
#define __SDO_CLIENT_HPP__

#include <stdint.h>

#include <string>

#include "CanSenderDelegate.hpp"
#include "SdoSemaphore.hpp"

namespace roboticslab
{

class SdoClient
{
public:
    SdoClient(unsigned int id, double timeout) : id(id), sender(0), sdoSemaphore(new SdoSemaphore(timeout))
    {}

    ~SdoClient()
    { delete sdoSemaphore; }

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    void notify(const uint8_t * raw)
    { sdoSemaphore->notify(raw); }

    template<typename T>
    bool upload(const std::string & name, T * data, uint16_t index, uint8_t subindex = 0x00)
    { return uploadInternal(name, data, sizeof(T), index, subindex); }

    template<typename T>
    bool download(const std::string & name, T data, uint16_t index, uint8_t subindex = 0x00)
    { return downloadInternal(name, &data, sizeof(T), index, subindex); }

private:
    bool send(const uint8_t * msg);
    std::string msgToStr(uint16_t cob, const uint8_t * msgData);

    bool uploadInternal(const std::string & name, void * data, size_t size, uint16_t index, uint8_t subindex);
    bool downloadInternal(const std::string & name, const void * data, size_t size, uint16_t index, uint8_t subindex);
    bool performTransfer(const std::string & name, const uint8_t * req, uint8_t * resp);

    unsigned int id;
    CanSenderDelegate * sender;
    SdoSemaphore * sdoSemaphore;

    static const uint16_t COB_D = 0x600;
    static const uint16_t COB_U = 0x580;
};

} // namespace roboticslab

// template specializations
#include "SdoClient-inl.hpp"

#endif // __SDO_CLIENT_HPP__
