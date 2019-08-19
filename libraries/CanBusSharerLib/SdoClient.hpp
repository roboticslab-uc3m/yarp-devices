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

    void notify(const uint8_t * raw, size_t len)
    { sdoSemaphore->notify(raw, len); }

    template<typename T>
    bool upload(const std::string & name, T * data, uint16_t index, uint8_t subindex = 0x00);

    template<typename T>
    bool download(const std::string & name, T data, uint16_t index, uint8_t subindex = 0x00);

private:
    static const uint16_t COB = 0x600;

    bool send(uint8_t * msg, int len)
    { return sender->prepareMessage(message_builder(COB + id, len, msg)); }

    unsigned int id;
    CanSenderDelegate * sender;
    SdoSemaphore * sdoSemaphore;
};

} // namespace roboticslab

#include "SdoClient-inl.hpp"

#endif // __SDO_CLIENT_HPP__
