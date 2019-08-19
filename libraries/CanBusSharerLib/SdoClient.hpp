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
    bool upload(const std::string & name, T * data, uint16_t index, uint8_t subindex = 0x00)
    { return expeditedUpload(name, data, sizeof(T), index, subindex); }

    template<typename T>
    bool download(const std::string & name, T data, uint16_t index, uint8_t subindex = 0x00)
    { return expeditedDownload(name, &data, sizeof(T), index, subindex); }

private:
    bool send(const uint8_t * msg, int len);
    bool expeditedUpload(const std::string & name, void * data, size_t size, uint16_t index, uint8_t subindex);
    bool expeditedDownload(const std::string & name, const void * data, size_t size, uint16_t index, uint8_t subindex);
    bool performTransfer(const std::string & name, const uint8_t * req, size_t reqSize, void * resp = 0, size_t respSize = 0);

    unsigned int id;
    CanSenderDelegate * sender;
    SdoSemaphore * sdoSemaphore;

    static const uint16_t COB_D = 0x600;
    static const uint16_t COB_U = 0x580;
};

} // namespace roboticslab

#endif // __SDO_CLIENT_HPP__
