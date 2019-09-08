// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_OPEN_HPP__
#define __CAN_OPEN_HPP__

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "PdoProtocol.hpp"
#include "EmcyConsumer.hpp"

namespace roboticslab
{

class CanOpen final
{
public:
    CanOpen(unsigned int id, CanSenderDelegate * sender = nullptr);

    CanOpen(const CanOpen &) = delete;
    CanOpen & operator=(const CanOpen &) = delete;

    ~CanOpen();

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    unsigned int getId() const
    { return id; }

    // TODO: return const pointer?
    SdoClient * sdo() const
    { return _sdo; }

    ReceivePdo * rpdo1() const
    { return _rpdo1; }
    ReceivePdo * rpdo2() const
    { return _rpdo2; }
    ReceivePdo * rpdo3() const
    { return _rpdo3; }
    ReceivePdo * rpdo4() const
    { return _rpdo4; }

    TransmitPdo * tpdo1() const
    { return _tpdo1; }
    TransmitPdo * tpdo2() const
    { return _tpdo2; }
    TransmitPdo * tpdo3() const
    { return _tpdo3; }
    TransmitPdo * tpdo4() const
    { return _tpdo4; }

    EmcyConsumer * emcy() const
    { return _emcy; }

private:
    unsigned int id;
    CanSenderDelegate * sender;

    SdoClient * _sdo;

    ReceivePdo * _rpdo1;
    ReceivePdo * _rpdo2;
    ReceivePdo * _rpdo3;
    ReceivePdo * _rpdo4;

    TransmitPdo * _tpdo1;
    TransmitPdo * _tpdo2;
    TransmitPdo * _tpdo3;
    TransmitPdo * _tpdo4;

    EmcyConsumer * _emcy;
};

} // namespace roboticslab

#endif // __CAN_OPEN_HPP__
