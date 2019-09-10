// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanOpen.hpp"

using namespace roboticslab;

CanOpen::CanOpen(unsigned int id, CanSenderDelegate * sender)
    : _id(id),
      _sdo(new SdoClient(_id, 0x600, 0x580, 0.1, sender)), // TODO: timeout
      _rpdo1(new ReceivePdo(_id, 0x200, 1, _sdo, sender)),
      _rpdo2(new ReceivePdo(_id, 0x300, 2, _sdo, sender)),
      _rpdo3(new ReceivePdo(_id, 0x400, 3, _sdo, sender)),
      _rpdo4(new ReceivePdo(_id, 0x500, 4, _sdo, sender)),
      _tpdo1(new TransmitPdo(_id, 0x180, 1, _sdo)),
      _tpdo2(new TransmitPdo(_id, 0x280, 2, _sdo)),
      _tpdo3(new TransmitPdo(_id, 0x380, 3, _sdo)),
      _tpdo4(new TransmitPdo(_id, 0x480, 4, _sdo)),
      _emcy(new EmcyConsumer(_sdo)),
      _nmt(new NmtProtocol(_id, _sdo, sender))
{
}

CanOpen::~CanOpen()
{
    delete _sdo;

    delete _rpdo1;
    delete _rpdo2;
    delete _rpdo3;
    delete _rpdo4;

    delete _tpdo1;
    delete _tpdo2;
    delete _tpdo3;
    delete _tpdo4;

    delete _emcy;
    delete _nmt;
}

void CanOpen::configureSender(CanSenderDelegate * sender)
{
    _sdo->configureSender(sender);
    _rpdo1->configureSender(sender);
    _rpdo2->configureSender(sender);
    _rpdo3->configureSender(sender);
    _rpdo4->configureSender(sender);
    _nmt->configureSender(sender);
}

bool CanOpen::consumeMessage(std::uint16_t cobId, const std::uint8_t * data, std::size_t size) const
{
    const std::uint16_t op = cobId - _id;

    switch (op)
    {
    case 0x80:
        _emcy->accept(data);
        return true;
    case 0x180:
        return _tpdo1->accept(data, size);
    case 0x280:
        return _tpdo2->accept(data, size);
    case 0x380:
        return _tpdo3->accept(data, size);
    case 0x480:
        return _tpdo4->accept(data, size);
    case 0x580:
        return _sdo->notify(data);
    case 0x700:
        return _nmt->accept(data);
    default:
        return false;
    }
}
