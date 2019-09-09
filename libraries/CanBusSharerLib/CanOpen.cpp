// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanOpen.hpp"

using namespace roboticslab;

CanOpen::CanOpen(unsigned int _id, CanSenderDelegate * _sender)
    : id(_id),
      sender(_sender),
      _sdo(new SdoClient(id, 0x600, 0x580, 0.1)), // TODO: timeout
      _rpdo1(new ReceivePdo(id, 0x200, 1, _sdo)),
      _rpdo2(new ReceivePdo(id, 0x300, 2, _sdo)),
      _rpdo3(new ReceivePdo(id, 0x400, 3, _sdo)),
      _rpdo4(new ReceivePdo(id, 0x500, 4, _sdo)),
      _tpdo1(new TransmitPdo(id, 0x180, 1, _sdo)),
      _tpdo2(new TransmitPdo(id, 0x280, 2, _sdo)),
      _tpdo3(new TransmitPdo(id, 0x380, 3, _sdo)),
      _tpdo4(new TransmitPdo(id, 0x480, 4, _sdo)),
      _emcy(new EmcyConsumer(_sdo))
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
}

bool CanOpen::consumeMessage(std::uint16_t cobId, const std::uint8_t * data, std::size_t size)
{
    std::uint16_t op = cobId - this->id;

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
    default:
        return false;
    }
}
