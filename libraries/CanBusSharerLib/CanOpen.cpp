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
      _emcy(new EmcyConsumer)
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
