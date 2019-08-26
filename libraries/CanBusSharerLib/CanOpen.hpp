// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_OPEN_HPP__
#define __CAN_OPEN_HPP__

#include <cstdint>

#include <unordered_map>

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "PdoProtocol.hpp"

namespace roboticslab
{

// TODO: drop SDO/PDO overloads?
class CanOpen final
{
public:
    CanOpen(int id);
    CanOpen(int id, CanSenderDelegate * sender);

    CanOpen(const CanOpen &) = delete;
    CanOpen & operator=(const CanOpen &) = delete;

    ~CanOpen();

    void configureSender(CanSenderDelegate * sender);

    SdoClient * sdo() const;
    bool createSdo(double timeout);

    SdoClient * sdo(unsigned int n) const;
    bool createSdo(unsigned int n, std::uint16_t cobRx, std::uint16_t cobTx, double timeout);

    ReceivePdo * rpdo1() const;
    bool createRpdo1();

    ReceivePdo * rpdo2() const;
    bool createRpdo2();

    ReceivePdo * rpdo3() const;
    bool createRpdo3();

    ReceivePdo * rpdo4() const;
    bool createRpdo4();

    ReceivePdo * rpdo(unsigned int n) const;
    bool createRpdo(unsigned int n, std::uint16_t cob);

    TransmitPdo * tpdo1() const;
    bool createTpdo1();

    TransmitPdo * tpdo2() const;
    bool createTpdo2();

    TransmitPdo * tpdo3() const;
    bool createTpdo3();

    TransmitPdo * tpdo4() const;
    bool createTpdo4();

    TransmitPdo * tpdo(unsigned int n) const;
    bool createTpdo(unsigned int n, std::uint16_t cob);

private:
    int id;
    CanSenderDelegate * sender;
    bool hasSender;

    std::unordered_map<std::uint8_t, SdoClient *> sdos;
    std::unordered_map<std::uint8_t, ReceivePdo *> rpdos;
    std::unordered_map<std::uint8_t, TransmitPdo *> tpdos;
};

}  // namespace roboticslab

#endif // __CAN_OPEN_HPP__
