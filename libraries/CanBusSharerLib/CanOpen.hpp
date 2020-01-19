// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_OPEN_HPP__
#define __CAN_OPEN_HPP__

#include <cstddef>
#include <cstdint>

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "PdoProtocol.hpp"
#include "EmcyConsumer.hpp"
#include "NmtProtocol.hpp"
#include "DriveStatusMachine.hpp"

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup CanBusSharerLib
 * @brief Collection of classes and utilities to interface with CANopen.
 */

/**
 * @ingroup CanBusSharerLib
 * @brief Wrapper for CAN protocol handles with handy accessors.
 *
 * On construction, this class initializes all handles that define CAN protocols,
 * even if clients are not going to use them all. Also, it forwards CAN messages
 * to their corresponding protocol instances given the COB-ID.
 */
class CanOpen final
{
public:
    static constexpr double SDO_TIMEOUT = 0.1;  ///< Timeout on SDO transfers (seconds)
    static constexpr double STATE_MACHINE_TIMEOUT = 2.0; ///< Timeout on state machine transitions (seconds)

    //! Constructor, creates and configures all handles.
    CanOpen(unsigned int id, double sdoTimeout = SDO_TIMEOUT,
            double stateTimeout = STATE_MACHINE_TIMEOUT, CanSenderDelegate * sender = nullptr);

    //! Deleted copy constructor.
    CanOpen(const CanOpen &) = delete;

    //! Deleted copy assingment operator.
    CanOpen & operator=(const CanOpen &) = delete;

    //! Destructor.
    ~CanOpen();

    //! Pass sender handle to internal CAN protocol handles.
    void configureSender(CanSenderDelegate * sender);

    //! Retrieve CAN node id.
    unsigned int getId() const
    { return _id; }

    //! Retrieve handle of SDO client instance.
    SdoClient * sdo() const
    { return _sdo; }

    //! Retrieve handle of RPDO1 instance.
    ReceivePdo * rpdo1() const
    { return _rpdo1; }

    //! Retrieve handle of RPDO2 instance.
    ReceivePdo * rpdo2() const
    { return _rpdo2; }

    //! Retrieve handle of RPDO3 instance.
    ReceivePdo * rpdo3() const
    { return _rpdo3; }

    //! Retrieve handle of RPDO4 instance.
    ReceivePdo * rpdo4() const
    { return _rpdo4; }

    //! Retrieve handle of TPDO1 instance.
    TransmitPdo * tpdo1() const
    { return _tpdo1; }

    //! Retrieve handle of TPDO2 instance.
    TransmitPdo * tpdo2() const
    { return _tpdo2; }

    //! Retrieve handle of TPDO3 instance.
    TransmitPdo * tpdo3() const
    { return _tpdo3; }

    //! Retrieve handle of TPDO4 instance.
    TransmitPdo * tpdo4() const
    { return _tpdo4; }

    //! Retrieve handle of EMCY instance.
    EmcyConsumer * emcy() const
    { return _emcy; }

    //! Retrieve handle of NMT instance.
    NmtProtocol * nmt() const
    { return _nmt; }

    //! Retrieve handle of drive state machine instance.
    DriveStatusMachine * driveStatus() const
    { return _driveStatus; }

    //! Process incoming CAN message and forward to the correct protocol.
    bool consumeMessage(const can_message & message) const;

private:
    unsigned int _id;

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
    NmtProtocol * _nmt;
    DriveStatusMachine * _driveStatus;
};

} // namespace roboticslab

#endif // __CAN_OPEN_HPP__
