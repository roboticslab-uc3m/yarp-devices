// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_HPP__
#define __SDO_CLIENT_HPP__

#include <cstdint>

#include <string>
#include <type_traits>
#include <utility>

#include "CanSenderDelegate.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Representation of SDO client protocol.
 *
 * Provides access to SDO reads and writes in a user-friendly manner, sparing
 * clients the need of building char arrays and such. Uses CAN terminology
 * ("upload" means reads from the drive, "downloads" means writes to the drive).
 * Supports normal (segmented) and expedited transfers. Obtains data size from
 * the upload/download data type and takes care of managing the handshake.
 * SDO transfers block with timeout and always wait for the response or confirm
 * message from the drive, signalizing failures accordingly. Also supports SDO
 * abort protocol.
 */
class SdoClient final
{
public:
    //! Constructor, registers CAN sender handle.
    SdoClient(std::uint8_t id, std::uint16_t cobRx, std::uint16_t cobTx, double timeout, CanSenderDelegate * sender = nullptr)
        : id(id), cobRx(cobRx), cobTx(cobTx), sender(sender), stateObserver(timeout)
    {}

    //! Retrieve COB ID of SDO packages received by the drive.
    std::uint16_t getCobIdRx() const
    { return cobRx + id; }

    //! Retrieve COB ID of SDO packages sent by the drive.
    std::uint16_t getCobIdTx() const
    { return cobTx + id; }

    //! Configure CAN sender delegate handle.
    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    //! Notify observers on an SDO package sent by the drive.
    bool notify(const std::uint8_t * raw)
    { return stateObserver.notify(raw, 8); }

    /**
     * @brief Request an SDO package from the drive, only integral types.
     * @tparam T Integral data type.
     * @param name Description of the CAN dictionary object.
     * @param data Pointer to an external storage, will be populated with
     * received CAN data.
     * @param index Index of targeted CAN dictionary object.
     * @param index Subindex of targeted CAN dictionary object.
     * @return True on success, false on timeout.
     */
    template<typename T>
    bool upload(const std::string & name, T * data, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return uploadInternal(name, data, sizeof(T), index, subindex);
    }

    /**
     * @brief Request an SDO package from the drive with callback, only integral types.
     * @tparam T Integral data type.
     * @tparam Fn Function object type.
     * @param name Description of the CAN dictionary object.
     * @param fn Callback function, will be invoked with the received CAN data
     * as input parameter.
     * @param index Index of targeted CAN dictionary object.
     * @param index Subindex of targeted CAN dictionary object.
     * @return True on success, false on timeout.
     */
    template<typename T, typename Fn>
    bool upload(const std::string & name, Fn && fn, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        T data;
        return upload(name, &data, index, subindex) && (std::forward<Fn>(fn)(data), true);
    }

    /**
     * @brief Send an SDO package to the drive, only integral types.
     * @tparam T Integral data type.
     * @param name Description of the CAN dictionary object.
     * @param data Value to be sent.
     * @param index Index of targeted CAN dictionary object.
     * @param index Subindex of targeted CAN dictionary object.
     * @return True on success, false on timeout.
     */
    template<typename T>
    bool download(const std::string & name, T data, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        return downloadInternal(name, &data, sizeof(T), index, subindex);
    }

    /**
     * @brief Request an SDO package from the drive, only string type.
     * @param name Description of the CAN dictionary object.
     * @param s Output string value.
     * @param index Index of targeted CAN dictionary object.
     * @param index Subindex of targeted CAN dictionary object.
     * @return True on success, false on timeout.
     */
    bool upload(const std::string & name, std::string & s, std::uint16_t index, std::uint8_t subindex = 0x00);

    /**
     * @brief Request an SDO package from the drive with callback, only string type.
     * @tparam Fn Function object type.
     * @param name Description of the CAN dictionary object.
     * @param fn Callback function, will be invoked with the received CAN data
     * as input parameter.
     * @param index Index of targeted CAN dictionary object.
     * @param index Subindex of targeted CAN dictionary object.
     * @return True on success, false on timeout.
     */
    template<typename Fn>
    bool upload(const std::string & name, Fn && fn, std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        std::string s;
        return upload(name, s, index, subindex) && (std::forward<Fn>(fn)(s), true);
    }

    /**
     * @brief Send an SDO package to the drive, only string type.
     * @param name Description of the CAN dictionary object.
     * @param s String to be sent.
     * @param index Index of targeted CAN dictionary object.
     * @param index Subindex of targeted CAN dictionary object.
     * @return True on success, false on timeout.
     */
    bool download(const std::string & name, const std::string & s, std::uint16_t index, std::uint8_t subindex = 0x00);

    //! String literal overload, overrides templated variant.
    bool download(const std::string & name, const char * s, std::uint16_t index, std::uint8_t subindex = 0x00)
    { return download(name, std::string(s), index, subindex); }

private:
    bool send(const std::uint8_t * msg);
    std::string msgToStr(std::uint16_t cob, const std::uint8_t * msgData);

    bool uploadInternal(const std::string & name, void * data, std::uint32_t size, std::uint16_t index, std::uint8_t subindex);
    bool downloadInternal(const std::string & name, const void * data, std::uint32_t size, std::uint16_t index, std::uint8_t subindex);
    bool performTransfer(const std::string & name, const std::uint8_t * req, std::uint8_t * resp);

    std::uint8_t id;
    std::uint16_t cobRx;
    std::uint16_t cobTx;

    CanSenderDelegate * sender;
    TypedStateObserver<std::uint8_t[]> stateObserver;
};

} // namespace roboticslab

#endif // __SDO_CLIENT_HPP__
