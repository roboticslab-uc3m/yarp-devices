// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PDO_PROTOCOL_HPP__
#define __PDO_PROTOCOL_HPP__

#include <cstddef>
#include <cstdint>

#include <functional>
#include <type_traits>
#include <utility> // std::forward

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Wrapped enumeration of a PDO transmission type.
 *
 * Inspired by <a href="https://stackoverflow.com/a/53284026">this SO answer</a>.
 */
class PdoTransmissionType final
{
public:
    //! Wrapped enumerators.
    enum transmission_type : std::uint8_t
    {
        SYNCHRONOUS_ACYCLIC = 0x00, ///< Synchronous acyclic
        SYNCHRONOUS_CYCLIC = 0x01, ///< Synchronous cyclic
        RTR_SYNCHRONOUS = 0xFC, ///< Synchronous RTR
        RTR_EVENT_DRIVEN = 0xFD, ///< Event-driven RTR
        EVENT_DRIVEN_MANUFACTURER = 0xFE, ///< Manufacturer-specific event-driven
        EVENT_DRIVEN_DEVICE_APP_PROFILE = 0xFF ///< Device application profile-specific event-driven
    };

    //! Default constructor.
    constexpr PdoTransmissionType() = default;

    //! Constructor, accepts initial transmission type.
    constexpr PdoTransmissionType(transmission_type type) : type(type)
    { }

    //! User-defined conversion operator.
    constexpr operator std::uint8_t() const
    { return type; }

    //! Cast input byte to an @ref PdoTransmissionType enumerator.
    static constexpr PdoTransmissionType SYNCHRONOUS_CYCLIC_N(std::uint8_t n)
    { return static_cast<transmission_type>(n); }

    //! Cast input byte to an @ref PdoTransmissionType enumerator, performs static check on range [0x01-0xF0].
    template<std::uint8_t n>
    static constexpr PdoTransmissionType SYNCHRONOUS_CYCLIC_N()
    {
        // https://stackoverflow.com/a/12109644
        static_assert(n >= 0x01 && n <= 0xF0, "Illegal argument."); // same as [1-240]
        return static_cast<transmission_type>(n);
    }

private:
    transmission_type type = SYNCHRONOUS_ACYCLIC;
};

class PdoProtocol;

/**
 * @ingroup CanBusSharerLib
 * @brief Set of SDO configuration values for a @ref PdoProtocol.
 *
 * This class manages optional values via chainable setters. Unless a setter has
 * been called by client code, no configuration will be applied for its related
 * value. For instance: if @ref setInhibitTime is never called, @ref PdoProtocol
 * will not attempt to create and send a SDO package that configures the inhibit
 * time of the RPDO on the drive side.
 */
class PdoConfiguration final
{
    friend PdoProtocol;

public:
    //! Constructor.
    PdoConfiguration();

    //! Destructor
    ~PdoConfiguration();

    //! Copy constructor.
    PdoConfiguration(const PdoConfiguration &);

    //! Copy assignment operator.
    PdoConfiguration & operator=(const PdoConfiguration &);

    //! Set or reset valid bit.
    PdoConfiguration & setValid(bool value);

    //! Set or reset RTR bit.
    PdoConfiguration & setRtr(bool value);

    //! Set transmission type.
    PdoConfiguration & setTransmissionType(PdoTransmissionType value);

    //! Set inhibit time.
    PdoConfiguration & setInhibitTime(std::uint16_t value);

    //! Set event timer.
    PdoConfiguration & setEventTimer(std::uint16_t value);

    //! Set sync start value.
    PdoConfiguration & setSyncStartValue(std::uint8_t value);

    //! Configure PDO mapping, uses template parameter to deduce object size.
    template<typename T>
    PdoConfiguration & addMapping(std::uint16_t index, std::uint8_t subindex = 0x00)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        static_assert(sizeof(T) <= sizeof(std::uint32_t), "Size exceeds 4 bytes.");
        addMappingInternal((index << 16) + (subindex << 8) + sizeof(T) * 8);
        return *this;
    }

private:
    void addMappingInternal(std::uint32_t value);

    class Private;
    Private * priv;
};

/**
 * @ingroup CanBusSharerLib
 * @brief Abstract representation of PDO protocol.
 *
 * See @ref PdoConfiguration regarding how PDO configuration works.
 */
class PdoProtocol
{
public:
    //! Constructor, registers SDO client handle.
    PdoProtocol(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo)
        : id(id), cob(cob), n(n), sdo(sdo)
    { }

    //! Virtual destructor.
    virtual ~PdoProtocol() = default;

    //! Retrieve COB ID.
    std::uint16_t getCobId() const
    { return cob + id; };

    //! Configure this PDO drive-side via SDO packages.
    virtual bool configure(const PdoConfiguration & config);

protected:
    //! PDO type.
    enum class PdoType { RPDO, TPDO };

    //! Retrieve PDO type.
    virtual PdoType getType() const = 0;

    // https://stackoverflow.com/a/38776200
    template<typename T1 = std::uint8_t, typename... Tn>
    static constexpr std::size_t size()
    { return sizeof...(Tn) == 0 ? sizeof(T1) : sizeof(T1) + size<Tn...>(); }

    std::uint8_t id;
    std::uint16_t cob;
    unsigned int n;

    SdoClient * sdo;
};

/**
 * @ingroup CanBusSharerLib
 * @brief Representation of RPDO protocol.
 */
class ReceivePdo final : public PdoProtocol
{
public:
    //! Constructor, registers SDO and CAN sender handles.
    ReceivePdo(std::uint8_t id, std::uint16_t cob, unsigned int n, SdoClient * sdo, CanSenderDelegate * sender = nullptr)
        : PdoProtocol(id, cob, n, sdo), sender(sender)
    { }

    //! Configure CAN sender delegate handle.
    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    /**
     * @brief Send data to the drive.
     *
     * Usually, you'll want to pass as many parameters as CAN dictionary objects
     * have been mapped. Only integral types allowed, cumulative size cannot
     * exceed 8 bytes.
     */
    template<typename... Ts>
    bool write(Ts... data)
    {
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        std::uint8_t raw[size<Ts...>()]; unsigned int count = 0;
        ordered_call{(pack(&data, raw, &count), true)...}; // https://w.wiki/7M$
        return writeInternal(raw, count);
    }

protected:
    virtual PdoType getType() const override
    { return PdoType::RPDO; }

private:
    struct ordered_call
    { template<typename... Ts> ordered_call(Ts...) { } };

    template<typename T>
    void pack(const T * data, std::uint8_t * buff, unsigned int * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        packInternal(buff + *count, data, sizeof(T));
        *count += sizeof(T);
    }

    void packInternal(std::uint8_t * buff, const void * data, unsigned int size);
    bool writeInternal(const std::uint8_t * data, unsigned int size);

    CanSenderDelegate * sender;
};

/**
 * @ingroup CanBusSharerLib
 * @brief Representation of TPDO protocol.
 */
class TransmitPdo final : public PdoProtocol
{
public:
    using PdoProtocol::PdoProtocol; // inherit parent constructor

    //! Invoke registered callback on raw CAN message data.
    bool accept(const std::uint8_t * data, unsigned int size)
    { return (bool)callback && callback(data, size); }

    /**
     * @brief Register callback.
     *
     * Usually, you'll want to pass as many parameters to the callback function
     * as CAN dictionary objects have been mapped. Only integral types allowed,
     * cumulative size cannot exceed 8 bytes.
     */
    template<typename... Ts, typename Fn>
    void registerHandler(Fn && fn)
    {
        static_assert(sizeof...(Ts) > 0 && size<Ts...>() <= 8, "Illegal cumulative size.");
        callback = [this, fn](const std::uint8_t * raw, unsigned int len)
            { unsigned int count = 0;
              return size<Ts...>() == len && (ordered_call{fn, unpack<Ts>(raw, &count)...}, true); };
    }

    //! Unregister callback.
    void unregisterHandler()
    { callback = HandlerFn(); }

protected:
    virtual PdoType getType() const override
    { return PdoType::TPDO; }

private:
    typedef std::function<bool(const std::uint8_t * data, unsigned int size)> HandlerFn;

    // https://stackoverflow.com/a/14058638
    struct ordered_call
    {
        template<typename Fn, typename... Ts>
        ordered_call(Fn && fn, Ts &&... ts)
        { std::forward<Fn>(fn)(std::forward<Ts>(ts)...); }
    };

    template<typename T>
    T unpack(const std::uint8_t * buff, unsigned int * count)
    {
        static_assert(std::is_integral<T>::value, "Integral required.");
        T data;
        unpackInternal(&data, buff + *count, sizeof(T));
        *count += sizeof(T);
        return data;
    }

    void unpackInternal(void * data, const std::uint8_t * buff, unsigned int size);

    HandlerFn callback;
};

} // namespace roboticslab

#endif // __PDO_PROTOCOL_HPP__
