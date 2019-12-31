// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_OBSERVER_HPP__
#define __STATE_OBSERVER_HPP__

#include <cstdint>
#include <cstdlib>

#include <type_traits>

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Base class for a state observer.
 *
 * This monitor class provides a synchronized timeout mechanism for clients to
 * wait for a specific event to happen. A call to @ref await blocks the caller
 * until @ref notify is invoked by another thread or the timeout has elapsed.
 */
class StateObserverBase
{
public:
    //! Constructor, configure with timeout in seconds.
    StateObserverBase(double timeout);

    //! Virtual destructor.
    virtual ~StateObserverBase() = 0;

    //! Deleted copy constructor.
    StateObserverBase(const StateObserverBase &) = delete;

    //! Deleted copy assignment operator.
    StateObserverBase & operator=(const StateObserverBase &) = delete;

    //! Retrieve configured timeout (in seconds).
    double getTimeout() const
    { return timeout; }

    //! Causes the current thread to wait until @ref notify is invoked or the timeout elapses.
    bool await(void * raw = nullptr);

    //! Wake up a thread that waits on this object's monitor.
    bool notify(const void * raw = nullptr, std::size_t len = 0);

protected:
    //! Retrieve modifiable handle to notified data storage.
    void * getRemoteStorage();

    //! Retrieve non-modifiable handle to notified data storage.
    const void * getRemoteStorage() const;

    //! Set data storage to notify observers with.
    virtual void setRemoteStorage(const void * ptr, std::size_t len);

private:
    double timeout;

    class Private;
    Private * impl;
};

/**
 * @ingroup CanBusSharerLib
 * @brief Data-free state observer.
 */
class StateObserver final : private StateObserverBase
{
public:
    using StateObserverBase::StateObserverBase;

    //! Wait with timeout until another thread invokes @ref notify.
    bool await()
    { return StateObserverBase::await(); }

    //! Wakes up a waiting thread.
    bool notify()
    { return StateObserverBase::notify(); }
};

/**
 * @ingroup CanBusSharerLib
 * @brief Type state observer for non-arithmetic types.
 * @tparam T Non-fundamental type, mainly user-defined classes.
 */
template<typename T, typename NonFundamentalType = void>
class TypedStateObserver final : private StateObserverBase
{
public:
    using StateObserverBase::StateObserverBase;

    //! Wait with timeout until another thread invokes @ref notify.
    bool await(T & remote)
    { return StateObserverBase::await(&remote); }

    //! Wakes up a waiting thread.
    bool notify(const T & remote)
    { return StateObserverBase::notify(&remote); }

protected:
    virtual void setRemoteStorage(const void * remote, std::size_t len) override
    { *reinterpret_cast<T *>(getRemoteStorage()) = *reinterpret_cast<const T *>(remote); }
};

/**
 * @ingroup CanBusSharerLib
 * @brief Type state observer for arithmetic types.
 * @tparam T Arithmetic type, see <a href="https://en.cppreference.com/w/c/language/arithmetic_types">reference</a>.
 */
template<typename T>
class TypedStateObserver<T, typename std::enable_if<std::is_arithmetic<T>::value>::type> final : private StateObserverBase
{
public:
    using StateObserverBase::StateObserverBase;

    //! Wait with timeout until another thread invokes @ref notify.
    bool await(T * raw)
    { return StateObserverBase::await(raw); }

    //! Wakes up a waiting thread.
    bool notify(T raw)
    { return StateObserverBase::notify(&raw, sizeof(T)); }

    //! Wakes up a waiting thread with byte array.
    bool notify(const std::uint8_t * raw, std::size_t len)
    { return len == sizeof(T) && StateObserverBase::notify(raw, len); }
};

/**
 * @ingroup CanBusSharerLib
 * @brief Type state observer for unsigned char arrays.
 */
template<>
class TypedStateObserver<std::uint8_t[]> final : private StateObserverBase
{
public:
    using StateObserverBase::StateObserverBase;

    //! Wait with timeout until another thread invokes @ref notify.
    bool await(std::uint8_t * raw)
    { return StateObserverBase::await(raw); }

    //! Wakes up a waiting thread.
    bool notify(const std::uint8_t * raw, std::size_t len)
    { return StateObserverBase::notify(raw, len); }
};

} // namespace roboticslab

#endif // __STATE_OBSERVER_HPP__
