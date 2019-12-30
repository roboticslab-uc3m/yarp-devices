// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_OBSERVER_HPP__
#define __STATE_OBSERVER_HPP__

#include <cstdint>
#include <cstdlib>

#include <type_traits>

namespace roboticslab
{

class StateObserverBase
{
public:
    StateObserverBase(double timeout);
    virtual ~StateObserverBase() = 0;

    StateObserverBase(const StateObserverBase &) = delete;
    StateObserverBase & operator=(const StateObserverBase &) = delete;

    double getTimeout() const
    { return timeout; }

    bool await(void * raw = nullptr);
    bool notify(const void * raw = nullptr, std::size_t len = 0);

protected:
    void * getRemoteStorage();
    const void * getRemoteStorage() const;
    virtual void setRemoteStorage(const void * ptr, std::size_t len);

private:
    double timeout;

    class Private;
    Private * impl;
};

class StateObserver final : private StateObserverBase
{
public:
    StateObserver(double timeout) : StateObserverBase(timeout)
    { }

    bool await()
    { return StateObserverBase::await(); }

    bool notify()
    { return StateObserverBase::notify(); }
};

template<typename T, typename NonFundamentalType = void>
class TypedStateObserver final : private StateObserverBase
{
public:
    TypedStateObserver(double timeout) : StateObserverBase(timeout)
    { }

    bool await(T & remote)
    { return StateObserverBase::await(&remote); }

    bool notify(const T & remote)
    { return StateObserverBase::notify(&remote); }

protected:
    virtual void setRemoteStorage(const void * remote, std::size_t len) override
    { *reinterpret_cast<T *>(getRemoteStorage()) = *reinterpret_cast<const T *>(remote); }
};

template<typename T>
class TypedStateObserver<T, typename std::enable_if<std::is_arithmetic<T>::value>::type> final : private StateObserverBase
{
public:
    TypedStateObserver(double timeout) : StateObserverBase(timeout)
    { }

    bool await(T * raw)
    { return StateObserverBase::await(raw); }

    bool notify(T raw)
    { return StateObserverBase::notify(&raw, sizeof(T)); }

    bool notify(const std::uint8_t * raw, std::size_t len)
    { return len == sizeof(T) && StateObserverBase::notify(raw, len); }
};

template<>
class TypedStateObserver<std::uint8_t[]> final : private StateObserverBase
{
public:
    TypedStateObserver(double timeout) : StateObserverBase(timeout)
    { }

    bool await(std::uint8_t * raw)
    { return StateObserverBase::await(raw); }

    bool notify(const std::uint8_t * raw, std::size_t len)
    { return StateObserverBase::notify(raw, len); }
};

} // namespace roboticslab

#endif // __STATE_OBSERVER_HPP__
