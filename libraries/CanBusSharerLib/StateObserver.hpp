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

class StateObserver : private StateObserverBase
{
public:
    StateObserver(double timeout) : StateObserverBase(timeout)
    { }

    StateObserver(const StateObserver &) = delete;
    StateObserver & operator=(const StateObserver &) = delete;

    bool await()
    { return StateObserverBase::await(); }

    bool notify()
    { return StateObserverBase::notify(); }
};

template<typename T, typename NonFundamentalType = void>
class TypedStateObserver : private StateObserverBase
{
public:
    TypedStateObserver(double timeout) : StateObserverBase(timeout)
    { }

    TypedStateObserver(const TypedStateObserver &) = delete;
    TypedStateObserver & operator=(const TypedStateObserver &) = delete;

    bool await(T * remote)
    { return StateObserverBase::await(remote); }

    bool notify(const T * remote)
    { return StateObserverBase::notify(remote); }

protected:
    virtual void setRemoteStorage(const void * remote, std::size_t len) override
    { *reinterpret_cast<T *>(getRemoteStorage()) = *reinterpret_cast<const T *>(remote); }
};

template<typename T>
class TypedStateObserver<T, std::enable_if<std::is_integral<T>::value>> : private StateObserverBase
{
public:
    TypedStateObserver(double timeout) : StateObserverBase(timeout)
    { }

    TypedStateObserver(const TypedStateObserver &) = delete;
    TypedStateObserver & operator=(const TypedStateObserver &) = delete;

    bool await(T * raw)
    { return StateObserverBase::await(raw); }

    bool notify(const T * raw, std::size_t len = sizeof(T))
    { return StateObserverBase::notify(raw, len); }
};

} // namespace roboticslab

#endif // __STATE_OBSERVER_HPP__
