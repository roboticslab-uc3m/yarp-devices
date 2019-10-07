#include "gtest/gtest.h"

#include <cstdint>
#include <cstring>

#include <chrono>
#include <functional>
#include <future>

#include "StateObserver.hpp"
#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "CanUtils.hpp"

namespace roboticslab
{

/**
 * @ingroup yarp_devices_tests
 * @brief ...
 */
class CanBusSharerTest : public testing::Test
{
public:

    virtual void SetUp()
    {
        senderDelegate = new FakeCanSenderDelegate;
    }

    virtual void TearDown()
    {
        if (f.valid())
        {
            f.wait();
        }

        delete senderDelegate;
    }

protected:

    CanSenderDelegate * getSenderDelegate()
    { return senderDelegate; }

    static constexpr double TIMEOUT = 0.1; // [s]
    static constexpr int MILLIS = 10;

    std::future<void> f;

private:

    class FakeCanSenderDelegate : public CanSenderDelegate
    {
    public:
        virtual bool prepareMessage(const can_message & msg) override
        { return true; }
    };

    FakeCanSenderDelegate * senderDelegate;
};

class observer_timer
{
public:
    observer_timer(int _milliseconds, const std::function<bool()> & _fn)
        : milliseconds(_milliseconds), fn(_fn)
    { }

    void operator()()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
        ASSERT_TRUE(fn());
    }

private:
    int milliseconds;
    std::function<bool()> fn;
};

TEST_F(CanBusSharerTest, CanUtils)
{
    // http://www.hugi.scene.org/online/coding/hugi%2015%20-%20cmtadfix.htm
    // https://planetcalc.com/862/

    // test CanUtils::encodeFixedPoint(double, std::int8_t *, std::uint8_t *)

    double v1 = 3.14;
    std::int8_t int1;
    std::uint8_t frac1;
    CanUtils::encodeFixedPoint(v1, &int1, &frac1); // 11.00100100
    ASSERT_EQ(int1, 3);
    ASSERT_EQ(frac1, 36);

    // test CanUtils::encodeFixedPoint(double, std::int32_t *, std::uint16_t *)

    double v2 = 4444.4444;
    std::int32_t int2;
    std::uint16_t frac2;
    CanUtils::encodeFixedPoint(v2, &int2, &frac2); // 1000101011100.0111000111000100
    ASSERT_EQ(int2, 4444);
    ASSERT_EQ(frac2, 29124);

    // test CanUtils::decodeFixedPoint(std::int8_t, std::uint8_t)

    std::int8_t int3 = 0;
    std::uint8_t frac3 = 204;
    double v3 = CanUtils::decodeFixedPoint(int3, frac3); // 0.11001100
    ASSERT_EQ(v3, 0.796875);

    // test CanUtils::decodeFixedPoint(std::int16_t, std::uint16_t)

    std::int16_t int4 = -4444;
    std::uint16_t frac4 = 4444;
    double v4 = CanUtils::decodeFixedPoint(int4, frac4);
    ASSERT_NEAR(v4, -4444.06781, 1e-6);
}

TEST_F(CanBusSharerTest, StateObserver)
{
    // test StateObserver

    StateObserver emptyStateObserver(TIMEOUT);
    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return emptyStateObserver.notify(); }});
    ASSERT_TRUE(emptyStateObserver.await());

    // test TypedStateObserver<dummy> with dummy = user-defined type

    struct dummy
    { int val; };

    dummy d;
    const dummy _v1{4};
    TypedStateObserver<dummy> dummyStateObserver(TIMEOUT);
    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return dummyStateObserver.notify(_v1); }});
    ASSERT_TRUE(dummyStateObserver.await(d));
    ASSERT_EQ(d.val, _v1.val);

    // test TypedStateObserver<int>, use notify(int)

    int val1;
    const int _v2 = 4;
    TypedStateObserver<int> intStateObserver(TIMEOUT);
    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return intStateObserver.notify(_v2); }});
    ASSERT_TRUE(intStateObserver.await(&val1));
    ASSERT_EQ(val1, _v2);

    // test TypedStateObserver<int>, use notify(const std::uint8_t *, std::size_t)

    std::int8_t val2;
    const std::uint8_t _v3[] = {4};
    TypedStateObserver<std::int8_t> intBuffStateObserver(TIMEOUT);
    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return intBuffStateObserver.notify(_v3, 1); }});
    ASSERT_TRUE(intBuffStateObserver.await(&val2));
    ASSERT_EQ(val2, _v3[0]);

    // test TypedStateObserver<std::uint8_t[]>

    std::uint8_t buff[1];
    const std::uint8_t _v4[] = {4};
    TypedStateObserver<std::uint8_t[]> bufferStateObserver(TIMEOUT);
    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return bufferStateObserver.notify(_v4, 1); }});
    ASSERT_TRUE(bufferStateObserver.await(buff));
    ASSERT_EQ(buff[0], _v4[0]);

    // test StateObserver, never call notify()

    StateObserver orphanStateObserver(TIMEOUT);
    ASSERT_FALSE(orphanStateObserver.await());

    // test StateObserver on existing instance, await() but don't notify()

    ASSERT_FALSE(emptyStateObserver.await());

    // test StateObserver on existing instance, await() and notify()

    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return emptyStateObserver.notify(); }});
    ASSERT_TRUE(emptyStateObserver.await());

    // test StateObserver on existing instance, notify() but don't await()

    ASSERT_FALSE(emptyStateObserver.notify());
}

TEST_F(CanBusSharerTest, SdoClient)
{
    const std::uint8_t id = 0x05;
    const std::uint16_t cobRx = 0x600;
    const std::uint16_t cobTx = 0x580;

    SdoClient sdo(id, cobRx, cobTx, TIMEOUT, getSenderDelegate());
    ASSERT_EQ(sdo.getCobIdRx(), id + cobRx);
    ASSERT_EQ(sdo.getCobIdTx(), id + cobTx);

    std::uint16_t actual1;
    std::uint16_t expected1 = 0x4444;
    std::uint16_t index1 = 0x1234;
    std::uint8_t subindex1 = 0x56;
    std::uint8_t response[8] = {0x4B, 0x00, 0x00, subindex1};
    std::memcpy(response + 1, &index1, 2);
    std::memcpy(response + 4, &expected1, 2);
    f = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 1", &actual1, index1, subindex1));
    ASSERT_EQ(actual1, expected1);
}

} // namespace roboticslab
