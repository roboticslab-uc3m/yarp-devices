#include "gtest/gtest.h"

#include <cstdint>
#include <cstring>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <vector>

#include "StateObserver.hpp"
#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "PdoProtocol.hpp"
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
        for (auto f : futures)
        {
            if (f->valid())
            {
                f->wait();
            }

            delete f;
        }

        delete senderDelegate;
    }

protected:

    CanSenderDelegate * getSenderDelegate()
    { return senderDelegate; }

    std::future<void> & f()
    {
        auto * f = new std::future<void>;
        futures.push_back(f);
        return *f;
    }

    std::vector<std::future<void> *> futures;

    static constexpr double TIMEOUT = 0.1; // [s]
    static constexpr int MILLIS = 10;

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
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return emptyStateObserver.notify(); }});
    ASSERT_TRUE(emptyStateObserver.await());

    // test TypedStateObserver<dummy> with dummy = user-defined type

    struct dummy
    { int val; };

    dummy d;
    const dummy _v1{4};
    TypedStateObserver<dummy> dummyStateObserver(TIMEOUT);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return dummyStateObserver.notify(_v1); }});
    ASSERT_TRUE(dummyStateObserver.await(d));
    ASSERT_EQ(d.val, _v1.val);

    // test TypedStateObserver<int>, use notify(int)

    int val1;
    const int _v2 = 4;
    TypedStateObserver<int> intStateObserver(TIMEOUT);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return intStateObserver.notify(_v2); }});
    ASSERT_TRUE(intStateObserver.await(&val1));
    ASSERT_EQ(val1, _v2);

    // test TypedStateObserver<int>, use notify(const std::uint8_t *, std::size_t)

    std::int8_t val2;
    const std::uint8_t _v3[] = {4};
    TypedStateObserver<std::int8_t> intBuffStateObserver(TIMEOUT);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return intBuffStateObserver.notify(_v3, 1); }});
    ASSERT_TRUE(intBuffStateObserver.await(&val2));
    ASSERT_EQ(val2, _v3[0]);

    // test TypedStateObserver<std::uint8_t[]>

    std::uint8_t buff[1];
    const std::uint8_t _v4[] = {4};
    TypedStateObserver<std::uint8_t[]> bufferStateObserver(TIMEOUT);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return bufferStateObserver.notify(_v4, 1); }});
    ASSERT_TRUE(bufferStateObserver.await(buff));
    ASSERT_EQ(buff[0], _v4[0]);

    // test StateObserver, never call notify()

    StateObserver orphanStateObserver(TIMEOUT);
    ASSERT_FALSE(orphanStateObserver.await());

    // test StateObserver on existing instance, await() but don't notify()

    ASSERT_FALSE(emptyStateObserver.await());

    // test StateObserver on existing instance, await() and notify()

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return emptyStateObserver.notify(); }});
    ASSERT_TRUE(emptyStateObserver.await());

    // test StateObserver on existing instance, notify() but don't await()

    ASSERT_FALSE(emptyStateObserver.notify());
}

TEST_F(CanBusSharerTest, SdoClientExpedited)
{
    const std::uint8_t id = 0x05;
    const std::uint16_t cobRx = 0x600;
    const std::uint16_t cobTx = 0x580;

    SdoClient sdo(id, cobRx, cobTx, TIMEOUT, getSenderDelegate());
    ASSERT_EQ(sdo.getCobIdRx(), id + cobRx);
    ASSERT_EQ(sdo.getCobIdTx(), id + cobTx);

    const std::uint16_t index = 0x1234;
    const std::uint8_t subindex = 0x56;

    std::uint8_t response[8] = {0x00, 0x00, 0x00, subindex};
    std::memcpy(response + 1, &index, 2);

    std::int32_t expected;

    // test SdoClient::upload(), request 1 byte

    std::int8_t actual1;
    response[0] = 0x4F;
    expected = 0x44;
    std::memcpy(response + 4, &expected, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 1", &actual1, index, subindex));
    ASSERT_EQ(actual1, expected);

    // test SdoClient::upload(), request 1 byte (lambda overload)

    std::int8_t actual2;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload<std::int8_t>("Upload test 2", [&](std::int8_t data) { actual2 = data; }, index, subindex));
    ASSERT_EQ(actual1, expected);

    // test SdoClient::upload(), size mismatch (expect 1 byte, receive 2 bytes)

    response[0] = 0x4B;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.upload("Upload test 1b", &actual1, index, subindex));

    // test SdoClient::upload(), request 2 bytes

    std::int16_t actual3;
    response[0] = 0x4B;
    expected = 0x4444;
    std::memcpy(response + 4, &expected, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 2", &actual3, index, subindex));
    ASSERT_EQ(actual3, expected);

    // test SdoClient::upload(), request 4 bytes

    std::int32_t actual4;
    response[0] = 0x43;
    expected = 0x44444444;
    std::memcpy(response + 4, &expected, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 3", &actual4, index, subindex));
    ASSERT_EQ(actual4, expected);

    std::memset(response, 0x00, 8); // reset

    // test SdoClient::download(), send 1 byte

    response[0] = 0x60;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download<std::int8_t>("Download test 1", 0x44, index, subindex));

    // test SDO abort transfer in download() operation

    response[0] = 0x80;
    std::uint32_t abortCode = 0x06090011; // "Sub-index does not exist"
    std::memcpy(response + 4, &abortCode, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.download<std::int8_t>("Download test 2", 0x44, index, subindex));
}

TEST_F(CanBusSharerTest, SdoClientSegmented)
{
    SdoClient sdo(0x05, 0x600, 0x580, TIMEOUT, getSenderDelegate());

    const std::uint8_t indexMSB = 0x12;
    const std::uint8_t indexLSB = 0x34;

    const std::uint16_t index = (indexMSB << 8) + indexLSB;
    const std::uint8_t subindex = 0x56;

    const std::string expected = "abcdefghijklmno"; // 15 chars

    const std::uint8_t response1[8] = {0x41, indexLSB, indexMSB, subindex, static_cast<std::uint8_t>(expected.size())};
    const std::uint8_t response2[8] = {0x10, 'a', 'b', 'c', 'd', 'e', 'f', 'g'};
    const std::uint8_t response3[8] = {0x00, 'h', 'i', 'j', 'k', 'l', 'm', 'n'};
    const std::uint8_t response4[8] = {0x17, 'o'};

    // test SdoClient::upload(), request string

    std::string actual1;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response1); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return sdo.notify(response2); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return sdo.notify(response3); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 4, [&]{ return sdo.notify(response4); }});
    ASSERT_TRUE(sdo.upload("Upload test 1", actual1, index, subindex));
    ASSERT_EQ(actual1, expected);

    // test SdoClient::upload(), request string (lambda overload)

    std::string actual2;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response1); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return sdo.notify(response2); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return sdo.notify(response3); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 4, [&]{ return sdo.notify(response4); }});
    ASSERT_TRUE(sdo.upload("Upload test 2", [&](const std::string & data) { actual2 = data; }, index, subindex));
    ASSERT_EQ(actual2, expected);

    // test SdoClient::download(), send string

    const std::uint8_t response5[8] = {0x60, indexLSB, indexMSB, subindex};
    const std::uint8_t response6[8] = {0x20};
    const std::uint8_t response7[8] = {0x30};
    const std::uint8_t response8[8] = {0x20};

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response5); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return sdo.notify(response6); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return sdo.notify(response7); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 4, [&]{ return sdo.notify(response8); }});
    ASSERT_TRUE(sdo.download("Download test", expected, index, subindex));
}

TEST_F(CanBusSharerTest, ReceivePdo)
{
    SdoClient sdo(0x05, 0x600, 0x580, TIMEOUT, getSenderDelegate());

    const std::uint8_t id = 0x05;
    const std::uint16_t cob = 0x200;
    const std::uint16_t cobId = cob + id;
    const std::uint8_t cobIdLSB = cobId & 0x00FF;
    const std::uint8_t cobIdMSB = cobId >> 8;
    const unsigned int n = 1;

    ReceivePdo rpdo1(id, cob, n, &sdo, getSenderDelegate());
    ASSERT_EQ(rpdo1.getCobId(), cobId);

    const std::uint8_t responseUpload[8] = {0x43, static_cast<std::uint8_t>(n - 1), 0x14, 0x01, cobIdLSB, cobIdMSB};
    const std::uint8_t responseDownload[8] = {0x60};

    PdoConfiguration rpdo1Conf;
    rpdo1Conf.setValid(true);
    rpdo1Conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC_N(0x04));
    rpdo1Conf.setInhibitTime(0x1234);
    rpdo1Conf.setEventTimer(0x5678);
    rpdo1Conf.addMapping<std::int16_t>(0x1111, 0x45).addMapping<std::int32_t>(0x2000);

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(responseUpload); }});

    for (int i = 2; i <= 10; i++)
    {
        f() = std::async(std::launch::async, observer_timer{MILLIS * i, [&]{ return sdo.notify(responseDownload); }});
    }

    ASSERT_TRUE(rpdo1.configure(rpdo1Conf));

    bool res = rpdo1.write<std::int16_t, std::int32_t>(4444, 44444444);
    ASSERT_TRUE(res); // strange compile bug
}

} // namespace roboticslab
