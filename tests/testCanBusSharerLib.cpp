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
#include "NmtProtocol.hpp"
#include "EmcyConsumer.hpp"
#include "CanUtils.hpp"

namespace roboticslab
{

// expedited transfers
inline std::uint64_t toInt64(std::uint8_t op, std::uint16_t index, std::uint8_t subindex, std::uint32_t data = 0)
{
    return op + (index << 8) + (subindex << 24) + (static_cast<std::uint64_t>(data) << 32);
}

// segmented transfers
inline std::uint64_t toInt64(std::uint8_t op, const std::string & s = "")
{
    std::uint64_t v = op;

    for (auto i = 0u; i < s.size(); i++)
    {
        v += static_cast<std::uint64_t>(s.data()[i]) << (8 * (i + 1));
    }

    return v;
}

struct fake_message
{
    fake_message() : id(0), len(0), data(0)
    { }

    fake_message(const can_message & msg) : id(msg.id), len(msg.len), data(0)
    { std::memcpy(&data, msg.data, len); }

    unsigned int id;
    unsigned int len;
    uint64_t data;
};

class FakeCanSenderDelegate : public CanSenderDelegate
{
public:
    virtual bool prepareMessage(const can_message & msg) override
    { return messages.push_back(msg), true; }

    const fake_message & getLastMessage() const
    { return messages.back(); }

    const fake_message & getMessage(std::size_t n) const
    { return messages.at(n); }

    void flush()
    { messages.clear(); }

private:
    std::vector<fake_message> messages;
};

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

    FakeCanSenderDelegate * getSender()
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

    SdoClient sdo(id, cobRx, cobTx, TIMEOUT, getSender());
    ASSERT_EQ(sdo.getCobIdRx(), id + cobRx);
    ASSERT_EQ(sdo.getCobIdTx(), id + cobTx);

    const std::uint16_t index = 0x1234;
    const std::uint8_t subindex = 0x56;

    std::uint8_t response[8] = {0x00, 0x00, 0x00, subindex};
    std::memcpy(response + 1, &index, 2);

    // test SdoClient::upload(), request 1 byte

    std::int8_t actual1;
    const std::int8_t expected1 = 0x44;
    response[0] = 0x4F;
    std::memcpy(response + 4, &expected1, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 1", &actual1, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x40, index, subindex));
    ASSERT_EQ(actual1, expected1);

    // test SdoClient::upload(), request 1 byte (lambda overload)

    std::int8_t actual2;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload<std::int8_t>("Upload test 2", [&](std::int8_t data) { actual2 = data; }, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x40, index, subindex));
    ASSERT_EQ(actual1, expected1);

    // test SdoClient::upload(), size mismatch (expect 1 byte, receive 2 bytes)

    response[0] = 0x4B;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.upload("Upload test 3", &actual1, index, subindex));

    // test SdoClient::upload(), request 2 bytes

    std::int16_t actual3;
    const std::int16_t expected3 = 0x4444;
    response[0] = 0x4B;
    std::memcpy(response + 4, &expected3, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 4", &actual3, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x40, index, subindex));
    ASSERT_EQ(actual3, expected3);

    // test SdoClient::upload(), request 4 bytes

    std::int32_t actual4;
    const std::int32_t expected4 = 0x44444444;
    response[0] = 0x43;
    std::memcpy(response + 4, &expected4, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 5", &actual4, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x40, index, subindex));
    ASSERT_EQ(actual4, expected4);

    std::memset(response, 0x00, 8); // reset

    // test SdoClient::download(), send 1 byte

    std::int8_t request1 = 0x44;
    response[0] = 0x60;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download("Download test 1", request1, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x2F, index, subindex, request1));

    // test SdoClient::download(), send 2 bytes

    std::int16_t request2 = 0x4444;
    response[0] = 0x60;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download("Download test 2", request2, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x2B, index, subindex, request2));

    // test SdoClient::download(), send 4 bytes

    std::int32_t request3 = 0x44444444;
    response[0] = 0x60;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download("Download test 3", request3, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x23, index, subindex, request3));

    // test SDO abort transfer in download() operation

    response[0] = 0x80;
    std::uint32_t abortCode = 0x06090011; // "Sub-index does not exist"
    std::memcpy(response + 4, &abortCode, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.download<std::int8_t>("Download test 4", 0x44, index, subindex));
}

TEST_F(CanBusSharerTest, SdoClientSegmented)
{
    SdoClient sdo(0x05, 0x600, 0x580, TIMEOUT, getSender());

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

    for (auto i = 0; i < 4; i++)
    {
        ASSERT_EQ(getSender()->getMessage(i).id, sdo.getCobIdRx());
        ASSERT_EQ(getSender()->getMessage(i).len, 8);
    }

    ASSERT_EQ(getSender()->getMessage(0).data, toInt64(0x40, index, subindex));
    ASSERT_EQ(getSender()->getMessage(1).data, toInt64(0x60));
    ASSERT_EQ(getSender()->getMessage(2).data, toInt64(0x70));
    ASSERT_EQ(getSender()->getMessage(3).data, toInt64(0x60));

    ASSERT_EQ(actual1, expected);

    getSender()->flush();

    // test SdoClient::upload(), request string (lambda overload)

    std::string actual2;

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response1); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return sdo.notify(response2); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return sdo.notify(response3); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 4, [&]{ return sdo.notify(response4); }});

    ASSERT_TRUE(sdo.upload("Upload test 2", [&](const std::string & data) { actual2 = data; }, index, subindex));

    for (auto i = 0; i < 4; i++)
    {
        ASSERT_EQ(getSender()->getMessage(i).id, sdo.getCobIdRx());
        ASSERT_EQ(getSender()->getMessage(i).len, 8);
    }

    ASSERT_EQ(getSender()->getMessage(0).data, toInt64(0x40, index, subindex));
    ASSERT_EQ(getSender()->getMessage(1).data, toInt64(0x60));
    ASSERT_EQ(getSender()->getMessage(2).data, toInt64(0x70));
    ASSERT_EQ(getSender()->getMessage(3).data, toInt64(0x60));

    ASSERT_EQ(actual2, expected);

    getSender()->flush();

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

    for (auto i = 0; i < 4; i++)
    {
        ASSERT_EQ(getSender()->getMessage(i).id, sdo.getCobIdRx());
        ASSERT_EQ(getSender()->getMessage(i).len, 8);
    }

    ASSERT_EQ(getSender()->getMessage(0).data, toInt64(0x21, index, subindex, 15));
    ASSERT_EQ(getSender()->getMessage(1).data, toInt64(0x00, expected.substr(0, 7)));
    ASSERT_EQ(getSender()->getMessage(2).data, toInt64(0x10, expected.substr(7, 7)));
    ASSERT_EQ(getSender()->getMessage(3).data, toInt64(0x0D, expected.substr(14, 1)));
}

TEST_F(CanBusSharerTest, ReceivePdo)
{
    SdoClient sdo(0x05, 0x600, 0x580, TIMEOUT, getSender());

    const std::uint8_t id = 0x05;
    const std::uint16_t cob = 0x200;
    const std::uint16_t cobId = cob + id;
    const std::uint8_t cobIdLSB = cobId & 0x00FF;
    const std::uint8_t cobIdMSB = cobId >> 8;

    const unsigned int n = 1;

    PdoTransmissionType type = PdoTransmissionType::SYNCHRONOUS_ACYCLIC;

    const std::uint16_t comm = 0x1400 + n - 1;
    const std::uint8_t commLSB = comm & 0x00FF;
    const std::uint8_t commMSB = comm >> 8;

    const std::uint16_t mapper = 0x1600 + n - 1;
    const std::uint16_t inhibitTime = 0x1234;
    const std::uint16_t eventTimer = 0x5678;

    const std::uint16_t mapping1 = 0x1111;
    const std::uint8_t mapping1sub = 0x45;
    const std::uint16_t mapping2 = 0x2000;

    ReceivePdo rpdo1(id, cob, n, &sdo, getSender());
    ASSERT_EQ(rpdo1.getCobId(), cobId);

    // test ReceivePdo::configure()

    const std::uint8_t responseUpload[8] = {0x43, commLSB, commMSB, 0x01, cobIdLSB, cobIdMSB};
    const std::uint8_t responseDownload[8] = {0x60};

    PdoConfiguration rpdo1Conf;
    rpdo1Conf.setTransmissionType(type);
    rpdo1Conf.setInhibitTime(inhibitTime);
    rpdo1Conf.setEventTimer(eventTimer);
    rpdo1Conf.addMapping<std::int16_t>(mapping1, mapping1sub).addMapping<std::int32_t>(mapping2);
    rpdo1Conf.setValid(true);

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(responseUpload); }});

    for (int i = 2; i < 11; i++)
    {
        f() = std::async(std::launch::async, observer_timer{MILLIS * i, [&]{ return sdo.notify(responseDownload); }});
    }

    ASSERT_TRUE(rpdo1.configure(rpdo1Conf));

    for (auto i = 0; i < 10; i++)
    {
        ASSERT_EQ(getSender()->getMessage(i).id, sdo.getCobIdRx());
        ASSERT_EQ(getSender()->getMessage(i).len, 8);
    }

    ASSERT_EQ(getSender()->getMessage(0).data, toInt64(0x40, comm, 0x01));
    ASSERT_EQ(getSender()->getMessage(1).data, toInt64(0x23, comm, 0x01, cobId + (1 << 31)));
    ASSERT_EQ(getSender()->getMessage(2).data, toInt64(0x2F, comm, 0x02, type));
    ASSERT_EQ(getSender()->getMessage(3).data, toInt64(0x2B, comm, 0x03, inhibitTime));
    ASSERT_EQ(getSender()->getMessage(4).data, toInt64(0x2B, comm, 0x05, eventTimer));
    ASSERT_EQ(getSender()->getMessage(5).data, toInt64(0x2F, mapper, 0x00, 0));
    ASSERT_EQ(getSender()->getMessage(6).data, toInt64(0x23, mapper, 0x01, (mapping1 << 16) + (mapping1sub << 8) + 16));
    ASSERT_EQ(getSender()->getMessage(7).data, toInt64(0x23, mapper, 0x02, (mapping2 << 16) + 32));
    ASSERT_EQ(getSender()->getMessage(8).data, toInt64(0x2F, mapper, 0x00, 2));
    ASSERT_EQ(getSender()->getMessage(9).data, toInt64(0x23, comm, 0x01, cobId));

    getSender()->flush();

    // test ReceivePdo::write()

    ASSERT_TRUE((rpdo1.write<std::int16_t, std::int32_t>(0x1234, 0x98765432))); // double parens are intentional

    ASSERT_EQ(getSender()->getLastMessage().id, cobId);
    ASSERT_EQ(getSender()->getLastMessage().len, 6);
    ASSERT_EQ(getSender()->getLastMessage().data, 0x987654321234);

    // test unsupported property in ReceivePdo::configure()

    rpdo1Conf.setRtr(true);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(responseUpload); }});
    ASSERT_FALSE(rpdo1.configure(rpdo1Conf));
}

TEST_F(CanBusSharerTest, TransmitPdo)
{
    SdoClient sdo(0x05, 0x600, 0x580, TIMEOUT, getSender());

    const std::uint8_t id = 0x05;
    const std::uint16_t cob = 0x180;
    const std::uint16_t cobId = cob + id;
    const std::uint8_t cobIdLSB = cobId & 0x00FF;
    const std::uint8_t cobIdMSB = cobId >> 8;

    const unsigned int n = 1;

    PdoTransmissionType type = PdoTransmissionType::SYNCHRONOUS_CYCLIC_N(0x04);

    const std::uint16_t comm = 0x1800 + n - 1;
    const std::uint8_t commLSB = comm & 0x00FF;
    const std::uint8_t commMSB = comm >> 8;

    const std::uint16_t mapper = 0x1A00 + n - 1;
    const std::uint16_t inhibitTime = 0x1234;
    const std::uint16_t eventTimer = 0x5678;
    const std::uint8_t syncStartValue = 0x77;

    const std::uint16_t mapping1 = 0x1111;
    const std::uint8_t mapping1sub = 0x45;
    const std::uint16_t mapping2 = 0x2000;

    TransmitPdo tpdo1(id, cob, n, &sdo);
    ASSERT_EQ(tpdo1.getCobId(), cobId);

    // test TransmitPdo::configure()

    const std::uint8_t responseUpload[8] = {0x43, commLSB, commMSB, 0x01, cobIdLSB, cobIdMSB};
    const std::uint8_t responseDownload[8] = {0x60};

    PdoConfiguration tpdo1Conf;
    tpdo1Conf.setRtr(false);
    tpdo1Conf.setTransmissionType(type);
    tpdo1Conf.setInhibitTime(inhibitTime);
    tpdo1Conf.setEventTimer(eventTimer);
    tpdo1Conf.setSyncStartValue(syncStartValue);
    tpdo1Conf.addMapping<std::int16_t>(mapping1, mapping1sub).addMapping<std::int32_t>(mapping2);
    tpdo1Conf.setValid(true);

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(responseUpload); }});

    for (int i = 2; i < 12; i++)
    {
        f() = std::async(std::launch::async, observer_timer{MILLIS * i, [&]{ return sdo.notify(responseDownload); }});
    }

    ASSERT_TRUE(tpdo1.configure(tpdo1Conf));

    for (auto i = 0; i < 11; i++)
    {
        ASSERT_EQ(getSender()->getMessage(i).id, sdo.getCobIdRx());
        ASSERT_EQ(getSender()->getMessage(i).len, 8);
    }

    ASSERT_EQ(getSender()->getMessage(0).data, toInt64(0x40, comm, 0x01));
    ASSERT_EQ(getSender()->getMessage(1).data, toInt64(0x23, comm, 0x01, cobId + (1 << 31) + (1 << 30)));
    ASSERT_EQ(getSender()->getMessage(2).data, toInt64(0x2F, comm, 0x02, type));
    ASSERT_EQ(getSender()->getMessage(3).data, toInt64(0x2B, comm, 0x03, inhibitTime));
    ASSERT_EQ(getSender()->getMessage(4).data, toInt64(0x2B, comm, 0x05, eventTimer));
    ASSERT_EQ(getSender()->getMessage(5).data, toInt64(0x2F, comm, 0x06, syncStartValue));
    ASSERT_EQ(getSender()->getMessage(6).data, toInt64(0x2F, mapper, 0x00, 0));
    ASSERT_EQ(getSender()->getMessage(7).data, toInt64(0x23, mapper, 0x01, (mapping1 << 16) + (mapping1sub << 8) + 16));
    ASSERT_EQ(getSender()->getMessage(8).data, toInt64(0x23, mapper, 0x02, (mapping2 << 16) + 32));
    ASSERT_EQ(getSender()->getMessage(9).data, toInt64(0x2F, mapper, 0x00, 2));
    ASSERT_EQ(getSender()->getMessage(10).data, toInt64(0x23, comm, 0x01, cobId + (1 << 30)));

    getSender()->flush();

    // test TransmitPdo::accept(), no handler attached

    ASSERT_FALSE(tpdo1.accept(nullptr, 0));

    // test TransmitPdo::registerHandler() and accept()

    std::uint8_t actual1;
    std::int16_t actual2;
    std::uint32_t actual3;

    tpdo1.registerHandler<std::uint8_t, std::int16_t, std::uint32_t>([&](std::uint8_t v1, std::int16_t v2, std::uint32_t v3)
            { actual1 = v1; actual2 = v2; actual3 = v3; });

    const std::uint8_t expected1 = 0x12;
    const std::int16_t expected2 = 0x1234;
    const std::uint32_t expected3 = 0x12345678;

    std::uint8_t raw[7];
    std::memcpy(raw, &expected1, 1);
    std::memcpy(raw + 1, &expected2, 2);
    std::memcpy(raw + 3, &expected3, 4);
    ASSERT_TRUE(tpdo1.accept(raw, 7));

    ASSERT_EQ(actual1, expected1);
    ASSERT_EQ(actual2, expected2);
    ASSERT_EQ(actual3, expected3);

    // test TransmitPdo::accept(), handler was detached

    tpdo1.unregisterHandler();
    ASSERT_FALSE(tpdo1.accept(nullptr, 0));
}

TEST_F(CanBusSharerTest, NmtProtocol)
{
    const std::uint8_t id = 0x05;
    SdoClient sdo(id, 0x600, 0x580, TIMEOUT, getSender());
    NmtProtocol nmt(id, &sdo, getSender());

    const uint16_t heartbeatPeriod = 0x1234;

    // test NmtProtocol::setupHeartbeat()

    const std::uint8_t response[8] = {0x40, 0x17, 0x10};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(nmt.setupHeartbeat(heartbeatPeriod));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x2B, 0x1017, 0x00, heartbeatPeriod));

    // test NmtProtocol::accept(), no handler attached

    ASSERT_FALSE(nmt.accept(nullptr));

    std::uint8_t temp[1];

    // test NmtProtocol::registerHandler() and accept() in BOOTUP state

    NmtState actual1;
    const NmtState expected1 = NmtState::BOOTUP;
    temp[0] = static_cast<std::uint8_t>(expected1);
    nmt.registerHandler([&](NmtState s) { actual1 = s; });
    ASSERT_TRUE(nmt.accept(temp));
    ASSERT_EQ(actual1, expected1);

    // test NmtProtocol::registerHandler() and accept() in STOPPED state

    NmtState actual2;
    const NmtState expected2 = NmtState::STOPPED;
    temp[0] = static_cast<std::uint8_t>(expected2);
    nmt.registerHandler([&](NmtState s) { actual2 = s; });
    ASSERT_TRUE(nmt.accept(temp));
    ASSERT_EQ(actual2, expected2);

    // test NmtProtocol::registerHandler() and accept() in OPERATIONAL state

    NmtState actual3;
    const NmtState expected3 = NmtState::OPERATIONAL;
    temp[0] = static_cast<std::uint8_t>(expected3);
    nmt.registerHandler([&](NmtState s) { actual3 = s; });
    ASSERT_TRUE(nmt.accept(temp));
    ASSERT_EQ(actual3, expected3);

    // test NmtProtocol::registerHandler() and accept() in PRE_OPERATIONAL state

    NmtState actual4;
    const NmtState expected4 = NmtState::PRE_OPERATIONAL;
    temp[0] = static_cast<std::uint8_t>(expected4);
    nmt.registerHandler([&](NmtState s) { actual4 = s; });
    ASSERT_TRUE(nmt.accept(temp));
    ASSERT_EQ(actual4, expected4);

    // test NmtProtocol::registerHandler() and accept() in unknown state

    temp[0] = 0xFF;
    ASSERT_FALSE(nmt.accept(temp));

    // test NmtProtocol::accept(), handler was detached

    nmt.unregisterHandler();
    ASSERT_FALSE(nmt.accept(nullptr));

    // test NmtProtocol::issueServiceCommand()

    ASSERT_TRUE(nmt.issueServiceCommand(NmtService::START_REMOTE_NODE));
    ASSERT_EQ(getSender()->getLastMessage().id, 0);
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, static_cast<std::uint8_t>(NmtService::START_REMOTE_NODE) + (id << 8));
}

TEST_F(CanBusSharerTest, EmcyConsumer)
{
    SdoClient sdo(0x05, 0x600, 0x580, TIMEOUT, getSender());
    EmcyConsumer emcy(&sdo);

    const uint16_t inhibitTime = 0x1234;

    // test EmcyConsumer::configure()

    const std::uint8_t response[8] = {0x40, 0x15, 0x10};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(emcy.configure(inhibitTime));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x2B, 0x1015, 0x00, inhibitTime));

    // test EmcyConsumer::accept(), no handler attached

    ASSERT_FALSE(emcy.accept(nullptr));

    // test NmtProtocol::registerHandler() and accept()

    EmcyConsumer::code_t actualCode1;
    std::uint8_t actualReg1;
    std::uint64_t actualMsef1 = 0;

    const EmcyConsumer::code_t expectedCode1 = {0x1000, "Generic error"};
    const std::uint8_t expectedReg1 = 0x04;
    const std::uint64_t expectedMsef1 = 0x1234567890;

    std::uint8_t raw1[8];
    std::memcpy(raw1, &expectedCode1.first, 2);
    raw1[2] = expectedReg1;
    std::memcpy(raw1 + 3, &expectedMsef1, 5);

    emcy.registerHandler([&](EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
            {
                actualCode1 = code;
                actualReg1 = reg;
                std::memcpy(&actualMsef1, msef, 5);
            });

    ASSERT_TRUE(emcy.accept(raw1));
    ASSERT_EQ(actualCode1.first, expectedCode1.first);
    ASSERT_EQ(actualReg1, expectedReg1);
    ASSERT_EQ(actualMsef1, expectedMsef1);

    // test NmtProtocol::registerHandler() and accept() with custom code registry

    struct FakeCodeRegistry : public EmcyCodeRegistry
    {
        virtual std::string codeToMessage(std::uint16_t code) override
        { return code == 0x1234 ? "pass" : "fail"; }
    };

    emcy.setErrorCodeRegistry<FakeCodeRegistry>();

    EmcyConsumer::code_t actualCode2;
    std::uint8_t actualReg2;
    std::uint64_t actualMsef2;

    const EmcyConsumer::code_t expectedCode2 = {0x1234, "pass"};
    const std::uint8_t expectedReg2 = 0x04;
    const std::uint64_t expectedMsef2 = 0x1234567890;

    std::uint8_t raw2[8];
    std::memcpy(raw2, &expectedCode2.first, 2);
    raw2[2] = expectedReg1;
    std::memcpy(raw2 + 3, &expectedMsef2, 5);

    emcy.registerHandler([&](EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
            {
                actualCode2 = code;
                actualReg2 = reg;
                std::memcpy(&actualMsef2, msef, 5);
            });

    ASSERT_TRUE(emcy.accept(raw2));
    ASSERT_EQ(actualCode2, expectedCode2);
    ASSERT_EQ(actualReg2, expectedReg2);
    ASSERT_EQ(actualMsef2, expectedMsef2);

    // test EmcyConsumer::accept(), handler was detached

    emcy.unregisterHandler();
    ASSERT_FALSE(emcy.accept(nullptr));
}

} // namespace roboticslab
