#include "gtest/gtest.h"

#include <cstdint>
#include <cstring>

#include <bitset>
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <utility>
#include <vector>

#include "StateObserver.hpp"
#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "PdoProtocol.hpp"
#include "NmtProtocol.hpp"
#include "EmcyConsumer.hpp"
#include "DriveStatusMachine.hpp"
#include "CanOpen.hpp"
#include "CanUtils.hpp"

namespace roboticslab
{

namespace test
{

/**
 * @ingroup yarp_devices_tests
 * @defgroup testCanBusSharerLib
 * @brief Unit tests related to @ref CanBusSharerLib.
 */

/**
 * @ingroup testCanBusSharerLib
 * @brief Generate 8-byte SDO response data for expedited transfers.
 */
inline std::uint64_t toInt64(std::uint8_t op, std::uint16_t index, std::uint8_t subindex, std::uint32_t data = 0)
{
    return op + (index << 8) + (subindex << 24) + (static_cast<std::uint64_t>(data) << 32);
}

/**
 * @ingroup testCanBusSharerLib
 * @brief Generate 8-byte SDO response data for segmented (normal) transfers.
 */
inline std::uint64_t toInt64(std::uint8_t op, const std::string & s = "")
{
    std::uint64_t v = op;

    for (auto i = 0u; i < s.size(); i++)
    {
        v += static_cast<std::uint64_t>(s.data()[i]) << (8 * (i + 1));
    }

    return v;
}

/**
 * @ingroup testCanBusSharerLib
 * @brief Dummy CAN message proxy container of can_message.
 *
 * Since can_message only holds a pointer to CAN message data stored somewhere
 * else, this structure aims to preserve a local copy for later use.
 */
struct fake_message
{
    fake_message() : id(0), len(0), data(0)
    { }

    //! Copy message data into this instance.
    fake_message(const can_message & msg) : id(msg.id), len(msg.len), data(0)
    { std::memcpy(&data, msg.data, len); }

    unsigned int id;
    unsigned int len;
    uint64_t data;
};

/**
 * @ingroup testCanBusSharerLib
 * @brief Stores registered fake CAN messages and eases access to them.
 */
class FakeCanSenderDelegate : public CanSenderDelegate
{
public:
    //! Store message data internally.
    virtual bool prepareMessage(const can_message & msg) override
    { return messages.push_back(msg), true; }

    //! Retrieve last message.
    const fake_message & getLastMessage() const
    { return messages.back(); }

    //! Retrieve message at specified index.
    const fake_message & getMessage(std::size_t n) const
    { return messages.at(n); }

    //! Empties internal message registry.
    void flush()
    { messages.clear(); }

private:
    std::vector<fake_message> messages;
};

/**
 * @ingroup testCanBusSharerLib
 * @brief Manages a FakeCanSenderDelegate and registers asynchronous operations.
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
    //! Retrieve a pointer to a FakeCanSenderDelegate instance.
    FakeCanSenderDelegate * getSender()
    { return senderDelegate; }

    //! Register an asynchronous operation that can be assigned thereafter.
    std::future<void> & f()
    {
        auto * f = new std::future<void>;
        futures.push_back(f);
        return *f;
    }

    static constexpr double TIMEOUT = 0.1; // [s]
    static constexpr int MILLIS = 50;

private:
    FakeCanSenderDelegate * senderDelegate;
    std::vector<std::future<void> *> futures;
};

/**
 * @ingroup testCanBusSharerLib
 * @brief Functor wait-with-callback class.
 */
class observer_timer
{
public:
    //! Register function object and configure wait time.
    template<typename Fn>
    observer_timer(int _milliseconds, Fn && _fn)
        : milliseconds(_milliseconds), fn(std::move(_fn))
    { }

    //! Wait and call stored function.
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

    StateObserver orphanStateObserver1(TIMEOUT);
    ASSERT_FALSE(orphanStateObserver1.await());

    // test StateObserver, never call await()

    StateObserver orphanStateObserver2(TIMEOUT);
    ASSERT_TRUE(orphanStateObserver2.notify());

    // test StateObserver on existing instance, await() but don't notify()

    ASSERT_FALSE(emptyStateObserver.await());

    // test StateObserver on existing instance, await() and notify()

    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return emptyStateObserver.notify(); }});
    ASSERT_TRUE(emptyStateObserver.await());

    // test StateObserver on existing instance, notify() but don't await()

    ASSERT_TRUE(emptyStateObserver.notify());
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

    const std::string s = "abcdefghijklmno"; // 15 chars

    const std::uint8_t response1[8] = {0x41, indexLSB, indexMSB, subindex, static_cast<std::uint8_t>(s.size())};
    const std::uint8_t response2[8] = {0x00, 'a', 'b', 'c', 'd', 'e', 'f', 'g'};
    const std::uint8_t response3[8] = {0x10, 'h', 'i', 'j', 'k', 'l', 'm', 'n'};
    const std::uint8_t response4[8] = {0x07, 'o'};

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

    ASSERT_EQ(actual1, s);

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

    ASSERT_EQ(actual2, s);

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

    ASSERT_TRUE(sdo.download("Download test", s, index, subindex));

    for (auto i = 0; i < 4; i++)
    {
        ASSERT_EQ(getSender()->getMessage(i).id, sdo.getCobIdRx());
        ASSERT_EQ(getSender()->getMessage(i).len, 8);
    }

    ASSERT_EQ(getSender()->getMessage(0).data, toInt64(0x21, index, subindex, 15));
    ASSERT_EQ(getSender()->getMessage(1).data, toInt64(0x00, s.substr(0, 7)));
    ASSERT_EQ(getSender()->getMessage(2).data, toInt64(0x10, s.substr(7, 7)));
    ASSERT_EQ(getSender()->getMessage(3).data, toInt64(0x0D, s.substr(14, 1)));
}

TEST_F(CanBusSharerTest, SdoClientPing)
{
    const std::uint8_t id = 0x05;
    const std::uint16_t cobRx = 0x600;
    const std::uint16_t cobTx = 0x580;

    SdoClient sdo(id, cobRx, cobTx, TIMEOUT, getSender());
    ASSERT_EQ(sdo.getCobIdRx(), id + cobRx);
    ASSERT_EQ(sdo.getCobIdTx(), id + cobTx);

    // test unsuccessful ping

    ASSERT_FALSE(sdo.ping());

    // test successful ping

    std::uint8_t response[8] = {0x00};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.ping());
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

    ASSERT_EQ(getSender()->getLastMessage().id, rpdo1.getCobId());
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

    PdoTransmissionType type = PdoTransmissionType::SYNCHRONOUS_CYCLIC_N<0x04>();

    const std::uint16_t comm = 0x1800 + n - 1;
    const std::uint8_t commLSB = comm & 0x00FF;
    const std::uint8_t commMSB = comm >> 8;

    const std::uint16_t mapper = 0x1A00 + n - 1;
    const std::uint16_t inhibitTime = 0x1234;
    const std::uint16_t eventTimer = 0x5678;
    const std::uint8_t syncStartValue = 0x77;

    const std::uint16_t mapping1 = 0x6111; // 0x6111 > 0x2000, testing PDO internals
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
    NmtProtocol nmt(id, getSender());

    const uint16_t heartbeatPeriod = 0x1234;

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
    EmcyConsumer emcy;

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
    std::uint64_t actualMsef2 = 0;

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

TEST_F(CanBusSharerTest, DriveStatusMachine)
{
    const std::uint8_t id = 0x05;
    SdoClient sdo(id, 0x600, 0x580, TIMEOUT, getSender());
    ReceivePdo rpdo(id, 0x200, 1, &sdo, getSender());
    DriveStatusMachine status(&rpdo, TIMEOUT);

    const std::uint16_t notReadyToSwitchOn = std::bitset<16>("0000000000000000").to_ulong();
    const std::uint16_t switchOnDisabled = std::bitset<16>("0000000001000000").to_ulong();
    const std::uint16_t readyToSwitchOn = std::bitset<16>("0000000000100001").to_ulong();
    const std::uint16_t switchedOn = std::bitset<16>("0000000000100011").to_ulong();
    const std::uint16_t operationEnabled = std::bitset<16>("0000000000100111").to_ulong();
    const std::uint16_t quickStopActive = std::bitset<16>("0000000000000111").to_ulong();
    const std::uint16_t faultReactionActive = std::bitset<16>("0000000000001111").to_ulong();
    const std::uint16_t fault = std::bitset<16>("0000000000001000").to_ulong();

    // test initial state

    ASSERT_EQ(status.statusword(), 0x0000);
    ASSERT_EQ(status.controlword(), 0x0000);

    // test known states

    ASSERT_TRUE(status.update(notReadyToSwitchOn));
    ASSERT_EQ(status.statusword(), notReadyToSwitchOn);
    ASSERT_EQ(status.getCurrentState(), DriveState::NOT_READY_TO_SWITCH_ON);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(notReadyToSwitchOn), DriveState::NOT_READY_TO_SWITCH_ON);
    ASSERT_EQ(status.controlword(), 0x0000);

    ASSERT_TRUE(status.update(switchOnDisabled));
    ASSERT_EQ(status.statusword(), switchOnDisabled);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(switchOnDisabled), DriveState::SWITCH_ON_DISABLED);
    ASSERT_EQ(status.controlword(), static_cast<std::uint16_t>(DriveTransition::DISABLE_VOLTAGE));

    ASSERT_TRUE(status.update(readyToSwitchOn));
    ASSERT_EQ(status.statusword(), readyToSwitchOn);
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(readyToSwitchOn), DriveState::READY_TO_SWITCH_ON);
    ASSERT_EQ(status.controlword(), static_cast<std::uint16_t>(DriveTransition::SHUTDOWN));

    ASSERT_TRUE(status.update(switchedOn));
    ASSERT_EQ(status.statusword(), switchedOn);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(switchedOn), DriveState::SWITCHED_ON);
    ASSERT_EQ(status.controlword(), static_cast<std::uint16_t>(DriveTransition::SWITCH_ON));

    ASSERT_TRUE(status.update(operationEnabled));
    ASSERT_EQ(status.statusword(), operationEnabled);
    ASSERT_EQ(status.getCurrentState(), DriveState::OPERATION_ENABLED);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(operationEnabled), DriveState::OPERATION_ENABLED);
    ASSERT_EQ(status.controlword(), static_cast<std::uint16_t>(DriveTransition::ENABLE_OPERATION));

    ASSERT_TRUE(status.update(quickStopActive));
    ASSERT_EQ(status.statusword(), quickStopActive);
    ASSERT_EQ(status.getCurrentState(), DriveState::QUICK_STOP_ACTIVE);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(quickStopActive), DriveState::QUICK_STOP_ACTIVE);
    ASSERT_EQ(status.controlword(), static_cast<std::uint16_t>(DriveTransition::QUICK_STOP));

    ASSERT_TRUE(status.update(faultReactionActive));
    ASSERT_EQ(status.statusword(), faultReactionActive);
    ASSERT_EQ(status.getCurrentState(), DriveState::FAULT_REACTION_ACTIVE);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(faultReactionActive), DriveState::FAULT_REACTION_ACTIVE);
    ASSERT_EQ(status.controlword(), 0x0000);

    ASSERT_TRUE(status.update(fault));
    ASSERT_EQ(status.statusword(), fault);
    ASSERT_EQ(status.getCurrentState(), DriveState::FAULT);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(fault), DriveState::FAULT);
    ASSERT_EQ(status.controlword(), 0x0000);

    // test random controlword commands

    ASSERT_TRUE(status.controlword(0x1234));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x1234);

    ASSERT_TRUE(status.controlword(0x0000)); // reset

    // test SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON (transition 2: SHUTDOWN)

    ASSERT_TRUE(status.update(switchOnDisabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::SHUTDOWN));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0006);
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test READY_TO_SWITCH_ON -> SWITCHED_ON (transition 3: SWITCH_ON)

    ASSERT_TRUE(status.update(readyToSwitchOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchedOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::SWITCH_ON));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0007);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);

    // test SWITCHED_ON -> OPERATION_ENABLED (transition 4: ENABLE_OPERATION)

    ASSERT_TRUE(status.update(switchedOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(operationEnabled); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::ENABLE_OPERATION));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x000F);
    ASSERT_EQ(status.getCurrentState(), DriveState::OPERATION_ENABLED);

    // test OPERATION_ENABLED -> SWITCHED_ON (transition 5: SWITCH_ON)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchedOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::SWITCH_ON));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0007);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);

    // test OPERATION_ENABLED -> SWITCHED_ON (transition 5, DISABLE_OPERATION (alias))

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchedOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::DISABLE_OPERATION));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0007);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);

    // test SWITCHED_ON -> READY_TO_SWITCH_ON (transition 6: SHUTDOWN)

    ASSERT_TRUE(status.update(switchedOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::SHUTDOWN));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0006);
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test READY_TO_SWITCH_ON -> SWITCH_ON_DISABLED (transition 7: DISABLE_VOLTAGE)

    ASSERT_TRUE(status.update(readyToSwitchOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::DISABLE_VOLTAGE));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0000);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test OPERATION_ENABLED -> READY_TO_SWITCH_ON (transition 8: SHUTDOWN)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::SHUTDOWN));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0006);
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test OPERATION_ENABLED -> SWITCH_ON_DISABLED (transition 9: DISABLE_VOLTAGE)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::DISABLE_VOLTAGE));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0000);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test SWITCHED_ON -> SWITCH_ON_DISABLED (transition 10: DISABLE_VOLTAGE)

    ASSERT_TRUE(status.update(switchedOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::DISABLE_VOLTAGE));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0000);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test OPERATION_ENABLED -> QUICK_STOP_ACTIVE (transition 11: QUICK_STOP)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(quickStopActive); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::QUICK_STOP));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x0002);
    ASSERT_EQ(status.getCurrentState(), DriveState::QUICK_STOP_ACTIVE);

    // test FAULT -> SWITCH_ON_DISABLED (transition 15: FAULT_RESET)

    ASSERT_TRUE(status.update(fault));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::FAULT_RESET));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, static_cast<std::uint16_t>(DriveTransition::FAULT_RESET)); // special case, rising edge
    ASSERT_EQ(status.controlword(), 0x0000);
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test QUICK_STOP_ACTIVE -> OPERATION_ENABLED (transition 16: ENABLE_OPERATION)

    ASSERT_TRUE(status.update(quickStopActive));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(operationEnabled); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::ENABLE_OPERATION));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x000F);
    ASSERT_EQ(status.getCurrentState(), DriveState::OPERATION_ENABLED);

    // test illegal transition SWITCH_ON_DISABLED -> OPERATION_ENABLED ("ENABLE_OPERATION")

    ASSERT_TRUE(status.update(switchOnDisabled));
    ASSERT_FALSE(status.requestTransition(DriveTransition::ENABLE_OPERATION));

    // test non-null controlword on transition SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON (transition 2: SHUTDOWN)

    ASSERT_FALSE(status.controlword().test(14));
    ASSERT_TRUE(status.controlword(status.controlword().set(14)));
    ASSERT_TRUE(status.update(switchOnDisabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestTransition(DriveTransition::SHUTDOWN));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x4006);
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON (state request)

    ASSERT_TRUE(status.update(switchOnDisabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestState(DriveState::READY_TO_SWITCH_ON));
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test SWITCH_ON_DISABLED -> SWITCHED_ON (state request)

    ASSERT_TRUE(status.update(switchOnDisabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return status.update(switchedOn); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCHED_ON));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);

    // test SWITCH_ON_DISABLED -> OPERATION_ENABLED (state request)

    ASSERT_TRUE(status.update(switchOnDisabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return status.update(switchedOn); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return status.update(operationEnabled); }});
    ASSERT_TRUE(status.requestState(DriveState::OPERATION_ENABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::OPERATION_ENABLED);

    // test READY_TO_SWITCH_ON -> SWITCHED_ON (state request)

    ASSERT_TRUE(status.update(readyToSwitchOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchedOn); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCHED_ON));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);

    // test READY_TO_SWITCH_ON -> OPERATION_ENABLED (state request)

    ASSERT_TRUE(status.update(readyToSwitchOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchedOn); }});
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return status.update(operationEnabled); }});
    ASSERT_TRUE(status.requestState(DriveState::OPERATION_ENABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::OPERATION_ENABLED);

    // test SWITCHED_ON -> OPERATION_ENABLED (state request)

    ASSERT_TRUE(status.update(switchedOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(operationEnabled); }});
    ASSERT_TRUE(status.requestState(DriveState::OPERATION_ENABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::OPERATION_ENABLED);

    // test OPERATION_ENABLED -> SWITCHED_ON (state request)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchedOn); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCHED_ON));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCHED_ON);

    // test OPERATION_ENABLED -> READY_TO_SWITCH_ON (state request)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestState(DriveState::READY_TO_SWITCH_ON));
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test OPERATION_ENABLED -> SWITCH_ON_DISABLED (state request)

    ASSERT_TRUE(status.update(operationEnabled));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCH_ON_DISABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test SWITCHED_ON -> READY_TO_SWITCH_ON (state request)

    ASSERT_TRUE(status.update(switchedOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(readyToSwitchOn); }});
    ASSERT_TRUE(status.requestState(DriveState::READY_TO_SWITCH_ON));
    ASSERT_EQ(status.getCurrentState(), DriveState::READY_TO_SWITCH_ON);

    // test SWITCHED_ON -> SWITCH_ON_DISABLED (state request)

    ASSERT_TRUE(status.update(switchedOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCH_ON_DISABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test READY_TO_SWITCH_ON -> SWITCH_ON_DISABLED (state request)

    ASSERT_TRUE(status.update(readyToSwitchOn));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCH_ON_DISABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test QUICK_STOP_ACTIVE -> SWITCH_ON_DISABLED (state request)

    ASSERT_TRUE(status.update(quickStopActive));
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(switchOnDisabled); }});
    ASSERT_TRUE(status.requestState(DriveState::SWITCH_ON_DISABLED));
    ASSERT_EQ(status.getCurrentState(), DriveState::SWITCH_ON_DISABLED);

    // test FAULT -> SWITCH_ON_DISABLED (state request, unsupported)

    ASSERT_TRUE(status.update(fault));
    ASSERT_FALSE(status.requestState(DriveState::SWITCH_ON_DISABLED));
}

TEST_F(CanBusSharerTest, CanOpen)
{
    std::uint8_t id = 0x05;
    CanOpen can(id, TIMEOUT, TIMEOUT, getSender());

    ASSERT_EQ(can.getId(), id);

    ASSERT_NE(can.sdo(), nullptr);
    ASSERT_NE(can.rpdo1(), nullptr);
    ASSERT_NE(can.rpdo2(), nullptr);
    ASSERT_NE(can.rpdo3(), nullptr);
    ASSERT_NE(can.rpdo4(), nullptr);
    ASSERT_NE(can.tpdo1(), nullptr);
    ASSERT_NE(can.tpdo2(), nullptr);
    ASSERT_NE(can.tpdo3(), nullptr);
    ASSERT_NE(can.tpdo4(), nullptr);
    ASSERT_NE(can.emcy(), nullptr);
    ASSERT_NE(can.nmt(), nullptr);
    ASSERT_NE(can.driveStatus(), nullptr);

    ASSERT_EQ(can.sdo()->getCobIdRx(), 0x600 + id);
    ASSERT_EQ(can.sdo()->getCobIdTx(), 0x580 + id);

    ASSERT_EQ(can.rpdo1()->getCobId(), 0x200 + id);
    ASSERT_EQ(can.rpdo2()->getCobId(), 0x300 + id);
    ASSERT_EQ(can.rpdo3()->getCobId(), 0x400 + id);
    ASSERT_EQ(can.rpdo4()->getCobId(), 0x500 + id);

    ASSERT_EQ(can.tpdo1()->getCobId(), 0x180 + id);
    ASSERT_EQ(can.tpdo2()->getCobId(), 0x280 + id);
    ASSERT_EQ(can.tpdo3()->getCobId(), 0x380 + id);
    ASSERT_EQ(can.tpdo4()->getCobId(), 0x480 + id);

    // test EMCY

    EmcyConsumer::code_t actualCode;
    std::uint8_t actualReg;
    std::uint64_t actualMsef = 0;

    const EmcyConsumer::code_t expectedCode = {0x1000, "Generic error"};
    const std::uint8_t expectedReg = 0x04;
    const std::uint64_t expectedMsef = 0x1234567890;

    std::uint8_t raw1[8];
    std::memcpy(raw1, &expectedCode.first, 2);
    raw1[2] = expectedReg;
    std::memcpy(raw1 + 3, &expectedMsef, 5);

    can.emcy()->registerHandler([&](EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
            {
                actualCode = code;
                actualReg = reg;
                std::memcpy(&actualMsef, msef, 5);
            });

    ASSERT_TRUE(can.consumeMessage(0x80 + id, raw1, 8));
    ASSERT_EQ(actualCode.first, expectedCode.first);
    ASSERT_EQ(actualReg, expectedReg);
    ASSERT_EQ(actualMsef, expectedMsef);

    // test TPDO1

    std::uint8_t actualTpdo1;
    const std::uint8_t expectedTpdo1 = 0x12;
    const std::uint8_t raw2[] = {expectedTpdo1};
    can.tpdo1()->registerHandler<std::uint8_t>([&](std::uint8_t v) { actualTpdo1 = v; });
    ASSERT_TRUE(can.consumeMessage(0x180 + id, raw2, 1));
    ASSERT_EQ(actualTpdo1, expectedTpdo1);

    // test TPDO2

    std::uint8_t actualTpdo2;
    const std::uint8_t expectedTpdo2 = 0x34;
    const std::uint8_t raw3[] = {expectedTpdo2};
    can.tpdo2()->registerHandler<std::uint8_t>([&](std::uint8_t v) { actualTpdo2 = v; });
    ASSERT_TRUE(can.consumeMessage(0x280 + id, raw3, 1));
    ASSERT_EQ(actualTpdo2, expectedTpdo2);

    // test TPDO3

    std::uint8_t actualTpdo3;
    const std::uint8_t expectedTpdo3 = 0x56;
    const std::uint8_t raw4[] = {expectedTpdo3};
    can.tpdo3()->registerHandler<std::uint8_t>([&](std::uint8_t v) { actualTpdo3 = v; });
    ASSERT_TRUE(can.consumeMessage(0x380 + id, raw4, 1));
    ASSERT_EQ(actualTpdo3, expectedTpdo3);

    // test TPDO4

    std::uint8_t actualTpdo4;
    const std::uint8_t expectedTpdo4 = 0x78;
    const std::uint8_t raw5[] = {expectedTpdo4};
    can.tpdo4()->registerHandler<std::uint8_t>([&](std::uint8_t v) { actualTpdo4 = v; });
    ASSERT_TRUE(can.consumeMessage(0x480 + id, raw5, 1));
    ASSERT_EQ(actualTpdo4, expectedTpdo4);

    // test SDO

    const std::uint8_t raw6[8] = {0x60};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return can.consumeMessage(0x580 + id, raw6, 8); }});
    ASSERT_TRUE(can.sdo()->download("Download test 1", 0x00, 0x1234));

    // test NMT

    NmtState actualNmt;
    const NmtState expectedNmt = NmtState::OPERATIONAL;
    const std::uint8_t raw7[] = {static_cast<std::uint8_t>(NmtState::OPERATIONAL), id};
    can.nmt()->registerHandler([&](NmtState s) { actualNmt = s; });
    ASSERT_TRUE(can.consumeMessage(0x700 + id, raw7, 2));
    ASSERT_EQ(actualNmt, expectedNmt);
}

} // namespace test
} // namespace roboticslab
