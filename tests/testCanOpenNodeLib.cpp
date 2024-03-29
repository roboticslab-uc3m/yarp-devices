#include "gtest/gtest.h"

#include <cstdint>
#include <cstring>

#include <future>
#include <string>
#include <utility>
#include <vector>

#include "ICanSenderDelegate.hpp"
#include "SdoClient.hpp"
#include "PdoProtocol.hpp"
#include "NmtProtocol.hpp"
#include "EmcyConsumer.hpp"
#include "DriveStatusMachine.hpp"
#include "CanOpenNode.hpp"

#include "FutureObserverLib.hpp"

namespace roboticslab::test
{

/**
 * @ingroup yarp_devices_tests
 * @defgroup testCanOpenNodeLib
 * @brief Unit tests related to @ref CanOpenNodeLib.
 */

/**
 * @ingroup testCanOpenNodeLib
 * @brief Generate 8-byte SDO response data for expedited transfers.
 */
inline std::uint64_t toInt64(std::uint8_t op, std::uint16_t index, std::uint8_t subindex, std::uint32_t data = 0)
{
    return op + (index << 8) + (subindex << 24) + (static_cast<std::uint64_t>(data) << 32);
}

/**
 * @ingroup testCanOpenNodeLib
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
 * @ingroup testCanOpenNodeLib
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
 * @ingroup testCanOpenNodeLib
 * @brief Stores registered fake CAN messages and eases access to them.
 */
class FakeCanSenderDelegate : public ICanSenderDelegate
{
public:
    //! Store message data internally.
    bool prepareMessage(const can_message & msg) override
    { return messages.push_back(msg), true; }

    void reportAvailability(bool available, unsigned int id) override
    {}

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
 * @ingroup testCanOpenNodeLib
 * @brief Manages a FakeCanSenderDelegate and registers asynchronous operations.
 */
class CanOpenNodeTest : public testing::Test,
                        protected FutureObserver
{
public:
    void SetUp() override
    {
        senderDelegate = new FakeCanSenderDelegate;
    }

    void TearDown() override
    {
        shutdown();
        delete senderDelegate;
    }

protected:
    //! Retrieve a pointer to a FakeCanSenderDelegate instance.
    FakeCanSenderDelegate * getSender()
    { return senderDelegate; }

    static constexpr double TIMEOUT = 0.125; // [s]

private:
    FakeCanSenderDelegate * senderDelegate;
};

TEST_F(CanOpenNodeTest, SdoClientExpeditedUpload)
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
    std::memcpy(response + 4, &expected1, 1);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload("Upload test 1", &actual1, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x40, index, subindex));
    ASSERT_EQ(actual1, expected1);

    // test SdoClient::upload(), request 1 byte (lambda overload)

    std::int8_t actual2;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.upload<std::int8_t>("Upload test 2", [&](auto data) { actual2 = data; }, index, subindex));
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
    std::memcpy(response + 4, &expected3, 2);
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

    // test SdoClient::upload with overrun

    std::uint8_t actualOvr;
    response[0] = 0x4F;
    response[3] = 0x69; // different subindex
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.upload("Upload overrun test", &actualOvr, index, subindex));
}

TEST_F(CanOpenNodeTest, SdoClientExpeditedDownload)
{
    const std::uint8_t id = 0x05;
    const std::uint16_t cobRx = 0x600;
    const std::uint16_t cobTx = 0x580;

    SdoClient sdo(id, cobRx, cobTx, TIMEOUT, getSender());
    ASSERT_EQ(sdo.getCobIdRx(), id + cobRx);
    ASSERT_EQ(sdo.getCobIdTx(), id + cobTx);

    const std::uint16_t index = 0x1234;
    const std::uint8_t subindex = 0x56;

    std::uint8_t response[8] = {0x60, 0x00, 0x00, subindex};
    std::memcpy(response + 1, &index, 2);

    // test SdoClient::download(), send 1 byte

    std::int8_t request1 = 0x44;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download("Download test 1", request1, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x2F, index, subindex, request1));

    // test SdoClient::download(), send 2 bytes

    std::int16_t request2 = 0x4444;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download("Download test 2", request2, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x2B, index, subindex, request2));

    // test SdoClient::download(), send 4 bytes

    std::int32_t request3 = 0x44444444;
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_TRUE(sdo.download("Download test 3", request3, index, subindex));
    ASSERT_EQ(getSender()->getLastMessage().id, sdo.getCobIdRx());
    ASSERT_EQ(getSender()->getLastMessage().len, 8);
    ASSERT_EQ(getSender()->getLastMessage().data, toInt64(0x23, index, subindex, request3));

    // test SdoClient::download with overrun

    std::uint8_t requestOvr = 0x44;
    response[3] = 0x69; // different subindex
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.download("Download overrun test", requestOvr, index, subindex));

    // test SDO abort transfer in download() operation

    response[0] = 0x80;
    std::uint32_t abortCode = 0x06090011; // "Sub-index does not exist"
    std::memcpy(response + 4, &abortCode, 4);
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(response); }});
    ASSERT_FALSE(sdo.download<std::int8_t>("Download test 4", 0x44, index, subindex));
}

TEST_F(CanOpenNodeTest, SdoClientSegmented)
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

    ASSERT_TRUE(sdo.upload("Upload test 2", [&](const auto & data) { actual2 = data; }, index, subindex));

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

TEST_F(CanOpenNodeTest, SdoClientPing)
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

TEST_F(CanOpenNodeTest, ReceivePdo)
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
    const std::uint8_t mapperLSB = mapper & 0x00FF;
    const std::uint8_t mapperMSB = mapper >> 8;

    const std::uint16_t inhibitTime = 0x1234;
    const std::uint16_t eventTimer = 0x5678;

    const std::uint16_t mapping1 = 0x1111;
    const std::uint8_t mapping1sub = 0x45;
    const std::uint16_t mapping2 = 0x2000;

    ReceivePdo rpdo1(id, cob, n, &sdo, getSender());
    ASSERT_EQ(rpdo1.getCobId(), cobId);

    // test ReceivePdo::configure()

    PdoConfiguration rpdo1Conf;
    rpdo1Conf.setTransmissionType(type);
    rpdo1Conf.setInhibitTime(inhibitTime);
    rpdo1Conf.setEventTimer(eventTimer);
    rpdo1Conf.addMapping<std::int16_t>(mapping1, mapping1sub).addMapping<std::int32_t>(mapping2);
    rpdo1Conf.setValid(true);

    const std::uint8_t responseUpload[8] = {0x43, commLSB, commMSB, 0x01, cobIdLSB, cobIdMSB};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(responseUpload); }});

    const std::uint8_t responseDownload1[8] = {0x60, commLSB, commMSB, 0x01}; // COB-ID
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return sdo.notify(responseDownload1); }});
    const std::uint8_t responseDownload2[8] = {0x60, commLSB, commMSB, 0x02}; // transmission type
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return sdo.notify(responseDownload2); }});
    const std::uint8_t responseDownload3[8] = {0x60, commLSB, commMSB, 0x03}; // inhibit time
    f() = std::async(std::launch::async, observer_timer{MILLIS * 4, [&]{ return sdo.notify(responseDownload3); }});
    const std::uint8_t responseDownload4[8] = {0x60, commLSB, commMSB, 0x05}; // event timer
    f() = std::async(std::launch::async, observer_timer{MILLIS * 5, [&]{ return sdo.notify(responseDownload4); }});
    const std::uint8_t responseDownload5[8] = {0x60, mapperLSB, mapperMSB, 0x00}; // start mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 6, [&]{ return sdo.notify(responseDownload5); }});
    const std::uint8_t responseDownload6[8] = {0x60, mapperLSB, mapperMSB, 0x01}; // first mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 7, [&]{ return sdo.notify(responseDownload6); }});
    const std::uint8_t responseDownload7[8] = {0x60, mapperLSB, mapperMSB, 0x02}; // second mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 8, [&]{ return sdo.notify(responseDownload7); }});
    const std::uint8_t responseDownload8[8] = {0x60, mapperLSB, mapperMSB, 0x00}; // end mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 9, [&]{ return sdo.notify(responseDownload8); }});
    const std::uint8_t responseDownload9[8] = {0x60, commLSB, commMSB, 0x01}; // set valid
    f() = std::async(std::launch::async, observer_timer{MILLIS * 10, [&]{ return sdo.notify(responseDownload9); }});

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

TEST_F(CanOpenNodeTest, TransmitPdo)
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
    const std::uint8_t mapperLSB = mapper & 0x00FF;
    const std::uint8_t mapperMSB = mapper >> 8;

    const std::uint16_t inhibitTime = 0x1234;
    const std::uint16_t eventTimer = 0x5678;
    const std::uint8_t syncStartValue = 0x77;

    const std::uint16_t mapping1 = 0x6111; // 0x6111 > 0x2000, testing PDO internals
    const std::uint8_t mapping1sub = 0x45;
    const std::uint16_t mapping2 = 0x2000;

    TransmitPdo tpdo1(id, cob, n, &sdo);
    ASSERT_EQ(tpdo1.getCobId(), cobId);

    // test TransmitPdo::configure()

    PdoConfiguration tpdo1Conf;
    tpdo1Conf.setRtr(false);
    tpdo1Conf.setTransmissionType(type);
    tpdo1Conf.setInhibitTime(inhibitTime);
    tpdo1Conf.setEventTimer(eventTimer);
    tpdo1Conf.setSyncStartValue(syncStartValue);
    tpdo1Conf.addMapping<std::int16_t>(mapping1, mapping1sub).addMapping<std::int32_t>(mapping2);
    tpdo1Conf.setValid(true);

    const std::uint8_t responseUpload[8] = {0x43, commLSB, commMSB, 0x01, cobIdLSB, cobIdMSB};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return sdo.notify(responseUpload); }});

    const std::uint8_t responseDownload1[8] = {0x60, commLSB, commMSB, 0x01}; // COB-ID
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return sdo.notify(responseDownload1); }});
    const std::uint8_t responseDownload2[8] = {0x60, commLSB, commMSB, 0x02}; // transmission type
    f() = std::async(std::launch::async, observer_timer{MILLIS * 3, [&]{ return sdo.notify(responseDownload2); }});
    const std::uint8_t responseDownload3[8] = {0x60, commLSB, commMSB, 0x03}; // inhibit time
    f() = std::async(std::launch::async, observer_timer{MILLIS * 4, [&]{ return sdo.notify(responseDownload3); }});
    const std::uint8_t responseDownload4[8] = {0x60, commLSB, commMSB, 0x05}; // event timer
    f() = std::async(std::launch::async, observer_timer{MILLIS * 5, [&]{ return sdo.notify(responseDownload4); }});
    const std::uint8_t responseDownload5[8] = {0x60, commLSB, commMSB, 0x06}; // SYNC start value
    f() = std::async(std::launch::async, observer_timer{MILLIS * 6, [&]{ return sdo.notify(responseDownload5); }});
    const std::uint8_t responseDownload6[8] = {0x60, mapperLSB, mapperMSB, 0x00}; // start mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 7, [&]{ return sdo.notify(responseDownload6); }});
    const std::uint8_t responseDownload7[8] = {0x60, mapperLSB, mapperMSB, 0x01}; // first mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 8, [&]{ return sdo.notify(responseDownload7); }});
    const std::uint8_t responseDownload8[8] = {0x60, mapperLSB, mapperMSB, 0x02}; // second mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 9, [&]{ return sdo.notify(responseDownload8); }});
    const std::uint8_t responseDownload9[8] = {0x60, mapperLSB, mapperMSB, 0x00}; // end mapping
    f() = std::async(std::launch::async, observer_timer{MILLIS * 10, [&]{ return sdo.notify(responseDownload9); }});
    const std::uint8_t responseDownload10[8] = {0x60, commLSB, commMSB, 0x01}; // set valid
    f() = std::async(std::launch::async, observer_timer{MILLIS * 11, [&]{ return sdo.notify(responseDownload10); }});

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

    tpdo1.registerHandler<std::uint8_t, std::int16_t, std::uint32_t>([&](auto v1, auto v2, auto v3)
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

TEST_F(CanOpenNodeTest, NmtProtocol)
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

TEST_F(CanOpenNodeTest, EmcyConsumer)
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

TEST_F(CanOpenNodeTest, DriveStatusMachine)
{
    const std::uint8_t id = 0x05;
    SdoClient sdo(id, 0x600, 0x580, TIMEOUT, getSender());
    ReceivePdo rpdo(id, 0x200, 1, &sdo, getSender());
    DriveStatusMachine status(&rpdo, TIMEOUT);

    const std::uint16_t notReadyToSwitchOn = 0b0000'0000'0000'0000;
    const std::uint16_t switchOnDisabled = 0b0000'0000'0100'0000;
    const std::uint16_t readyToSwitchOn = 0b0000'0000'0010'0001;
    const std::uint16_t switchedOn = 0b0000'0000'0010'0011;
    const std::uint16_t operationEnabled = 0b0000'0000'0010'0111;
    const std::uint16_t quickStopActive = 0b0000'0000'0000'0111;
    const std::uint16_t faultReactionActive = 0b0000'0000'0000'1111;
    const std::uint16_t fault = 0b0000'0000'0000'1000;

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
    // controlword doesn't change

    ASSERT_TRUE(status.update(fault));
    ASSERT_EQ(status.statusword(), fault);
    ASSERT_EQ(status.getCurrentState(), DriveState::FAULT);
    ASSERT_EQ(DriveStatusMachine::parseStatusword(fault), DriveState::FAULT);
    // controlword doesn't change

    // test random controlword commands

    ASSERT_TRUE(status.controlword(0x1234));
    ASSERT_EQ(getSender()->getLastMessage().id, rpdo.getCobId());
    ASSERT_EQ(getSender()->getLastMessage().len, 2);
    ASSERT_EQ(getSender()->getLastMessage().data, status.controlword().to_ulong());
    ASSERT_EQ(status.controlword(), 0x1234);

    // test reset

    status.reset();
    ASSERT_EQ(status.statusword(), 0);
    ASSERT_EQ(status.controlword(), 0);

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
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return status.update(notReadyToSwitchOn); }}); // fault state resets
    f() = std::async(std::launch::async, observer_timer{MILLIS * 2, [&]{ return status.update(switchOnDisabled); }});
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

TEST_F(CanOpenNodeTest, CanOpenNode)
{
    std::uint8_t id = 0x05;
    CanOpenNode can(id, TIMEOUT, TIMEOUT, getSender());

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

    ASSERT_TRUE(can.notifyMessage({0x80u + id, 8, raw1}));
    ASSERT_EQ(actualCode.first, expectedCode.first);
    ASSERT_EQ(actualReg, expectedReg);
    ASSERT_EQ(actualMsef, expectedMsef);

    // test TPDO1

    std::uint8_t actualTpdo1;
    const std::uint8_t expectedTpdo1 = 0x12;
    const std::uint8_t raw2[] = {expectedTpdo1};
    can.tpdo1()->registerHandler<std::uint8_t>([&](auto v) { actualTpdo1 = v; });
    ASSERT_TRUE(can.notifyMessage({0x180u + id, 1, raw2}));
    ASSERT_EQ(actualTpdo1, expectedTpdo1);

    // test TPDO2

    std::uint8_t actualTpdo2;
    const std::uint8_t expectedTpdo2 = 0x34;
    const std::uint8_t raw3[] = {expectedTpdo2};
    can.tpdo2()->registerHandler<std::uint8_t>([&](auto v) { actualTpdo2 = v; });
    ASSERT_TRUE(can.notifyMessage({0x280u + id, 1, raw3}));
    ASSERT_EQ(actualTpdo2, expectedTpdo2);

    // test TPDO3

    std::uint8_t actualTpdo3;
    const std::uint8_t expectedTpdo3 = 0x56;
    const std::uint8_t raw4[] = {expectedTpdo3};
    can.tpdo3()->registerHandler<std::uint8_t>([&](auto v) { actualTpdo3 = v; });
    ASSERT_TRUE(can.notifyMessage({0x380u + id, 1, raw4}));
    ASSERT_EQ(actualTpdo3, expectedTpdo3);

    // test TPDO4

    std::uint8_t actualTpdo4;
    const std::uint8_t expectedTpdo4 = 0x78;
    const std::uint8_t raw5[] = {expectedTpdo4};
    can.tpdo4()->registerHandler<std::uint8_t>([&](auto v) { actualTpdo4 = v; });
    ASSERT_TRUE(can.notifyMessage({0x480u + id, 1, raw5}));
    ASSERT_EQ(actualTpdo4, expectedTpdo4);

    // test SDO

    const std::uint8_t raw6[8] = {0x60, 0x34, 0x12, 0x56};
    f() = std::async(std::launch::async, observer_timer{MILLIS, [&]{ return can.notifyMessage({0x580u + id, 8, raw6}); }});
    ASSERT_TRUE(can.sdo()->download("Download test 1", 0x00, 0x1234, 0x56));

    // test NMT

    NmtState actualNmt;
    const NmtState expectedNmt = NmtState::OPERATIONAL;
    const std::uint8_t raw7[] = {static_cast<std::uint8_t>(NmtState::OPERATIONAL), id};
    can.nmt()->registerHandler([&](NmtState s) { actualNmt = s; });
    ASSERT_TRUE(can.notifyMessage({0x700u + id, 2, raw7}));
    ASSERT_EQ(actualNmt, expectedNmt);
}

} // namespace roboticslab::test
