#include "gtest/gtest.h"

#include <chrono>
#include <memory>
#include <thread>

#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include "FutureTask.hpp"
#include "DeviceMapper.hpp"

namespace
{
    constexpr double DUMMY_VALUE = 4.444;
}

namespace roboticslab
{

class DummyPositionDirectRawImpl : public yarp::dev::IPositionDirectRaw
{
public:
    virtual bool getAxes(int * axes) override
    { return false; }

    virtual bool setPositionRaw(int j, double ref) override
    { return ref >= 0.0 ? true : false; }

    virtual bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return false; }

    virtual bool setPositionsRaw(const double * refs) override
    { return false; }

    virtual bool getRefPositionRaw(int joint, double * ref) override
    { *ref = DUMMY_VALUE; return true; }

    virtual bool getRefPositionsRaw(double * refs) override
    { return false; };

    virtual bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return false; }
};

/**
 * @ingroup yarp_devices_tests
 * @brief ...
 */
class DeviceMapperTest : public testing::Test
{
public:
    DeviceMapperTest() : dummy(nullptr)
    { }

    virtual void SetUp()
    {
        dummy = new DummyPositionDirectRawImpl;
    }

    virtual void TearDown()
    {
        delete dummy;
        driver.close();
    }

protected:
    yarp::dev::IPositionDirectRaw * getDummy()
    { return dummy; }

    yarp::dev::PolyDriver & getDriver()
    {
        if (!driver.isValid())
        {
            yarp::os::Property config = getDriverConfig();
            driver.open(config);
        }

        return driver;
    }

    static constexpr double EPSILON = 1e-9;

private:
    yarp::os::Property getDriverConfig()
    {
        yarp::os::Property config;
        config.put("device", "fakeMotionControl");
        yarp::os::Property & general = config.addGroup("GENERAL");
        general.put("Joints", 5);

        return config;
    }

    yarp::dev::IPositionDirectRaw * dummy;
    yarp::dev::PolyDriver driver;
};

TEST_F(DeviceMapperTest, SequentialTask)
{
    auto taskFactory = std::unique_ptr<FutureTaskFactory>(new SequentialTaskFactory);
    auto task = taskFactory->createTask();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, 4, 1);
    ASSERT_EQ(task->size(), 1);
    ASSERT_TRUE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, 4, -1);
    ASSERT_EQ(task->size(), 1);
    ASSERT_FALSE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, 4, 1);
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::setPositionRaw, 4, -1);
    ASSERT_EQ(task->size(), 2);
    ASSERT_FALSE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    double ref = 0.0;
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, 4, &ref);

    ASSERT_EQ(task->size(), 1);
    ASSERT_NEAR(ref, 0.0, EPSILON); // not dispatched yet
    ASSERT_TRUE(task->dispatch());
    ASSERT_NEAR(ref, DUMMY_VALUE, EPSILON);
}

TEST_F(DeviceMapperTest, ParallelTask)
{
    auto taskFactory = std::unique_ptr<FutureTaskFactory>(new ParallelTaskFactory(2));
    auto task = taskFactory->createTask();
    ASSERT_EQ(task->size(), 0);

    double ref1 = 0.0;
    double ref2 = 0.0;
    double ref3 = 0.0;
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, 4, &ref1);
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, 4, &ref2);
    task->add(getDummy(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, 4, &ref3);

    ASSERT_EQ(task->size(), 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_NEAR(ref1, DUMMY_VALUE, EPSILON);
    ASSERT_NEAR(ref2, DUMMY_VALUE, EPSILON);
    ASSERT_NEAR(ref3, 0.0, EPSILON); // not dispatched yet
    ASSERT_TRUE(task->dispatch());
    ASSERT_NEAR(ref3, DUMMY_VALUE, EPSILON);
}

TEST_F(DeviceMapperTest, RawDevice)
{
    RawDevice rd(&getDriver());
    auto * p = rd.getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p, nullptr);
}

} // namespace roboticslab
