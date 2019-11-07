#include "gtest/gtest.h"

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>

// upstream bug in IPositionDirect.h, remove in YARP 3.2+
#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR < 2
# include <yarp/os/Vocab.h>
#endif

#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include "FutureTask.hpp"
#include "DeviceMapper.hpp"

namespace
{
    constexpr double DUMMY_VALUE = 4.444;

    struct DummyPositionDirectRawImpl : public yarp::dev::IPositionDirectRaw
    {
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

    template<unsigned int N>
    struct JointDriver : public yarp::dev::DeviceDriver,
                         public DummyPositionDirectRawImpl
    {
        virtual bool getAxes(int * axes) override
        { *axes = 1; return true; }

        static std::string name()
        { return "JointDriver" + std::to_string(N); }

        static yarp::dev::DriverCreator * makeCreator()
        { return new yarp::dev::DriverCreatorOf<JointDriver<N>>(name().c_str(), "", name().c_str()); }
    };
}

namespace roboticslab
{

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
    { }

    virtual void TearDown()
    {
        delete dummy;

        for (int i = 0; i < drivers.size(); i++)
        {
            delete drivers[i]->poly;
        }
    }

protected:
    yarp::dev::IPositionDirectRaw * getDummy()
    {
        if (!dummy)
        {
            dummy = new DummyPositionDirectRawImpl;
        }

        return dummy;
    }

    template<typename T>
    yarp::dev::PolyDriver * getDriver()
    {
        yarp::dev::Drivers::factory().add(T::makeCreator());

        yarp::os::Property config;
        config.put("device", T::name());

        auto * driver = new yarp::dev::PolyDriver(config);
        drivers.push(driver, T::name().c_str());
        return driver;
    }

    static constexpr double EPSILON = 1e-9;

private:
    yarp::dev::IPositionDirectRaw * dummy;
    yarp::dev::PolyDriverList drivers;
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
    RawDevice rd(getDriver<JointDriver<1>>());
    auto * p = rd.getHandle<yarp::dev::IPositionDirectRaw>();
    ASSERT_NE(p, nullptr);
}

} // namespace roboticslab
