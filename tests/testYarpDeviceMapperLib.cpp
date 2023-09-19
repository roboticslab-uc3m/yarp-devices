#include "gtest/gtest.h"

#include <algorithm> // std::copy_n, std::fill_n
#include <chrono>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include "FutureTask.hpp"
#include "DeviceMapper.hpp"

namespace roboticslab::test
{

/**
 * @ingroup yarp_devices_tests
 * @defgroup testYarpDeviceMapperLib
 * @brief Unit tests related to @ref YarpDeviceMapperLib.
 */

/**
 * @ingroup testYarpDeviceMapperLib
 * @brief Dummy implementation of a IPositionDirectRaw interface.
 *
 * Single-axis position-direct controller. Certain methods have been
 * overridden to produce predictable fake results than can be later tracked
 * by unit-testing client code.
 */
struct DummyPositionDirectRaw : public yarp::dev::IPositionDirectRaw
{
    bool getAxes(int * axes) override
    { *axes = 1; return true; }

    bool setPositionRaw(int j, double ref) override
    { return ref >= 0.0 ? true : false; }

    bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return false; }

    bool setPositionsRaw(const double * refs) override
    { return false; }

    bool getRefPositionRaw(int joint, double * ref) override
    { *ref = joint; return true; }

    bool getRefPositionsRaw(double * refs) override
    { int axes; bool ret; return ret = getAxes(&axes), std::iota(refs, refs + axes, 0.0), ret; };

    bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { std::copy_n(joints, n_joint, refs); return true; }
};

/**
 * @ingroup testYarpDeviceMapperLib
 * @brief Dummy implementation of a ISixAxisForceTorqueSensors interface.
 *
 * Single-sensor six-axis FT controller. Certain methods have been
 * overridden to produce predictable fake results than can be later tracked
 * by unit-testing client code.
 */
struct DummySixAxisForceTorqueSensors : public yarp::dev::ISixAxisForceTorqueSensors
{
    std::size_t getNrOfSixAxisForceTorqueSensors() const override
    { return 1; }

    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
    { return yarp::dev::MAS_OK; }

    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
    { name = "dummy"; return true; }

    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const
    { frameName = "dummy"; return true; }

    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
    { out[sens_index] = sens_index; timestamp = sens_index; return true; }
};

/**
 * @ingroup testYarpDeviceMapperLib
 * @brief Dummy implementation of a N-axis raw motor subdevice.
 *
 * This device provides utilities that help create static instances of itself
 * via YARP device registry, thus avoiding the need of prior installation and
 * consequent lookup in order to use them.
 *
 * @tparam N Number of controlled axes.
 */
template<unsigned int N>
struct JointDriver : public yarp::dev::DeviceDriver,
                     public DummyPositionDirectRaw
{
    //! Retrieve the number of controlled axes.
    bool getAxes(int * axes) override
    { *axes = N; return true; }

    //! Generate a dummy name that identifies this device given the number of axes.
    static const std::string name()
    { return "JointDriver" + std::to_string(N); }
};

/**
 * @ingroup testYarpDeviceMapperLib
 * @brief Dummy implementation of a N-component sensor subdevice.
 *
 * This device provides utilities that help create static instances of itself
 * via YARP device registry, thus avoiding the need of prior installation and
 * consequent lookup in order to use them.
 *
 * @tparam N Number of connected sensors.
 */
template<unsigned int N>
struct SensorDriver : public yarp::dev::DeviceDriver,
                      public DummySixAxisForceTorqueSensors
{
    //! Retrieve the number of connected sensors.
    std::size_t getNrOfSixAxisForceTorqueSensors() const override
    { return N; }

    //! Generate a dummy name that identifies this device given the number of sensors.
    static const std::string name()
    { return "SensorDriver" + std::to_string(N); }
};

/**
 * @ingroup testYarpDeviceMapperLib
 * @brief Provide a specialized YARP driver factory that creates instances of a dummy driver.
 */
template<typename T>
yarp::dev::DriverCreator * dummyDriverCreator()
{ return new yarp::dev::DriverCreatorOf<T>(T::name().c_str(), "", T::name().c_str()); }

/**
 * @ingroup testYarpDeviceMapperLib
 * @brief Wrapper class for dummy driver instances.
 */
class YarpDeviceMapperTest : public testing::Test
{
public:
    YarpDeviceMapperTest() : dummyMotor(nullptr), dummySensor(nullptr)
    {}

    void SetUp() override
    {}

    void TearDown() override
    {
        delete dummyMotor;
        dummyMotor = nullptr;

        delete dummySensor;
        dummySensor = nullptr;

        for (int i = 0; i < drivers.size(); i++)
        {
            delete drivers[i]->poly;
        }
    }

protected:
    //! Retrieve interface pointer to a singleton DummyPositionDirectRaw instance.
    yarp::dev::IPositionDirectRaw * getDummyMotor()
    {
        if (!dummyMotor)
        {
            dummyMotor = new DummyPositionDirectRaw;
        }

        return dummyMotor;
    }

    //! Retrieve interface pointer to a singleton DummySixAxisForceTorqueSensors instance.
    yarp::dev::ISixAxisForceTorqueSensors * getDummySensor()
    {
        if (!dummySensor)
        {
            dummySensor = new DummySixAxisForceTorqueSensors;
        }

        return dummySensor;
    }

    /**
     * @brief Instantiate a wrapped dummy driver device.
     *
     * The lifetime of created devices ends by the destruction of this class.
     *
     * @tparam T Any specialized class template of a dummy driver.
     * @return A pointer to the created device.
     */
    template<typename T>
    yarp::dev::PolyDriver * createDriver()
    {
        yarp::dev::Drivers::factory().add(dummyDriverCreator<T>());

        yarp::os::Property config {
            {"device", yarp::os::Value(T::name())},
            {"id", yarp::os::Value(T::name())}
        };

        auto * driver = new yarp::dev::PolyDriver(config);
        drivers.push(driver, T::name().c_str());
        return driver;
    }

    static constexpr double EPSILON = 1e-9;

private:
    yarp::dev::IPositionDirectRaw * dummyMotor;
    yarp::dev::ISixAxisForceTorqueSensors * dummySensor;
    yarp::dev::PolyDriverList drivers;
};

TEST_F(YarpDeviceMapperTest, SequentialTask)
{
    const int joint = 4;

    auto taskFactory = std::make_unique<SequentialTaskFactory>();
    auto task = taskFactory->createTask();
    ASSERT_EQ(task->size(), 0);

    int a = 0;

    task->add([&a] { a = 1; return true; });
    ASSERT_EQ(task->size(), 1);
    ASSERT_TRUE(task->dispatch());
    ASSERT_EQ(a, 1);

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add([](int * _a) { *_a = 2; return true; }, &a);
    ASSERT_EQ(task->size(), 1);
    ASSERT_TRUE(task->dispatch());
    ASSERT_EQ(a, 2);

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, 1);
    ASSERT_EQ(task->size(), 1);
    ASSERT_TRUE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, -1);
    ASSERT_EQ(task->size(), 1);
    ASSERT_FALSE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, 1);
    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::setPositionRaw, joint, -1);
    ASSERT_EQ(task->size(), 2);
    ASSERT_FALSE(task->dispatch());

    task->clear();
    ASSERT_EQ(task->size(), 0);

    double ref = 0.0;
    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref);

    ASSERT_EQ(task->size(), 1);
    ASSERT_NEAR(ref, 0.0, EPSILON); // not dispatched yet
    ASSERT_TRUE(task->dispatch());
    ASSERT_NEAR(ref, joint, EPSILON);
}

TEST_F(YarpDeviceMapperTest, ParallelTask)
{
    const int joint = 4;

    auto taskFactory = std::make_unique<ParallelTaskFactory>(2);
    auto task = taskFactory->createTask();
    ASSERT_EQ(task->size(), 0);

    double ref1 = 0.0;
    double ref2 = 0.0;
    double ref3 = 0.0;
    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref1);
    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref2);
    task->add(getDummyMotor(), &yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, &ref3);

    ASSERT_EQ(task->size(), 3);
    ASSERT_NEAR(ref1, 0.0, EPSILON); // not dispatched yet
    ASSERT_NEAR(ref2, 0.0, EPSILON); // not dispatched yet
    ASSERT_NEAR(ref2, 0.0, EPSILON); // not dispatched yet
    ASSERT_TRUE(task->dispatch());
    ASSERT_NEAR(ref1, joint, EPSILON);
    ASSERT_NEAR(ref2, joint, EPSILON);
    ASSERT_NEAR(ref3, joint, EPSILON);
}

TEST_F(YarpDeviceMapperTest, RawDevice)
{
    auto * driver = createDriver<JointDriver<1>>();
    RawDevice rd(driver);
    ASSERT_NE(rd.getHandle<yarp::dev::IPositionDirectRaw>(), nullptr);
    ASSERT_EQ(rd.getHandle<yarp::dev::IPositionControlRaw>(), nullptr);
    ASSERT_EQ(rd.castToType<std::string>(), nullptr);
    ASSERT_EQ(rd.getId(), driver->id());
    ASSERT_TRUE(rd.isValid());

    ASSERT_EQ(invalidDevice.getHandle<yarp::dev::IPositionDirectRaw>(), nullptr);
    ASSERT_EQ(invalidDevice.getHandle<yarp::dev::IPositionControlRaw>(), nullptr);
    ASSERT_EQ(invalidDevice.castToType<std::string>(), nullptr);
    ASSERT_FALSE(invalidDevice.isValid());
}

TEST_F(YarpDeviceMapperTest, DeviceMapper)
{
    DeviceMapper mapper;

    // DeviceMapper::registerDevice

    ASSERT_TRUE(mapper.registerDevice(createDriver<JointDriver<1>>()));  // motor indices:  [0],   global indices: [0]
    ASSERT_TRUE(mapper.registerDevice(createDriver<SensorDriver<4>>())); // sensor indices: [0-3], global indices: [1-4]
    ASSERT_TRUE(mapper.registerDevice(createDriver<JointDriver<2>>()));  // motor indices:  [1-2], global indices: [5-6]
    ASSERT_TRUE(mapper.registerDevice(createDriver<SensorDriver<3>>())); // sensor indices: [4-6], global indices: [7-9]
    ASSERT_TRUE(mapper.registerDevice(createDriver<JointDriver<3>>()));  // motor indices:  [3-5], global indices: [10-12]
    ASSERT_TRUE(mapper.registerDevice(createDriver<SensorDriver<2>>())); // sensor indices: [7-8], global indices: [13-14]
    ASSERT_TRUE(mapper.registerDevice(createDriver<JointDriver<4>>()));  // motor indices:  [6-9], global indices: [15-18]
    ASSERT_TRUE(mapper.registerDevice(createDriver<SensorDriver<1>>())); // sensor indices: [9],   global indices: [19]

    ASSERT_EQ(mapper.getDevices().size(), 8);

    const int localIndex0 = 0;
    const int localIndex1 = 1;
    const int localIndex2 = 2;
    const int localIndex3 = 3;

    {
        // DeviceMapper::getMotorDevice

        int axes1, axes2, axes3, axes4;
        double ref0, ref1, ref2, ref3;

        // device 1 [0]

        auto [dev0, idx0] = mapper.getMotorDevice(0);
        auto * p0 = dev0->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p0, nullptr);
        ASSERT_EQ(idx0, localIndex0);

        ASSERT_TRUE(p0->getAxes(&axes1));
        ASSERT_EQ(axes1, 1);

        ASSERT_TRUE(p0->getRefPositionRaw(localIndex0, &ref0));
        ASSERT_NEAR(ref0, localIndex0, EPSILON);

        // device 2 [1-2]

        auto [dev1, idx1] = mapper.getMotorDevice(1);
        auto * p1 = dev1->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p1, nullptr);
        ASSERT_EQ(idx1, localIndex0);

        auto [dev2, idx2] = mapper.getMotorDevice(2);
        auto * p2 = dev2->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p2, nullptr);
        ASSERT_EQ(idx2, localIndex1);

        ASSERT_EQ(p1, p2);

        ASSERT_TRUE(p1->getAxes(&axes2));
        ASSERT_EQ(axes2, 2);

        ASSERT_TRUE(p1->getRefPositionRaw(localIndex0, &ref0));
        ASSERT_NEAR(ref0, localIndex0, EPSILON);
        ASSERT_TRUE(p1->getRefPositionRaw(localIndex1, &ref1));
        ASSERT_NEAR(ref1, localIndex1, EPSILON);

        // device 3 [3-5]

        auto [dev3, idx3] = mapper.getMotorDevice(3);
        auto * p3 = dev3->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p3, nullptr);
        ASSERT_EQ(idx3, localIndex0);

        auto [dev4, idx4] = mapper.getMotorDevice(4);
        auto * p4 = dev4->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p4, nullptr);
        ASSERT_EQ(idx4, localIndex1);

        auto [dev5, idx5] = mapper.getMotorDevice(5);
        auto * p5 = dev5->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p5, nullptr);
        ASSERT_EQ(idx5, localIndex2);

        ASSERT_EQ(p3, p4);
        ASSERT_EQ(p3, p5);

        ASSERT_TRUE(p3->getAxes(&axes3));
        ASSERT_EQ(axes3, 3);

        ASSERT_TRUE(p3->getRefPositionRaw(localIndex0, &ref0));
        ASSERT_NEAR(ref0, localIndex0, EPSILON);
        ASSERT_TRUE(p3->getRefPositionRaw(localIndex1, &ref1));
        ASSERT_NEAR(ref1, localIndex1, EPSILON);
        ASSERT_TRUE(p3->getRefPositionRaw(localIndex2, &ref2));
        ASSERT_NEAR(ref2, localIndex2, EPSILON);

        // device 4 [6-9]

        auto [dev6, idx6] = mapper.getMotorDevice(6);
        auto * p6 = dev6->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p6, nullptr);
        ASSERT_EQ(idx6, localIndex0);

        auto [dev7, idx7] = mapper.getMotorDevice(7);
        auto * p7 = dev7->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p7, nullptr);
        ASSERT_EQ(idx7, localIndex1);

        auto [dev8, idx8] = mapper.getMotorDevice(8);
        auto * p8 = dev8->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p8, nullptr);
        ASSERT_EQ(idx8, localIndex2);

        auto [dev9, idx9] = mapper.getMotorDevice(9);
        auto * p9 = dev9->getHandle<yarp::dev::IPositionDirectRaw>();
        ASSERT_NE(p9, nullptr);
        ASSERT_EQ(idx9, localIndex3);

        ASSERT_EQ(p6, p7);
        ASSERT_EQ(p6, p8);
        ASSERT_EQ(p6, p9);

        ASSERT_TRUE(p6->getAxes(&axes4));
        ASSERT_EQ(axes4, 4);

        ASSERT_TRUE(p6->getRefPositionRaw(localIndex0, &ref0));
        ASSERT_NEAR(ref0, localIndex0, EPSILON);
        ASSERT_TRUE(p6->getRefPositionRaw(localIndex1, &ref1));
        ASSERT_NEAR(ref1, localIndex1, EPSILON);
        ASSERT_TRUE(p6->getRefPositionRaw(localIndex2, &ref2));
        ASSERT_NEAR(ref2, localIndex2, EPSILON);
        ASSERT_TRUE(p6->getRefPositionRaw(localIndex3, &ref3));
        ASSERT_NEAR(ref3, localIndex3, EPSILON);

        // DeviceMapper::getControlledAxes

        ASSERT_EQ(mapper.getControlledAxes(), axes1 + axes2 + axes3 + axes4);

        // DeviceMapper::getMotorDevicesWithOffsets

        const auto & devicesWithOffsets = mapper.getMotorDevicesWithOffsets();
        ASSERT_EQ(devicesWithOffsets.size(), 4);

        ASSERT_EQ(std::get<0>(devicesWithOffsets[0]), dev0);
        ASSERT_EQ(std::get<0>(devicesWithOffsets[1]), dev1);
        ASSERT_EQ(std::get<0>(devicesWithOffsets[2]), dev3);
        ASSERT_EQ(std::get<0>(devicesWithOffsets[3]), dev6);

        ASSERT_EQ(std::get<1>(devicesWithOffsets[0]), 0);
        ASSERT_EQ(std::get<1>(devicesWithOffsets[1]), 1);
        ASSERT_EQ(std::get<1>(devicesWithOffsets[2]), 3);
        ASSERT_EQ(std::get<1>(devicesWithOffsets[3]), 6);

        // DeviceMapper::getMotorDevicesWithIndices

        const int globalAxesCount = 5;
        const int globalAxes[globalAxesCount] = {2, 4, 5, 6, 8};
        auto devices = mapper.getMotorDevicesWithIndices(globalAxesCount, globalAxes);
        ASSERT_EQ(devices.size(), 3);

        ASSERT_EQ(std::get<0>(devices[0]), dev2);
        ASSERT_EQ(std::get<0>(devices[1]), dev4);
        ASSERT_EQ(std::get<0>(devices[1]), dev5); // same device
        ASSERT_EQ(std::get<0>(devices[2]), dev6);
        ASSERT_EQ(std::get<0>(devices[2]), dev8); // same device

        ASSERT_EQ(std::get<1>(devices[0]).size(), 1);
        ASSERT_EQ(std::get<1>(devices[1]).size(), 2);
        ASSERT_EQ(std::get<1>(devices[2]).size(), 2);

        ASSERT_EQ(std::get<1>(devices[0]), (std::vector<int>{1})); // parens intentional
        ASSERT_EQ(std::get<1>(devices[1]), (std::vector<int>{1, 2}));
        ASSERT_EQ(std::get<1>(devices[2]), (std::vector<int>{0, 2}));

        ASSERT_EQ(std::get<2>(devices[0]), 0);
        ASSERT_EQ(std::get<2>(devices[1]), 1);
        ASSERT_EQ(std::get<2>(devices[2]), 3);

        // DeviceMapper::getDevices

        const auto & allDevices = mapper.getDevices();
        ASSERT_EQ(allDevices[0].get(), dev0);
        ASSERT_EQ(allDevices[2].get(), dev1);
        ASSERT_EQ(allDevices[4].get(), dev3);
        ASSERT_EQ(allDevices[6].get(), dev6);
    }

    {
        // DeviceMapper::getSensorDevice

        auto [devInvalid, idInvalid] = mapper.getSensorDevice<yarp::dev::IThreeAxisGyroscopes>(0);
        ASSERT_FALSE(devInvalid->isValid());
        ASSERT_EQ(devInvalid->getHandle<yarp::dev::IThreeAxisGyroscopes>(), nullptr);
        ASSERT_EQ(idInvalid, 0);

        int sensors1, sensors2, sensors3, sensors4;
        double ref0, ref1, ref2, ref3;
        double timestamp;
        yarp::sig::Vector out(4);

        // device 1 [0-3]

        auto [dev0, idx0] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(0);
        auto * p0 = dev0->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p0, nullptr);
        ASSERT_EQ(idx0, localIndex0);

        auto [dev1, idx1] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(1);
        auto * p1 = dev1->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p1, nullptr);
        ASSERT_EQ(idx1, localIndex1);

        auto [dev2, idx2] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(2);
        auto * p2 = dev2->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p2, nullptr);
        ASSERT_EQ(idx2, localIndex2);

        auto [dev3, idx3] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(3);
        auto * p3 = dev3->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p3, nullptr);
        ASSERT_EQ(idx3, localIndex3);

        ASSERT_EQ(p0, p1);
        ASSERT_EQ(p0, p2);
        ASSERT_EQ(p0, p3);

        sensors1 = p0->getNrOfSixAxisForceTorqueSensors();
        ASSERT_EQ(sensors1, 4);

        ASSERT_TRUE(p0->getSixAxisForceTorqueSensorMeasure(localIndex0, out, timestamp));
        ASSERT_NEAR(timestamp, localIndex0, EPSILON);
        ASSERT_NEAR(out[localIndex0], localIndex0, EPSILON);

        // device 2 [4-6]

        auto [dev4, idx4] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(4);
        auto * p4 = dev4->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p4, nullptr);
        ASSERT_EQ(idx4, localIndex0);

        auto [dev5, idx5] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(5);
        auto * p5 = dev5->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p5, nullptr);
        ASSERT_EQ(idx5, localIndex1);

        auto [dev6, idx6] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(6);
        auto * p6 = dev6->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p6, nullptr);
        ASSERT_EQ(idx6, localIndex2);

        ASSERT_EQ(p4, p5);
        ASSERT_EQ(p4, p6);

        sensors2 = p4->getNrOfSixAxisForceTorqueSensors();
        ASSERT_EQ(sensors2, 3);

        ASSERT_TRUE(p4->getSixAxisForceTorqueSensorMeasure(localIndex1, out, timestamp));
        ASSERT_NEAR(timestamp, localIndex1, EPSILON);
        ASSERT_NEAR(out[localIndex1], localIndex1, EPSILON);

        // device 3 [7-8]

        auto [dev7, idx7] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(7);
        auto * p7 = dev7->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p7, nullptr);
        ASSERT_EQ(idx7, localIndex0);

        auto [dev8, idx8] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(8);
        auto * p8 = dev8->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p8, nullptr);
        ASSERT_EQ(idx8, localIndex1);

        ASSERT_EQ(p7, p8);

        sensors3 = p7->getNrOfSixAxisForceTorqueSensors();
        ASSERT_EQ(sensors3, 2);

        ASSERT_TRUE(p7->getSixAxisForceTorqueSensorMeasure(localIndex2, out, timestamp));
        ASSERT_NEAR(timestamp, localIndex2, EPSILON);
        ASSERT_NEAR(out[localIndex2], localIndex2, EPSILON);

        // device 4 [9]

        auto [dev9, idx9] = mapper.getSensorDevice<yarp::dev::ISixAxisForceTorqueSensors>(9);
        auto * p9 = dev9->getHandle<yarp::dev::ISixAxisForceTorqueSensors>();
        ASSERT_NE(p9, nullptr);
        ASSERT_EQ(idx9, localIndex0);

        sensors4 = p9->getNrOfSixAxisForceTorqueSensors();
        ASSERT_EQ(sensors4, 1);

        ASSERT_TRUE(p9->getSixAxisForceTorqueSensorMeasure(localIndex3, out, timestamp));
        ASSERT_NEAR(timestamp, localIndex3, EPSILON);
        ASSERT_NEAR(out[localIndex3], localIndex3, EPSILON);

        // DeviceMapper::getConnectedSensors

        ASSERT_EQ(mapper.getConnectedSensors<yarp::dev::ISixAxisForceTorqueSensors>(), sensors1 + sensors2 + sensors3 + sensors4);
        ASSERT_EQ(mapper.getConnectedSensors<yarp::dev::IThreeAxisGyroscopes>(), 0);

        // DeviceMapper::getDevices

        const auto & allDevices = mapper.getDevices();
        ASSERT_EQ(allDevices[1].get(), dev0);
        ASSERT_EQ(allDevices[3].get(), dev4);
        ASSERT_EQ(allDevices[5].get(), dev7);
        ASSERT_EQ(allDevices[7].get(), dev9);
    }

    // DeviceMapper::createTask

    auto task = mapper.createTask();
    ASSERT_EQ(task->size(), 0);

    // DeviceMapper::mapSingleJoint

    double ref_single;
    ASSERT_TRUE(mapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::getRefPositionRaw, 8, &ref_single));
    ASSERT_NEAR(ref_single, 2, EPSILON);

    // DeviceMapper::mapAllJoints

    double ref_full[10];
    ASSERT_TRUE(mapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, ref_full));
    ASSERT_EQ(std::vector<double>(ref_full, ref_full + 10), (std::vector<double>{0, 0, 1, 0, 1, 2, 0, 1, 2, 3})); // parens intentional

    // DeviceMapper::mapJointGroup

    const int jointCount = 5;
    const int joints[jointCount] = {1, 3, 5, 7, 9};
    double ref_group[jointCount];
    ASSERT_TRUE(mapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, jointCount, joints, ref_group));
    ASSERT_EQ(std::vector<double>(ref_group, ref_group + jointCount), (std::vector<double>{0, 0, 2, 1, 3})); // parens intentional

    // DeviceMapper::clear

    mapper.clear();
    ASSERT_EQ(mapper.getDevices().size(), 0);
    ASSERT_EQ(mapper.getControlledAxes(), 0);
    ASSERT_EQ(mapper.getConnectedSensors<yarp::dev::ISixAxisForceTorqueSensors>(), 0);
}

} // namespace roboticslab::test
