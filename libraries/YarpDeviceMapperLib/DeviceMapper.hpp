// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEVICE_MAPPER_HPP__
#define __DEVICE_MAPPER_HPP__

#include <cstdlib> // std::size_t

#include <functional> // std::invoke
#include <memory>
#include <tuple>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include <yarp/dev/PolyDriver.h>

#include "FutureTask.hpp"
#include "RawDevice.hpp"

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup YarpDeviceMapperLib
 * @brief Aims to standardize the way most calls to YARP interfaces are forwarded.
 */

/**
 * @ingroup YarpDeviceMapperLib
 * @brief Exposes raw subdevice interface handles on a per-axis manner.
 *
 * Some raw subdevices might control several axes. This class knows how to map
 * YARP commands to the right controlled motor axis or sensor of the correct
 * subdevice given one or more indices, and forwards the call either in a
 * sequential or parallel manner (see @ref FutureTask).
 *
 * For example, given three raw subdevices that manage one, two and three axes,
 * respectively, a YARP command that requests global index '2' (zero-based) must
 * be mapped to the second axis of the second subdevice and conveniently forwarded.
 *
 * Terminology:
 *
 * - <b>global index</b> the joint id exposed by the control board to the YARP
 *   network and used as input in most motor interfaces; '3' in the above example
 *
 * - <b>local index</b> the joint id of the requested axis within the specific
 *   subdevice context; '1' in the above example (the second axis of the second
 *   device)
 *
 * - <b>subdevice offset</b> the global index at which local indices of a subdevice
 *   start; '0', '1' and '3' for the subdevices in the above example, respectively
 *
 * - <b>parameter offset</b> the starting index at which array values pointed to
 *   by the array parameter (see `values` in the following examples) refer to the
 *   given subdevice
 *
 * - <b>single-joint</b> mapping: `command(int globalId, type value)`
 *
 * - <b>full-joint</b> mapping: `command(type * values)`
 *
 * - <b>joint-group</b> mapping: `command(int n, const int * globalIds, type * values)`
 */
class DeviceMapper final
{
public:
    //! Constructor.
    DeviceMapper();

    //! Destructor.
    ~DeviceMapper();

    //! Whether to enable parallel mappings and on how many concurrent threads.
    void enableParallelization(unsigned int concurrentTasks);

    //! Extract interface handles and perform sanity checks.
    bool registerDevice(yarp::dev::PolyDriver * driver);

    //! Delete all internal handles.
    void clear();

    //! Create an instance of a deferred task.
    std::unique_ptr<FutureTask> createTask() const
    { return taskFactory->createTask(); }

    //! Tuple of a raw device pointer and either an offset or a local index.
    using dev_index_t = std::tuple<const RawDevice *, int>;

    //! Tuple of a raw device pointer, its local indices and the global index.
    using dev_group_t = std::tuple<const RawDevice *, std::vector<int>, int>;

    /**
     * @brief Retrieve all registered raw devices, regardless of type.
     * @return A vector of raw device smart pointers.
     */
    const std::vector<std::unique_ptr<const RawDevice>> & getDevices() const
    { return devices; }

    /**
     * @brief Retrieve a motor device handle and its local index given a global index.
     * @param globalAxis The requested global axis index.
     *
     * Aimed for simple, single-joint motor commands. See example and terminology in
     * class description.
     *
     * @return A pack of the subdevice handle and the obtained local index.
     */
    dev_index_t getMotorDevice(int globalAxis) const;

    /**
     * @brief Retrieve all registered motor subdevices and their associated offsets.
     *
     * Aimed for full-mapping motor commands. See example and terminology in class description.
     *
     * @return A vector of packs of subdevice handles and their offsets.
     */
    std::vector<dev_index_t> getMotorDevicesWithOffsets() const;

    /**
      * @brief Retrieve motor subdevices that map to the specified global axes.
      *
      * Aimed for joint-group motor commands. See example and terminology in class description.
      *
      * @return A vector of packs of subdevices, their requested local indices,
      * and the associated parameter offsets.
      */
    std::vector<dev_group_t> getMotorDevicesWithIndices(int globalAxesCount, const int * globalAxes) const;

    //! Retrieve the number of controlled axes across all subdevices.
    int getControlledAxes() const
    { return totalAxes; }

    //! Alias for a single-joint command. See class description.
    template<typename T, typename... T_ref>
    using motor_single_joint_fn = bool (T::*)(int, T_ref...);

    //! Single-joint command mapping. See class description.
    template<typename T, typename... T_ref>
    bool mapSingleJoint(motor_single_joint_fn<T, T_ref...> fn, int j, T_ref... ref)
    {
        auto [device, offset] = getMotorDevice(j);
        T * p = device->getHandle<T>();
        return p ? std::invoke(fn, p, offset, ref...) : false;
    }

    //! Alias for a full-joint command. See class description.
    template<typename T, typename... T_refs>
    using motor_all_joints_fn = bool (T::*)(T_refs *...);

    //! Full-joint command mapping. See class description.
    template<typename T, typename... T_refs>
    bool mapAllJoints(motor_all_joints_fn<T, T_refs...> fn, T_refs *... refs)
    {
        auto task = createTask();
        bool ok = false;

        for (const auto & [device, offset] : getMotorDevicesWithOffsets())
        {
            T * p = device->template getHandle<T>();
            ok |= p && (task->add(p, fn, refs + offset...), true);
        }

        // at least one targeted device must implement the 'T' iface
        return ok && task->dispatch();
    }

    //! Alias for a joint-group command. See class description.
    template<typename T, typename... T_refs>
    using motor_multi_joints_fn = bool (T::*)(int, const int *, T_refs *...);

    //! Joint-group command mapping. See class description.
    template<typename T, typename... T_refs>
    bool mapJointGroup(motor_multi_joints_fn<T, T_refs...> fn, int n_joint, const int * joints, T_refs *... refs)
    {
        auto task = createTask();
        auto devices = getMotorDevicesWithIndices(n_joint, joints); // extend lifetime of vector of local indices
        bool ok = true;

        for (const auto & [device, localIndices, globalIndex] : devices)
        {
            T * p = device->template getHandle<T>();
            ok &= p && (task->add(p, fn, localIndices.size(), localIndices.data(), refs + globalIndex...), true);
        }

        // all targeted devices must implement the 'T' iface
        return ok && task->dispatch();
    }

    //! Retrieve the number of connected sensors of the specified type across all subdevices.
    template<typename T>
    int getConnectedSensors() const
    {
        // operator[] will insert a default-constructed value if not found
        if (auto it = connectedSensors.find(typeid(T)); it != connectedSensors.cend())
        {
            return it->second;
        }

        return 0;
    }

    /**
     * @brief Retrieve a sensor device handle and its local index given a global index.
     * @param globalAxis The requested global sensor index.
     * @tparam T The requested sensor type.
     * @return A pack of the subdevice handle and the obtained local index.
     */
    template<typename T>
    dev_index_t getSensorDevice(int globalIndex) const
    {
        if (auto it = sensorOffsets.find(typeid(T)); it != sensorOffsets.cend())
        {
            const auto & [deviceIndex, offset, count] = it->second[globalIndex];
            return {devices[deviceIndex].get(), globalIndex - offset};
        }

        return {&invalidDevice, 0};
    }

    template<typename T, typename T_out>
    using sensor_status_fn = T_out (T::*)(std::size_t) const;

    /**
      * @brief Retrieve the status of the sensor device at the specified global index.
      * @return An integer value representing sensor status.
      */
    template<typename T, typename T_out>
    T_out getSensorStatus(sensor_status_fn<T, T_out> fn, std::size_t index) const
    {
        auto [device, offset] = getSensorDevice<T>(index);
        T * p = device->template getHandle<T>();
        return p ? std::invoke(fn, p, offset) : static_cast<T_out>(DeviceMapper::getSensorFailureStatus());
    }

    template<typename T>
    using sensor_size_fn = std::size_t (T::*)(std::size_t) const;

    /**
      * @brief Retrieve the size of the sensor array at the specified global index.
      * @return An integer value representing the sensor array size.
      */
    template<typename T>
    std::size_t getSensorArraySize(sensor_size_fn<T> fn, std::size_t index) const
    {
        auto [device, offset] = getSensorDevice<T>(index);
        T * p = device->template getHandle<T>();
        return p ? std::invoke(fn, p, offset) : 0;
    }

    template<typename T, typename... T_out_params>
    using sensor_output_fn = bool (T::*)(std::size_t, T_out_params &...) const;

    /**
      * @brief Retrieve information from the sensor device at the specified global index.
      * @return True whether everything went fine, false otherwise.
      */
    template<typename T, typename... T_out_params>
    bool getSensorOutput(sensor_output_fn<T, T_out_params...> fn, std::size_t index, T_out_params &... params) const
    {
        auto [device, offset] = getSensorDevice<T>(index);
        T * p = device->template getHandle<T>();
        return p ? std::invoke(fn, p, offset, params...) : false;
    }

private:
    static const int getSensorFailureStatus();

    using dev_index_offset_t = std::tuple<int, int, int>;

    std::vector<std::unique_ptr<const RawDevice>> devices;
    std::vector<dev_index_offset_t> motorOffsets;
    std::unordered_map<std::type_index, std::vector<dev_index_offset_t>> sensorOffsets;
    std::unordered_map<std::type_index, int> connectedSensors;

    int totalAxes {0};

    std::unique_ptr<FutureTaskFactory> taskFactory;
};

} // namespace roboticslab

#endif // __DEVICE_MAPPER_HPP__
