// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEVICE_MAPPER_HPP__
#define __DEVICE_MAPPER_HPP__

#include <memory>
#include <tuple>
#include <vector>

#include <yarp/dev/PolyDriver.h>

#include "FutureTask.hpp"

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup YarpDeviceMapperLib
 * @brief Aims to standardize the way most calls to YARP interfaces are forwarded.
 */

/**
 * @ingroup YarpDeviceMapperLib
 * @brief Container for YARP raw motor interface handles.
 *
 * Performs all RTTI stuff on initial configuration to avoid dynamic_cast on
 * on every runtime command.
 */
class RawDevice final
{
public:
    //! Constructor, extracts all interface handles of the supplied driver.
    explicit RawDevice(yarp::dev::PolyDriver * driver);

    //! Destructor.
    ~RawDevice();

    /**
     * @brief Retrieve a handle to a motor interface implemented by the device.
     * @tparam T YARP raw motor interface.
     */
    template<typename T>
    T * getHandle() const;

    /**
     * @brief Perform a dynamic cast on the given type.
     * @tparam T A type the original driver should be able to be cast to.
     */
    template<typename T>
    T * castToType() const
    { return dynamic_cast<T *>(driver); }

private:
    class Private;
    std::unique_ptr<Private> priv;

    yarp::dev::DeviceDriver * driver;
};

/**
 * @ingroup YarpDeviceMapperLib
 * @brief Exposes raw subdevice interface handles on a per-axis manner.
 *
 * Some raw subdevices might control several axes. This class knows how to map
 * YARP motor commands to the right controlled axis of the correct subdevice
 * given one or more indices, and forwards the call either in a sequential or
 * a parallel manner (see @ref FutureTask).
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

    //! Whether to enable parallel mappings on how many concurrent threads.
    void enableParallelization(unsigned int concurrentTasks);

    //! Extract interface handles and perform sanity checks.
    bool registerDevice(yarp::dev::PolyDriver * driver);

    //! Tuple of a raw device pointer and either an offset or a local index.
    using dev_index_t = std::tuple<const RawDevice *, int>;

    //! Tuple of a raw device pointer, its local indices and the global index.
    using dev_group_t = std::tuple<const RawDevice *, std::vector<int>, int>;

    /**
     * @brief Retrieve a device handle and its local index given a global index.
     * @param globalAxis The requested global axis index.
     *
     * Aimed for simple, single-joint commands. See example and terminology in
     * class description.
     *
     * @return A pack of the subdevice handle and the obtained local index.
     */
    dev_index_t getDevice(int globalAxis) const;

    /**
     * @brief Retrieve all registered subdevices and their associated offsets.
     *
     * Aimed for full-mapping commands. See example and terminology in class description.
     *
     * @return A vector of packs of subdevice handles and their offsets.
     */
    const std::vector<dev_index_t> & getDevicesWithOffsets() const;

     /**
      * @brief Retrieve subdevices that map to the specified global axes.
      *
      * Aimed for joint-group commands. See example and terminology in class description.
      *
      * @return A vector of packs of subdevices, their requested local indices,
      * and the associated parameter offsets.
      */
    std::vector<dev_group_t> getDevices(int globalAxesCount, const int * globalAxes) const;

    //! Clear all internal handles.
    void clear();

    //! Retrieve number of controlled axes across all subdevices.
    int getControlledAxes() const
    { return totalAxes; }

    //! Create an instance of a deferred task.
    std::unique_ptr<FutureTask> createTask() const
    { return taskFactory->createTask(); }

    //! Alias for a single-joint command. See class description.
    template<typename T, typename... T_ref>
    using single_mapping_fn = bool (T::*)(int, T_ref...);

    //! Single-joint command mapping. See class description.
    template<typename T, typename... T_ref>
    bool mapSingleJoint(single_mapping_fn<T, T_ref...> fn, int j, T_ref... ref)
    {
        auto [device, offset] = getDevice(j);
        T * p = device->getHandle<T>();
        return p ? (p->*fn)(offset, ref...) : false;
    }

    //! Alias for a full-joint command. See class description.
    template<typename T, typename... T_refs>
    using full_mapping_fn = bool (T::*)(T_refs *...);

    //! Full-joint command mapping. See class description.
    template<typename T, typename... T_refs>
    bool mapAllJoints(full_mapping_fn<T, T_refs...> fn, T_refs *... refs)
    {
        auto task = createTask();
        bool ok = false;

        for (const auto & [device, offset] : getDevicesWithOffsets())
        {
            T * p = device->template getHandle<T>();
            ok |= p && (task->add(p, fn, refs + offset...), true);
        }

        // at least one targeted device must implement the 'T' iface
        return ok && task->dispatch();
    }

    //! Alias for a joint-group command. See class description.
    template<typename T, typename... T_refs>
    using multi_mapping_fn = bool (T::*)(int, const int *, T_refs *...);

    //! Joint-group command mapping. See class description.
    template<typename T, typename... T_refs>
    bool mapJointGroup(multi_mapping_fn<T, T_refs...> fn, int n_joint, const int * joints, T_refs *... refs)
    {
        auto task = createTask();
        auto devices = getDevices(n_joint, joints); // extend lifetime of local joint vector
        bool ok = true;

        for (const auto & [device, localIndices, globalIndex] : devices)
        {
            T * p = device->template getHandle<T>();
            ok &= p && (task->add(p, fn, localIndices.size(), localIndices.data(), refs + globalIndex...), true);
        }

        // all targeted devices must implement the 'T' iface
        return ok && task->dispatch();
    }

private:
    std::vector<dev_index_t> rawDevicesWithOffsets;
    std::vector<int> rawDeviceIndexAtGlobalAxisIndex;
    int totalAxes;

    std::unique_ptr<FutureTaskFactory> taskFactory;
};

} // namespace roboticslab

#endif // __DEVICE_MAPPER_HPP__
