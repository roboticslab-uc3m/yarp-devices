// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RAW_DEVICE_HPP__
#define __RAW_DEVICE_HPP__

#include <memory>
#include <string>

#include <yarp/dev/PolyDriver.h>

namespace roboticslab
{

/**
 * @ingroup YarpDeviceMapperLib
 * @brief Immutable container for YARP raw interface handles.
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
     * @brief Retrieve a handle to a raw interface implemented by the device.
     * @tparam T YARP raw interface.
     */
    template<typename T>
    T * getHandle() const;

    /**
     * @brief Retrieve the device id (can be an empty string if not set).
     * @return A string containing the device id.
     */
    std::string getId() const
    { return valid ? driver->getImplementation()->id() : ""; }

    /**
     * @brief Perform a dynamic cast on the given type.
     * @tparam T A type the original driver should be able to be cast to.
     */
    template<typename T>
    T * castToType() const
    { return valid ? dynamic_cast<T *>(driver->getImplementation()) : nullptr; }

    /**
     * @brief Whether this instance wraps a device with a supported interface.
     * @return True if the device exists and has at least one supported interface.
     */
    bool isValid() const
    { return valid; }

private:
    class Private;
    std::unique_ptr<Private> priv;
    yarp::dev::PolyDriver * driver {nullptr};
    bool valid {false};
};

//! Singleton instance for an invalid (empty) raw device.
inline const RawDevice invalidDevice(nullptr);

} // namespace roboticslab

#endif // __RAW_DEVICE_HPP__
