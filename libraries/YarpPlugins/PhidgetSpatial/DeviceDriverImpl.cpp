// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PhidgetSpatial.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool PhidgetSpatial::open(yarp::os::Searchable & config)
{
    yCInfo(PHSP) << "PhidgetSpatial::open() this:" << this;

    int result;
    const char *err;

    // Declare a phidget handle
    hSpatial0 = 0;  // CPhidgetSpatialHandle

    // Create the phidget object
    CPhidgetSpatial_create(&hSpatial0);

    // Set the handlers to be run when the device is plugged in or opened from software,
    // unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler(reinterpret_cast<CPhidgetHandle>(hSpatial0), AttachHandler, this);
    CPhidget_set_OnDetach_Handler(reinterpret_cast<CPhidgetHandle>(hSpatial0), DetachHandler, this);
    CPhidget_set_OnError_Handler(reinterpret_cast<CPhidgetHandle>(hSpatial0), ErrorHandler, this);

    // Registers a callback that will run if an input changes.
    // Requires the handle for the Phidget, the function that will be called, and an arbitrary
    // pointer that will be supplied to the callback function (may be NULL).
    //	CPhidgetSpatial_set_OnInputChange_Handler(hSpatial0, this->InputChangeHandler, this);

    // Registers a callback that will run if the encoder changes.
    // Requires the handle for the Spatial, the function that will be called, and an arbitrary
    // pointer that will be supplied to the callback function (may be NULL).
    // CPhidgetSpatial_set_OnPositionChange_Handler (hSpatial0, this->PositionChangeHandler0, this);
    // CPhidgetSpatial_set_OnSpatialData_Handler(this->spatial, SpatialDataHandler, NULL);
    CPhidgetSpatial_set_OnSpatialData_Handler(hSpatial0, SpatialDataHandler, this);

    CPhidget_open(reinterpret_cast<CPhidgetHandle>(hSpatial0), -1);  // Was serial number

    // Get the program to wait for an encoder device to be attached
    yCInfo(PHSP) << "Waiting for spatial to be attached....";

    if ((result = CPhidget_waitForAttachment(reinterpret_cast<CPhidgetHandle>(hSpatial0), 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        yCError(PHSP) << "Problem waiting for attachment:" << err;
        return false;
    }

    display_properties(hSpatial0);

    yCInfo(PHSP) << "PhidgetSpatial open() phidget";
    return true;
}

// -----------------------------------------------------------------------------

bool PhidgetSpatial::close()
{
    return true;
}

// -----------------------------------------------------------------------------
