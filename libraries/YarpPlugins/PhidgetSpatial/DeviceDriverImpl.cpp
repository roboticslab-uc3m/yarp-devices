// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PhidgetSpatial.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool PhidgetSpatial::open(yarp::os::Searchable& config)
{
    printf("PhidgetSpatial::open() this: %p\n",this);

    // \begin{encoderInit}
    int result;
    const char *err;
    // Declare an phidget handle
    hSpatial0 = 0;  // CPhidgetSpatialHandle
    // Create the phidget object
    CPhidgetSpatial_create(&hSpatial0);
    // Set the handlers to be run when the device is plugged in or opened from software,
    // unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)hSpatial0, this->AttachHandler, this);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)hSpatial0, this->DetachHandler, this);
    CPhidget_set_OnError_Handler((CPhidgetHandle)hSpatial0, this->ErrorHandler, this);

    // Registers a callback that will run if an input changes.
    // Requires the handle for the Phidget, the function that will be called, and an arbitrary
    // pointer that will be supplied to the callback function (may be NULL).
    //	CPhidgetSpatial_set_OnInputChange_Handler(hSpatial0, this->InputChangeHandler, this);

    // Registers a callback that will run if the encoder changes.
    // Requires the handle for the Spatial, the function that will be called, and an arbitrary
    // pointer that will be supplied to the callback function (may be NULL).
    //	CPhidgetSpatial_set_OnPositionChange_Handler (hSpatial0, this->PositionChangeHandler0, this);
    // CPhidgetSpatial_set_OnSpatialData_Handler(this->spatial, SpatialDataHandler, NULL);
    CPhidgetSpatial_set_OnSpatialData_Handler(hSpatial0, SpatialDataHandler, this);

    CPhidget_open((CPhidgetHandle)hSpatial0, -1);  // Was serial number

    // Get the program to wait for an encoder device to be attached
    printf("Waiting for spatial to be attached....");
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)hSpatial0, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return false;
    }

    // Display the properties of the attached encoder device
    //j//display_properties(hSpatial0);
    printf("[success] PhidgetSpatial open() phidget\n");
    // \end{encoderInit}

    return true;
}

// -----------------------------------------------------------------------------

bool PhidgetSpatial::close()
{
    return true;
}

// -----------------------------------------------------------------------------
