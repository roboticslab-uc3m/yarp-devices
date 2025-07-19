// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PhidgetSpatial.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

int PhidgetSpatial::AttachHandler(CPhidgetHandle ENC, void * userptr)
{
    int serialNo;
    CPhidget_DeviceID deviceID;
    // int inputcount;

    CPhidget_getSerialNumber(ENC, &serialNo);

    // Retrieve the device ID and number of encoders so that we can set the enables if needed
    CPhidget_getDeviceID(ENC, &deviceID);

    // CPhidgetSpatial_getSpatialCount((CPhidgetSpatialHandle)ENC, &inputcount);
    yCInfo(PHSP, "Spatial %10d attached", serialNo);

    // the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047
    /*if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT) {
        printf("Spatial requires Enable. Enabling inputs....\n");
        for (auto i = 0 ; i < inputcount ; i++)
            CPhidgetSpatial_setEnabled((CPhidgetSpatialHandle)ENC, i, 1);
    }*/

    return 0;
}

// ----------------------------------------------------------------------------

int PhidgetSpatial::DetachHandler(CPhidgetHandle ENC, void * userptr)
{
    int serialNo;
    CPhidget_getSerialNumber(ENC, &serialNo);
    yCInfo(PHSP, "Spatial %10d detached", serialNo);
    return 0;
}

// ----------------------------------------------------------------------------

int PhidgetSpatial::ErrorHandler(CPhidgetHandle ENC, void * userptr, int ErrorCode, const char * Description)
{
    yCInfo(PHSP) << "Error handled:" << ErrorCode << "=" << Description;
    return 0;
}

// ----------------------------------------------------------------------------

int PhidgetSpatial::SpatialDataHandler(CPhidgetSpatialHandle spatial, void * userptr, CPhidgetSpatial_SpatialEventDataHandle * data, int count)
{
    auto * thisObject = static_cast<PhidgetSpatial *>(userptr);

    for (int i = 0; i < count; i++)
	{
        std::lock_guard lock(thisObject->mtx);

        thisObject->acceleration[0] = data[i]->acceleration[0];
        thisObject->acceleration[1] = data[i]->acceleration[1];
        thisObject->acceleration[2] = data[i]->acceleration[2];

        thisObject->angularRate[0] = data[i]->angularRate[0];
        thisObject->angularRate[1] = data[i]->angularRate[1];
        thisObject->angularRate[2] = data[i]->angularRate[2];

        thisObject->magneticField[0] = data[i]->magneticField[0];
        thisObject->magneticField[1] = data[i]->magneticField[1];
        thisObject->magneticField[2] = data[i]->magneticField[2];

        thisObject->timestamp = data[i]->timestamp.seconds + data[i]->timestamp.microseconds * 1e-6;

        // module of gravity and angle
        // double modul = std::sqrt(Gx * Gx + Gy * Gy + Gz * Gz);
        // double angle = std::acos(-Gy / module) * 180.0 / M_PI;
	}

	return 0;
}

// ----------------------------------------------------------------------------

int PhidgetSpatial::display_properties(CPhidgetSpatialHandle phid)
{
    const char * ptr;
    int serialNo, version;

    CPhidget_getDeviceType(reinterpret_cast<CPhidgetHandle>(phid), &ptr);
    CPhidget_getSerialNumber(reinterpret_cast<CPhidgetHandle>(phid), &serialNo);
    CPhidget_getDeviceVersion(reinterpret_cast<CPhidgetHandle>(phid), &version);

    yCInfo(PHSP) << "Phidget Device:" << ptr;
    yCInfo(PHSP, "Serial Number: %10d", serialNo);
    yCInfo(PHSP, "Version: %8d", version);

    return 0;
}

// ----------------------------------------------------------------------------
