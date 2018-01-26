// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PhidgetSpatial.hpp"

namespace rd
{

// -----------------------------------------------------------------------------

int PhidgetSpatial::AttachHandler(CPhidgetHandle ENC, void *userptr) {
        int serialNo;
        CPhidget_DeviceID deviceID;
        int i, inputcount;

        CPhidget_getSerialNumber(ENC, &serialNo);

        //Retrieve the device ID and number of encoders so that we can set the enables if needed
        CPhidget_getDeviceID(ENC, &deviceID);
        //CPhidgetSpatial_getSpatialCount((CPhidgetSpatialHandle)ENC, &inputcount);
        printf("Spatial %10d attached! \n", serialNo);

        //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047    
        /*if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT) {
                printf("Spatial requires Enable. Enabling inputs....\n");
                for (i = 0 ; i < inputcount ; i++)
                        CPhidgetSpatial_setEnabled((CPhidgetSpatialHandle)ENC, i, 1);
        }*/
        return 0;
}

// ----------------------------------------------------------------------------

int PhidgetSpatial::DetachHandler(CPhidgetHandle ENC, void *userptr) {
	int serialNo;
	CPhidget_getSerialNumber(ENC, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);
	return 0;
}

// ----------------------------------------------------------------------------

int PhidgetSpatial::ErrorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description) {
	printf("Error handled. %d - %s \n", ErrorCode, Description);
	return 0;
}

// ----------------------------------------------------------------------------

//int PhidgetSpatial::InputChangeHandler(CPhidgetSpatialHandle ENC, void *usrptr, int Index, int State) {
//	printf("Input #%i - State: %i \n", Index, State);
//	return 0;
//}

// ----------------------------------------------------------------------------

int PhidgetSpatial::SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
    PhidgetSpatial* thisObject = static_cast<PhidgetSpatial*>(userptr);

    ///printf("Number of Data Packets in this event: %d\n", count);
    for(int i = 0; i < count; i++)
	{
        thisObject->hSemaphore.wait();

        thisObject->acceleration[0] = data[i]->acceleration[0];
        thisObject->acceleration[1] = data[i]->acceleration[1];
        thisObject->acceleration[2] = data[i]->acceleration[2];

        thisObject->angularRate[0] = data[i]->angularRate[0];
        thisObject->angularRate[1] = data[i]->angularRate[1];
        thisObject->angularRate[2] = data[i]->angularRate[2];

        thisObject->magneticField[0] = data[i]->magneticField[0];
        thisObject->magneticField[1] = data[i]->magneticField[1];
        thisObject->magneticField[2] = data[i]->magneticField[2];

        thisObject->hSemaphore.post();

        /*printf("=== Data Set: %d ===\n", i);
        printf("Acceleration> x: %6f  y: %6f  z: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
        printf("Angular Rate> x: %6f  y: %6f  z: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
        printf("Magnetic Field> x: %6f  y: %6f  z: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
        printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
        printf("Modul of gravity: %5f  and angle: %6f\n",modul,angle);*/

	}
    ///printf("---------------------------------------------\n");
	return 0;
}

// ----------------------------------------------------------------------------

// Display the properties of the attached phidget to the screen.
// We will be displaying the name, serial number and version of the attached device.
// Will also display the number of inputs and encoders on this device.
int PhidgetSpatial::display_properties(CPhidgetSpatialHandle phid) {
	int serialNo, version, num_inputs, num_encoders;
	const char* ptr;

	//CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	//CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	//CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	//CPhidgetSpatial_getInputCount(phid, &num_inputs);
	//CPhidgetSpatial_getSpatialCount(phid, &num_encoders);

	printf("Phidget Device: %s\n", ptr);
	//printf("Serial Number: %10d\tVersion: %8d\n", serialNo, version);
	//printf("Num Spatials: %d\tNum Inputs: %d\n", num_encoders, num_inputs);

	return 0;
}

// ----------------------------------------------------------------------------

}  // namespace rd
