// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PhidgetSpatial.hpp"

namespace rd
{

// -----------------------------------------------------------------------------

int PhidgetSpatial::read(yarp::sig::Vector &out)
{
    hSemaphore.wait();

    out.resize(DEFAULT_NUM_CHANNELS);

    double Gx = acceleration[0];
    double Gy = acceleration[1];
    double Gz = acceleration[2];
    double modul = sqrt (Gx*Gx+Gy*Gy+Gz*Gz);
    double angle = acos(-Gy/modul)*180.0/M_PI;

    /*printf("=== Data Set: %d ===\n", i);
    printf("Acceleration> x: %6f  y: %6f  z: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
    printf("Angular Rate> x: %6f  y: %6f  z: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
    printf("Magnetic Field> x: %6f  y: %6f  z: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
    printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
    printf("Modul of gravity: %5f  and angle: %6f\n",modul,angle);*/

    out[0] = acceleration[0];
    out[1] = acceleration[1];
    out[2] = acceleration[2];

    out[3] = angularRate[0];
    out[4] = angularRate[1];
    out[5] = angularRate[2];

    out[6] = magneticField[0];
    out[7] = magneticField[1];
    out[8] = magneticField[2];

    out[9] = modul;
    out[10] = angle;

    hSemaphore.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int PhidgetSpatial::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int PhidgetSpatial::getChannels()
{
    return DEFAULT_NUM_CHANNELS;
}

// -----------------------------------------------------------------------------

int PhidgetSpatial::calibrateSensor()
{
    return true;
}

// -----------------------------------------------------------------------------

int PhidgetSpatial::calibrateSensor(const yarp::sig::Vector& value)
{
    return true;
}

// -----------------------------------------------------------------------------

int PhidgetSpatial::calibrateChannel(int ch)
{
    return true;
}

// -----------------------------------------------------------------------------

int PhidgetSpatial::calibrateChannel(int ch, double value)
{
    return true;
}

// -----------------------------------------------------------------------------

}  // namespace rd
