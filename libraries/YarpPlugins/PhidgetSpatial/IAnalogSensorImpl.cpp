// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

int PhidgetSpatial::read(yarp::sig::Vector &out)
{
    hSemaphore.wait();

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

    out = {
        acceleration[0],
        acceleration[1],
        acceleration[2],

        angularRate[0],
        angularRate[1],
        angularRate[2],

        magneticField[0],
        magneticField[1],
        magneticField[2],

        modul,
        angle
    };

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
    return 11;
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
