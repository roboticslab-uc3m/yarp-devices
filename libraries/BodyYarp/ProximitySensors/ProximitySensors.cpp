// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProximitySensors.hpp"

#include <ColorDebug.hpp>

const int roboticslab::ProximitySensors::THRESHOLD_GRIPPER = 50;
const int roboticslab::ProximitySensors::THRESHOLD_ALERT = 300;

void roboticslab::ProximitySensors::SensorReader::onRead(yarp::os::Bottle& b)
{
    if (b.size() == 0)
        return;

    sens->gripperMutex.lock();
    if(b.get(14).asDouble() > THRESHOLD_GRIPPER && b.get(14).asDouble() < 1000)
	  {sens->gripper=true;
	   CD_INFO("Botella detectada\n");}
    else
       sens->gripper=false;
    sens->gripperMutex.unlock();

	double max=0;
	for(int i=0;i<b.size();i++)
	{if(b.get(i).asDouble()>max)
	max=b.get(i).asDouble();}
        CD_INFO("Current maximum sensor value: %f\n", max);
    sens->alertMutex.lock();
	sens->alert = max > THRESHOLD_ALERT;
    sens->alertMutex.unlock();
}










