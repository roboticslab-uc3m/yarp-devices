// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"


// ---------------------------- IInteractionMode Related ----------------------------------

bool teo::CanBusControlboard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    *mode = interactionMode[axis];
    return true;
}

bool teo::CanBusControlboard::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

        bool ok = true;
        for(unsigned int i=0; i < nodes.size(); i++)
        {
            if( joints[i] )
            {
                ok &= getInteractionMode(i,&modes[i]);
            }
        }
        return ok;
}

bool teo::CanBusControlboard::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < nodes.size(); i++)
        ok &= getInteractionMode(i,&modes[i]);
    return ok;
}

bool teo::CanBusControlboard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d)\n",axis);

    interactionModeSemaphore.wait();
    interactionMode[axis] = mode;
    interactionModeSemaphore.post();

    return true;
}

bool teo::CanBusControlboard::setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

       bool ok = true;
       for(int j=0; j<nodes.size(); j++)
       {
           if( joints[j] )
           {
               ok &= this->setInteractionMode(j,modes[j]);
           }
       }
       return ok;
}

bool teo::CanBusControlboard::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        ok &= setInteractionMode(i,modes[i]);
    }
    return ok;
}
