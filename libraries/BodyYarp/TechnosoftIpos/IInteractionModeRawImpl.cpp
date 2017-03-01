// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IInteractionModeRaw Related ----------------------------------------


bool teo::TechnosoftIpos::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    *mode = interactionMode;
    return true;
}


bool teo::TechnosoftIpos::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

        bool ok = true;
            if( joints )
            {
                ok &= getInteractionModeRaw(0,modes);
            }
        return ok;
}

bool teo::TechnosoftIpos::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

    return getInteractionModeRaw(0,modes);
}

bool teo::TechnosoftIpos::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d), (%s)\n",axis, mode); //-- I don't know if this is correct (if I want to print mode?)

    interactionModeSemaphore.wait();
    interactionMode = mode;
    interactionModeSemaphore.post();

    return true;
}

bool teo::TechnosoftIpos::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

   if( joints )
   {
       return this->setInteractionModeRaw(0,*modes);
   }

}

bool teo::TechnosoftIpos::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_INFO("\n");

    return setInteractionModeRaw(0,*modes);

}


