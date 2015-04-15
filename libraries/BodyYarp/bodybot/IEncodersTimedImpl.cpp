// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IEncodersTimed Related -----------------------------------------

bool teo::BodyBot::getEncodersTimed(double *encs, double *time) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= getEncoderTimed(i,&(encs[i]),&(time[i]));
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoderTimed(int j, double *encs, double *time) {
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    return iEncodersTimedRaw[j]->getEncoderTimedRaw( 0, encs, time );
}

// -----------------------------------------------------------------------------
