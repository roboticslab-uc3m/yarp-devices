// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

namespace teo
{

/************************************************************************/

void InSrPort::onRead(Bottle& b) {
    switch ( b.get(0).asVocab() ) {
        case VOCAB_FOLLOW_ME:
            printf("enabling callback\n");
            inCvPortPtr->useCallback();
            break;
        case VOCAB_STOP_FOLLOWING:
            printf("disabling callback\n");
            inCvPortPtr->disableCallback();
            break;
        default:
            break;
    }
}

/************************************************************************/

}  // namespace teo
