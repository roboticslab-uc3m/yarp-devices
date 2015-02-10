// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

/************************************************************************/
void MoveGripperPort::onRead(yarp::os::Bottle &in) {

    int armSizeLeft, armSizeRight;
    iPositionControlLeft->getAxes(&armSizeLeft);
    iPositionControlRight->getAxes(&armSizeRight);

    int j = in.get(0).asInt();
    int ref = in.get(1).asInt();
    if(j==0)
        iPositionControlLeft->positionMove(armSizeLeft-1,ref);
    else if (j==1)
        iPositionControlRight->positionMove(armSizeRight-1,ref);
}

/************************************************************************/
