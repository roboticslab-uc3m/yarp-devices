// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

/************************************************************************/
void MoveGripperPort::onRead(yarp::os::Bottle &in) {

    int armSizeLeft, armSizeRight;
    iPositionControlLeft->getAxes(&armSizeLeft);
    iPositionControlRight->getAxes(&armSizeRight);

    if( in.size() != 4 ) {
        CD_ERROR("Only '[set] [pos] axis ref' for now\n");
        return;
    }

    if( (in.get(0).asVocab() != VOCAB_SET) || (in.get(0).asString() != "set")  ) {
        CD_ERROR("Only '[set] [pos] axis ref' for now\n");
        return;
    }

    if( (in.get(1).asVocab() != VOCAB_POSITION) || (in.get(0).asString() != "pos")  ) {
        CD_ERROR("Only '[set] [pos] axis ref' for now\n");
        return;
    }

    int j = in.get(2).asInt();
    double ref = in.get(3).asDouble();

    if(j==0)
        iPositionControlLeft->positionMove(armSizeLeft-1,ref);
    else if (j==1)
        iPositionControlRight->positionMove(armSizeRight-1,ref);
}

/************************************************************************/
