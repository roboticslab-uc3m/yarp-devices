// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordLeftManipulation.hpp"

/************************************************************************/
void RecordRateThread::run() {

    //-- Make room!!
    std::vector< double > leftArmAngles( leftArmNumMotors );
    double leftGripperAngle;

    leftArmEnc->getEncoders( leftArmAngles.data() );
    leftGripperEnc->getEncoder( 0, &leftGripperAngle );

    for(int i=0;i<leftArmNumMotors;i++)
        fprintf(filePtr,"%f ",leftArmAngles[i]);
    fprintf(filePtr,"%f ",leftGripperAngle);
    fprintf(filePtr,"\n");

    CD_DEBUG("--> ");
    for(int i=0;i<leftArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",leftArmAngles[i]);
    CD_DEBUG_NO_HEADER("%f ",leftGripperAngle);
    CD_DEBUG_NO_HEADER("\n");

}

/************************************************************************/

void RecordRateThread::setFilePtr(FILE *value)
{
    filePtr = value;
}

/************************************************************************/

