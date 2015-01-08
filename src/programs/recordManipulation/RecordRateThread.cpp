// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

/************************************************************************/
void RecordRateThread::run() {

    //-- Make room!!
    std::vector< double > leftArmAngles( leftArmNumMotors );
    std::vector< double > rightArmAngles( rightArmNumMotors );
    double leftGripperAngle, rightGripperAngle;

    leftArmEnc->getEncoders( leftArmAngles.data() );
    rightArmEnc->getEncoders( rightArmAngles.data() );
    leftGripperEnc->getEncoder( 0, &leftGripperAngle );
    rightGripperEnc->getEncoder( 0, &rightGripperAngle );

    for(int i=0;i<leftArmNumMotors;i++)
        fprintf(filePtr,"%f ",leftArmAngles[i]);
    for(int i=0;i<rightArmNumMotors;i++)
        fprintf(filePtr,"%f ",rightArmAngles[i]);
    fprintf(filePtr,"%f ",leftGripperAngle);
    fprintf(filePtr,"%f ",rightGripperAngle);
    fprintf(filePtr,"\n");

    CD_DEBUG("--> ");
    for(int i=0;i<leftArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",leftArmAngles[i]);
    CD_DEBUG_NO_HEADER("|");
    for(int i=0;i<rightArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",rightArmAngles[i]);
    CD_DEBUG_NO_HEADER("%f ",leftGripperAngle);
    CD_DEBUG_NO_HEADER("%f ",rightGripperAngle);
    CD_DEBUG_NO_HEADER("\n");

}

/************************************************************************/

void RecordRateThread::setFilePtr(FILE *value)
{
    filePtr = value;
}

/************************************************************************/

