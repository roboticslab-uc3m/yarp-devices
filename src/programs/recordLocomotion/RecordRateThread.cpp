// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordLocomotion.hpp"

/************************************************************************/
void RecordRateThread::run() {

    //-- Make room!!
    std::vector< double > vLeftLegAngles( leftLegNumMotors );
    double* leftLegAngles = vLeftLegAngles.data();
    std::vector< double > vRightArmAngles( rightLegNumMotors );
    double* rightLegAngles = vRightArmAngles.data();

    leftLegEnc->getEncoders( leftLegAngles );
    rightLegEnc->getEncoders( rightLegAngles );

    for(int i=0;i<leftLegNumMotors;i++)
        fprintf(filePtr,"%f ",leftLegAngles[i]);
    for(int i=0;i<rightLegNumMotors;i++)
        fprintf(filePtr,"%f ",rightLegAngles[i]);
    fprintf(filePtr,"\n");

    CD_DEBUG("--> ");
    for(int i=0;i<leftLegNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",leftLegAngles[i]);
    CD_DEBUG_NO_HEADER("|");
    for(int i=0;i<rightLegNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",rightLegAngles[i]);
    CD_DEBUG_NO_HEADER("\n");

}

/************************************************************************/

void RecordRateThread::setFilePtr(FILE *value)
{
    filePtr = value;
}

/************************************************************************/

