// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

namespace teo
{

/************************************************************************/
void RecordRateThread::run()
{

    //-- Make room!!
    std::vector< double > leftArmPoss( leftArmNumMotors );
    std::vector< double > rightArmPoss( rightArmNumMotors );

    leftArmEnc->getEncoders( leftArmPoss.data() );
    rightArmEnc->getEncoders( rightArmPoss.data() );

    for(int i=0; i<leftArmNumMotors; i++)
        fprintf(filePtr,"%f ",leftArmPoss[i]);
    for(int i=0; i<rightArmNumMotors; i++)
        fprintf(filePtr,"%f ",rightArmPoss[i]);
    fprintf(filePtr,"\n");

    CD_DEBUG("--> ");
    for(int i=0; i<leftArmNumMotors; i++)
        CD_DEBUG_NO_HEADER("%f ",leftArmPoss[i]);
    CD_DEBUG_NO_HEADER("|");
    for(int i=0; i<rightArmNumMotors; i++)
        CD_DEBUG_NO_HEADER("%f ",rightArmPoss[i]);
    CD_DEBUG_NO_HEADER("\n");

}

/************************************************************************/

void RecordRateThread::setFilePtr(FILE *value)
{
    filePtr = value;
}

/************************************************************************/

}  // namespace teo
