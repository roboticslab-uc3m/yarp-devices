// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo
{

/************************************************************************/

bool PlaybackThread::threadInit() {

    leftArmDone = false;
    rightArmDone = false;

    ifs.open( this->fileName.c_str() );
    if( ! ifs.is_open() ) {
        CD_ERROR("Could not open read file: %s.\n", this->fileName.c_str());
        return false;
    }
    CD_SUCCESS("Opened file: %s.\n", this->fileName.c_str());

    return true;

}

/************************************************************************/
void PlaybackThread::run() {

    CD_DEBUG("Begin parsing file.\n" );
    std::string line;
    int lineCount = 1;
    while( getline( ifs, line) && ( ! this->isStopping() ) ) {

        CD_DEBUG("[L:%d] %s\n", lineCount,line.c_str() );
        yarp::os::Bottle lineBottle(line);  //-- yes, using a bottle to parse a string
        //CD_DEBUG("[L:%d] string from bottle from string: %s\n", lineCount,lineBottle.toString().c_str() );
        if( leftArmNumMotors+rightArmNumMotors != lineBottle.size() )
            CD_ERROR("-------------SIZE!!!!!!!!!!!!!\n");

        std::vector< double > leftArmOutDoubles( leftArmNumMotors );
        for(int i=0;i<leftArmNumMotors;i++)
            leftArmOutDoubles[i] = lineBottle.get(i).asDouble();

        std::vector< double > rightArmOutDoubles( rightArmNumMotors );
        for(int i=0;i<rightArmNumMotors;i++)
            rightArmOutDoubles[i] = lineBottle.get(leftArmNumMotors+i).asDouble();

        leftArmPosDirect->setPositions( leftArmOutDoubles.data() );
        rightArmPosDirect->setPositions( rightArmOutDoubles.data() );
        lineCount++;

        if( log ) {
            std::vector< double > leftArmInDoubles( leftArmNumMotors );
            std::vector< double > rightArmInDoubles( rightArmNumMotors );
            std::vector< double > leftArmTimes( leftArmNumMotors ); // only use [0] of here
            //-- Not using rightArmTimes
            std::vector< double > leftArmTorques( leftArmNumMotors );
            std::vector< double > rightArmTorques( rightArmNumMotors );

            leftArmEncTimed->getEncodersTimed( leftArmInDoubles.data(), leftArmTimes.data() );
            leftArmTorque->getTorques( leftArmTorques.data() );
            rightArmEncTimed->getEncoders( rightArmInDoubles.data() );
            leftArmTorque->getTorques( rightArmTorques.data() );

            fprintf(logFilePtr,"%f ",leftArmTimes[0]);

            for(int i=0;i<leftArmNumMotors;i++)
                fprintf(logFilePtr,"%f ",leftArmOutDoubles[i]);

            for(int i=0;i<rightArmNumMotors;i++)
                fprintf(logFilePtr,"%f ",rightArmOutDoubles[i]);

            for(int i=0;i<leftArmNumMotors;i++)
                fprintf(logFilePtr,"%f ",leftArmInDoubles[i]);

            for(int i=0;i<rightArmNumMotors;i++)
                fprintf(logFilePtr,"%f ",rightArmInDoubles[i]);

            for(int i=0;i<leftArmNumMotors;i++)
                fprintf(logFilePtr,"%f ",leftArmTorques[i]);

            for(int i=0;i<rightArmNumMotors;i++)
                fprintf(logFilePtr,"%f ",rightArmTorques[i]);

            fprintf(logFilePtr,"\n");
        }

    }
    if( this->isStopping() ) return;
    CD_DEBUG("End parsing file.\n" );


    while( (! leftArmDone) &&  ( ! this->isStopping() )) {
        CD_DEBUG("Left arm not done!\n");
        leftArmPos->checkMotionDone(&leftArmDone);
        yarp::os::Time::delay(0.1);  //-- [s]
    }
    if( this->isStopping() ) return;
    CD_DEBUG("Left arm done!\n");

    while( (! rightArmDone) &&  ( ! this->isStopping() )) {
        CD_DEBUG("Right arm not done!\n");
        rightArmPos->checkMotionDone(&rightArmDone);
        yarp::os::Time::delay(0.1);  //-- [s]
    }
    if( this->isStopping() ) return;
    CD_DEBUG("Right arm done!\n");

    if( this->hold ) {
        while ( ! this->isStopping() ) {
            CD_INFO("\"--hold\" activated, press CTRL-C to release hold...\n");
            yarp::os::Time::delay(1);  //-- [s]
        }
    }

}

/************************************************************************/

void PlaybackThread::setFileName(const std::string &value)
{
    fileName = value;
    CD_SUCCESS("Configured: %s\n",fileName.c_str());
}

/************************************************************************/

}  // namespace teo
