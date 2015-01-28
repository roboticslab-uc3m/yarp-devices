// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

/************************************************************************/

bool PlaybackThread::threadInit() {

    leftLegDone = false;
    rightLegDone = false;

    return true;

}

/************************************************************************/
void PlaybackThread::run() {

    CD_DEBUG("Begin parsing file.\n" );
    std::string line;
    int lineCount = 1;
    while( getline( ifs, line) && ( ! this->isStopping() ) ) {

        yarp::os::Bottle lineBottle(line);  //-- yes, using a bottle to parse a string
        CD_DEBUG("[L:%d] string from bottle from string: %s\n", lineCount,lineBottle.toString().c_str() );
        if( leftLegNumMotors+rightLegNumMotors != lineBottle.size() )
            CD_ERROR("-------------SIZE!!!!!!!!!!!!!\n");

        std::vector< double > leftLegOutDoubles( leftLegNumMotors );
        for(int i=0;i<leftLegNumMotors;i++)
            leftLegOutDoubles[i] = lineBottle.get(i).asDouble();

        std::vector< double > rightLegOutDoubles( rightLegNumMotors );
        for(int i=0;i<rightLegNumMotors;i++)
            rightLegOutDoubles[i] = lineBottle.get(leftLegNumMotors+i).asDouble();

        leftLegPosDirect->setPositions( leftLegOutDoubles.data() );
        rightLegPosDirect->setPositions( rightLegOutDoubles.data() );
        lineCount++;
    }
    if( this->isStopping() ) return;
    CD_DEBUG("End parsing file.\n" );


    while( (! leftLegDone) &&  ( ! this->isStopping() )) {
        CD_INFO("Left Leg not done!\n");
        leftLegPos->checkMotionDone(&leftLegDone);
        yarp::os::Time::delay(0.1);  //-- [s]
    }
    if( this->isStopping() ) return;
    CD_INFO("Left Leg done!\n");

    while( (! rightLegDone) &&  ( ! this->isStopping() )) {
        CD_INFO("Right Leg not done!\n");
        rightLegPos->checkMotionDone(&rightLegDone);
        yarp::os::Time::delay(0.1);  //-- [s]
    }
    if( this->isStopping() ) return;
    CD_DEBUG("Right Leg done!\n");

}

/************************************************************************/
