// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

/************************************************************************/

bool PlaybackThread::threadInit() {

    leftArmDone = false;

    return true;

}

/************************************************************************/
void PlaybackThread::run() {

    CD_DEBUG("Begin parsing file.\n" );
    std::string line;
    while( getline( ifs, line) && ( ! this->isStopping() ) ) {

        yarp::os::Bottle lineBottle(line);  //-- yes, using a bottle to parse a string
        CD_DEBUG("string from bottle from string: %s\n", lineBottle.toString().c_str() );
        if( leftArmNumMotors != lineBottle.size() )
            if( leftArmNumMotors+1 != lineBottle.size() )
                CD_ERROR("-------------SIZE!!!!!!!!!!!!!\n");

        std::vector< double > leftArmOutDoubles( leftArmNumMotors );
        for(int i=0;i<leftArmNumMotors;i++)
            leftArmOutDoubles[i] = lineBottle.get(i).asDouble();

        leftArmPosDirect->setPositions( leftArmOutDoubles.size(), NULL, leftArmOutDoubles.data() );

        leftGripperPos->positionMove(0,lineBottle.get(leftArmNumMotors).asDouble());
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

}


/************************************************************************/
