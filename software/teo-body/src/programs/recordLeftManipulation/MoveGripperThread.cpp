// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordLeftManipulation.hpp"

/************************************************************************/
bool MoveGripperThread::setLeftOpenChar(const char& openLeftChar) {
    this->openLeftChar = openLeftChar;
}

/************************************************************************/
bool MoveGripperThread::setLeftCloseChar(const char& closeLeftChar) {
    this->closeLeftChar = closeLeftChar;
}

/************************************************************************/
bool MoveGripperThread::threadInit() {

    w = initscr();
    //raw();  //-- raw deactivates signals and gets raw text. this we do NOT want.
    cbreak();  //-- cbreak allow ctrl-c to reach normal handler. this we DO want.
    nodelay(w, TRUE);

    return true;
}

/************************************************************************/
void MoveGripperThread::run() {
    noecho();
    while( ! this->isStopping() ) {
        char c = getch();
        if(c == openLeftChar) {
            //CD_WARNING("openLeftChar\n");
            leftGripperPos->positionMove(0,1200);
        } else if(c == closeLeftChar) {
            //CD_WARNING("closeLeftChar");
            leftGripperPos->positionMove(0,-1200);
        }
    }
    endwin();

}

/************************************************************************/
