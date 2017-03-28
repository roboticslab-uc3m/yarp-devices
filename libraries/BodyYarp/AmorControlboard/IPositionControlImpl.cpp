// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IPositionControl Related --------------------------------

bool roboticslab::AmorControlboard::getAxes(int *ax) {
    CD_DEBUG("\n");
    *ax = axes;
    CD_DEBUG("Reporting %d axes are present\n", *ax);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setPositionMode() {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(int j, double ref) {  // encExposed = ref;
    CD_DEBUG("\n");
    if ((unsigned int)j>axes) {
        fprintf(stderr,"[FakeControlboardOR] error: axis index more than axes.\n");
        return false;
    }

    AMOR_VECTOR7 refs;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++) {
        if (i == j)
            refs[i] = ref * 180 / 3.14159;
        else
            refs[i] = 0;
    }

    return amor_set_positions(handle, refs) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(const double *refs) {  // encExposed = refs;
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(int j, double delta) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const double *deltas) {  // encExposed = deltas + encExposed
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(int j, bool *flag) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(bool *flag) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeed(int j, double sp) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const double *spds) {
    CD_DEBUG("\n");
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAcceleration(int j, double acc) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const double *accs) {
    CD_DEBUG("\n");
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeed(int j, double *ref) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(double *spds) {
    CD_DEBUG("\n");
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAcceleration(int j, double *acc) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(double *accs) {
    CD_DEBUG("\n");
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(int j) {
    CD_DEBUG("NOTHING TO DO\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop() {
    CD_DEBUG("\n");
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= stop(i);
    return ok;
}

// -----------------------------------------------------------------------------

