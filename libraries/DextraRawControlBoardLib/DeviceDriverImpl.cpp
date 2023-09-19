// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::open(yarp::os::Searchable & config)
{
    axisPrefix = config.check("axisPrefix", yarp::os::Value(""), "common refix for all axis names").asString();
    return true;
}

// -----------------------------------------------------------------------------
