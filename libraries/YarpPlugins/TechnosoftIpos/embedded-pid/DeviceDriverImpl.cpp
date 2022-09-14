// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::open(yarp::os::Searchable & config)
{
    return TechnosoftIposBase::open(config);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::close()
{
    if (ipBuffer)
    {
        delete ipBuffer;
        ipBuffer = nullptr;
    }

    return TechnosoftIposBase::close();
}

// -----------------------------------------------------------------------------
