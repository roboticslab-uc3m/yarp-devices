// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

#include <limits>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// ------------------ Thread Related -----------------------------------------

void DumpCanBus::run()
{
    CD_INFO("Started DumpCanBus reading thread run.\n");

    while (!yarp::os::RFModule::isStopping())
    {
        //-- Lend CPU time to write threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(std::numeric_limits<double>::min());

        unsigned int read;

        //-- Return immediately if there is nothing to be read (non-blocking call), return false on errors.
        bool ok = iCanBus->canRead(canInputBuffer, 1, &read);

        //-- All debugging messages should be contained in canRead, so just loop again.
        if (!ok || read == 0) continue;

        const yarp::dev::CanMessage & msg = canInputBuffer[0];
        CD_INFO("Read CAN message: %s\n", CanUtils::msgToStr(msg.getId(), msg.getLen(), msg.getData()).c_str());
    }

    CD_INFO("Stopping DumpCanBus reading thread run.\n");
}

// -----------------------------------------------------------------------------
