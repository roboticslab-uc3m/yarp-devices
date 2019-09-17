// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cstring>

#include <atomic>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

namespace
{
    std::atomic_bool hasArrived(false);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::initialize()
{
    CD_INFO("Sending \"Start Continuous Publishing\" message to Cui Absolute (PIC ID: %d)\n", canId);

    if (!startContinuousPublishing(0))
    {
        return false;
    }

    yarp::os::Time::delay(0.2);

    if (!hasArrived)
    {
        const double start = yarp::os::Time::now();

        while (!hasArrived)
        {
            if (yarp::os::Time::now() - start > cuiTimeout)
            {
                CD_ERROR("Time out passed and CuiAbsolute ID (%d) doesn't respond\n", canId);
                return false;
            }

            yarp::os::Time::delay(0.1);
        }
    }

    double value;
    getEncoderRaw(0, &value);

    CD_INFO("Absolute encoder value: %f\n", value);
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::start()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::readyToSwitchOn()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::switchOn()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::enable()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::recoverFromError()
{
    return true;
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::interpretMessage(const yarp::dev::CanMessage & message)
{
    if (message.getData()[3] == 0xc4)
    {
        CD_ERROR_NO_HEADER("Known PIC error. %s\n", CanUtils::msgToStr(message).c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex);
        std::memcpy(&encoder, message.getData(), 4);

        if (reverse)
        {
            encoder = -encoder;
        }

        if (encoder < -180.0)
        {
            encoder += 360.0;
        }
        else if (encoder > 180.0)
        {
            encoder -= 360.0;
        }

        encoderTimestamp = yarp::os::Time::now();
    }

    hasArrived = true;
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------
