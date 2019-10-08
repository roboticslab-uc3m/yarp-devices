// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setCanBusPtr(yarp::dev::ICanBus *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::recoverFromError()
{

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::startPushPublishing(uint8_t delay)
{
    // -- start message
    uint8_t msgData[2] = {0x01, delay};
    if( ! send(0 , 2, msgData) )   // -- primer campo "cob". En este campo se codifica un campo adicional al ID para indicar información adicional (0 - no se utiliza)
    {
        CD_ERROR("Could not send \"startPushPublishing\" (push mode) to Cui Absolute Encoder. %s\n", msgToStr(0, 2, msgData).c_str());
        return false;
    }
    CD_SUCCESS("Sent \"startPushPublishing\" to Cui Absolute Encoder. %s\n", msgToStr(0, 2, msgData).c_str());
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::stopPushPublishing()
{

    uint8_t msgData[2] = {0x02, 0x00}; // -- Para de publicar mensajes
    if( ! send(0, 2, msgData) )
    {
        CD_ERROR("Could not send \"stopPublishingMessages\" to Cui Absolute Encoder. %s\n", msgToStr(0, 2, msgData).c_str());
        return false;
    }
    CD_SUCCESS("Sent \"stopPublishingMessages\" to Cui Absolute Encoder. %s\n", msgToStr(0, 2, msgData).c_str());
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getCurrentPosition() // polling mode
{

    uint8_t msgData[2] = {0x03, 0x00}; // -- Envia mensaje en modo pull
    if( ! send(0, 2, msgData) )   // -- utilizaremos la funcion "send" por ser una funcion publica en vez de la funcion privada sendRaw
    {
        CD_ERROR("Could not send \"getCurrentPosition\" to Cui Absolute Encoder. %s\n", msgToStr(0, 2, msgData).c_str());
        return false;
    }
    CD_SUCCESS("Sent \"getCurrentPosition\" to Cui Absolute Encoder. %s\n", msgToStr(0, 2, msgData).c_str());
    return true;
}

// ------------------------------------------------------------------------------
/*** Esta función actualmente no se utiliza debido a que implica cierta peligrosidad...
 *
 * bool roboticslab::CuiAbsolute::setZeroPosition()
 * {
 *
 *   uint8_t msgData[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // -- Homing
 *   if( ! send(0, 8, msgData) )
 *   {
 *       CD_ERROR("Could not send \"setZeroPosition\" to Cui Absolute Encoder. %s\n");
 *       return false;
 *   }
 *   CD_SUCCESS("Send: \"setZeroPosition\" to Cui Absolute Encoder. %s\n");
 *   return true;
 * }
*/
// ------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::interpretMessage(const yarp::dev::CanMessage & message)
{

    //CD_DEBUG("Got absolute encoder value. %s\n",msgToStr(message).c_str());
    float got;
    memcpy(&got, message.getData(),4);

    if( (message.getData()[3]==0xc4) ) // If you want to print a specific Cui known error: Ex 113: (message->data[3]==0xc4) && (message->id & 0x7F == 113)
    {
        CD_ERROR_NO_HEADER("Known PIC error: %f | %f | %s\n", encoder,got,msgToStr(message).c_str());
        return false;
    }

    encoderReady.wait();
    encoder = got * this->tr;

    if (encoder < -180.0)  // maybe a while?
        encoder += 360.0;

    if (encoder > 180.0)  // maybe a while?
        encoder -= 360.0;

    encoderTimestamp = yarp::os::Time::now();
    encoderReady.post();
    firstHasReached = true;
    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
