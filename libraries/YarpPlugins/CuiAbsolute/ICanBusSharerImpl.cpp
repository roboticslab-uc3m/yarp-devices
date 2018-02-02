// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setCanBusPtr(ICanBusHico *canDevicePtr)
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

bool roboticslab::CuiAbsolute::startContinuousPublishing(uint8_t delay)
{
    // -- start message
    uint8_t msgData[8] = {0x01, 0x01, delay, 0x00, 0x00, 0x00, 0x00, 0x00};
    if( ! send(0 , 8, msgData) )   // -- primer campo "cob" lo dejamos a 0 (este campo resulta desconocido para nosotros)
    {
        CD_ERROR("Could not send \"startContinuousPublishing\" to Cui Absolute Encoder.\n");
        return false;
    }
    CD_INFO("Send: \"startContinuousPublishing\" to Cui Absolute Encoder.\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::startPullPublishing()
{

    uint8_t msgData[8] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // -- Comienza a publicar mensajes en modo pulling (modo 2) sin delay
    if( ! send(0, 8, msgData) )   // -- utilizaremos la funcion "send" por ser una funcion publica en vez de la funcion privada sendRaw
    {
        CD_ERROR("Could not send \"startPullPublishing\" to Cui Absolute Encoder.\n");
        return false;
    }
    CD_INFO("Send: \"startPullPublishing\" to Cui Absolute Encoder. \n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::stopPublishingMessages()
{

    uint8_t msgData[8] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // -- Para de publicar mensajes
    if( ! send(0, 8, msgData) )
    {
        CD_ERROR("Could not send \"stopPublishingMessages\" to Cui Absolute Encoder. %s\n");
        return false;
    }
    CD_INFO("Send: \"stopPublishingMessages\" to Cui Absolute Encoder. %s\n");
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

bool roboticslab::CuiAbsolute::interpretMessage( can_msg * message)
{

    //CD_DEBUG("Got absolute encoder value. %s\n",msgToStr(message).c_str());
    float got;
    memcpy(&got, message->data,4);

    if( (message->data[3]==0xc4) ) // If you want to print a specific Cui known error: Ex 113: (message->data[3]==0xc4) && (message->id & 0x7F == 113)
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

    encoderTimestamp = message->ts;
    encoderReady.post();
    firstHasReached = true;
    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
