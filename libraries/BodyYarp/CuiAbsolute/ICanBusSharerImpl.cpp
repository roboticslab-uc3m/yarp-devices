// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::setCanBusPtr(CanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::recoverFromError()
{

    return true;
}

// ------------------------------------------------------------------------------

bool teo::CuiAbsolute::startContinuousPublishing(uint8_t delay)
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

bool teo::CuiAbsolute::startPullPublishing()
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

bool teo::CuiAbsolute::stopPublishingMessages()
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
 * bool teo::CuiAbsolute::setZeroPosition()
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

bool teo::CuiAbsolute::interpretMessage( can_msg * message)
{

    //CD_DEBUG("Got absolute encoder value. %s\n",msgToStr(message).c_str());
    float got;
    memcpy(&got, message->data,4);
    //CD_SUCCESS("Got absolute encoder value, as a float: %f\n",got);

    /*
    if( (message->data[3]==0xc4) && ((message->id & 0x7F) == 108) ) // (message->data[3]==0xc4) && (message->id & 0x7F == 113)
    {
        CD_ERROR_NO_HEADER("Known PIC error (ID->%d): %f | %f | %s\n", message->id & 0x7F, encoder,got,msgToStr(message).c_str());
        return false;
    }
    */

    if( (message->data[3]==0xc4) ) // (message->data[3]==0xc4) && (message->id & 0x7F == 113)
    {
        CD_ERROR_NO_HEADER("Known PIC error (ID->%d): %f | %f | %s\n", message->id & 0x7F, encoder,got,msgToStr(message).c_str());
        return false;
    }

    encoderReady.wait();
     //canId = buffer.id  & 0x7F;                      // -- if it recive the message, it will get ID
    encoder = got * this->tr;

    if (encoder < -180.0)  // maybe a while?
        encoder += 360.0;

    if (encoder > 180.0)  // maybe a while?
        encoder -= 360.0;

    encoderTimestamp = message->ts;

    encoderReady.post();

    /*if(encoder < -1000)
    {
        if( (message->data[0]==0x3c)&&(message->data[1]==0x13)&&(message->data[2]==0xfe)&&(message->data[3]==0xc4) )
        {
            // known case
        }
        else if( (message->data[0]==0x37)&&(message->data[1]==0xb6)&&(message->data[2]==0xff)&&(message->data[3]==0xc4) )
        {
            // known case
        }
        else
        {
            CD_DEBUG("%f | %s\n",got,msgToStr(message).c_str());
        }
    }*/

        firstHasReached = true;
        return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
