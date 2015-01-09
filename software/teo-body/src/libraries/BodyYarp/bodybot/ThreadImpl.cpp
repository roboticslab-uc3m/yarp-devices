// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ RateThread Related -----------------------------------------

void teo::BodyBot::run() {
    while( ! this->isStopping() ) {

        struct can_msg buffer;

        int ret = canDevice.read_timeout(&buffer,1);

        int canId = buffer.id  & 0x7F;
        //-- CD_INFO("Read from id: %d\n", canId);

        if( ret == 0 )
        {
            //-- CD_WARNING("Read timeout.\n");  //--way too verbose
            continue;
        }
        else if ( ret < 0 )
        {
            if((canId != 0)&&(canId != 128))
            {
                CD_WARNING("Read ret < 0.\n");  //--way too verbose
                display(buffer);
                canDevice.show_er(&buffer);
            }
        }
        else
        {
            if( (buffer.data[1]==0x64) && (buffer.data[2]==0x60) )
            {
                //-- CD_INFO("Is encoder value (response to petition).\n");
                int got;
                memcpy(&got, buffer.data+4,4);
                drivers[ idxFromCanId[canId] ]->setEncoder( got / ( 11.11112 * (drivers[ idxFromCanId[canId] ]->getTr()) ) );
            }
            else if( (buffer.data[0]==0x01)&&(buffer.data[1]==0xFF)&&(buffer.data[2]==0x01)&&(buffer.data[4]==0x20) )
            {
                drivers[ idxFromCanId[canId] ]->ptBuffer.wait();
                CD_WARNING("pt buffer full (%d)!\n",canId);
                display(buffer);
                canDevice.show_er(&buffer);
            }
            else if( (buffer.data[0]==0x01)&&(buffer.data[1]==0xFF)&&(buffer.data[2]==0x01)&&(buffer.data[4]==0x00) )
            {
                drivers[ idxFromCanId[canId] ]->ptBuffer.post();
                CD_WARNING("pt buffer empty (%d).\n",canId);
                display(buffer);
                canDevice.show_er(&buffer);
            }
            else if( (buffer.data[0]==0x01)&&(buffer.data[1]==0xFF)&&(buffer.data[2]==0x01)&&(buffer.data[3]==0x08) )
            {
                CD_WARNING("pt buffer message (don't know what it means) at canId %d.\n",canId);
                display(buffer);
                canDevice.show_er(&buffer);
            }
            else if( (buffer.data[0]==0x37)&&(buffer.data[1]==0x96) )
            {
                CD_WARNING("ended movement (canId %d).\n",canId);
                drivers[ idxFromCanId[canId] ]->ptMovementDone = true;
                display(buffer);
                canDevice.show_er(&buffer);
            }
            else if( (buffer.data[1]==0x41)&&(buffer.data[2]=0x60) )
            {
                if(buffer.data[5] & 4) {
                    drivers[ idxFromCanId[canId] ]->targetReached = true;
                    CD_DEBUG("\t-Target reached (%d).\n",canId);
                }
                else
                {
                    drivers[ idxFromCanId[canId] ]->targetReached = false;
                    CD_DEBUG("\t-Target NOT reached (%d).\n",canId);
                }
            }
            else
            {
                //CD_DEBUG("Read ret > 0 (%d) canId: %d.\n",ret,canId);  //--way too verbose
                display(buffer);
                canDevice.show_er(&buffer);
            }

        }
    }
    CD_INFO("Last thread run.\n");
    return;
}

// -----------------------------------------------------------------------------


