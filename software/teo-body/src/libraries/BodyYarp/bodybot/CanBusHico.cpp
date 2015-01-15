// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

// -----------------------------------------------------------------------------

bool CanBusHico::init(const std::string devicePath, const int bitrate) {

    //-- Open the CAN device for reading and writing.
    fileDescriptor = open(devicePath.c_str(), O_RDWR);
    if(fileDescriptor<0)
    {
        CD_ERROR("Could not open CAN device of path: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("Opened CAN device of path: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Set the CAN bitrate.
    if( ioctl(fileDescriptor,IOC_SET_BITRATE,&bitrate) != 0)
    {
        CD_ERROR("Could not set bitrate on CAN device: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("Bitrate set on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Start the CAN device.
    if( ioctl(fileDescriptor,IOC_START) != 0)
    {
        CD_ERROR("IOC_START failed on CAN device: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("IOC_START ok on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::close() {
    //release semaphore?
    ::close(fileDescriptor);
}

// -----------------------------------------------------------------------------

bool CanBusHico::sendRaw(uint32_t id, uint16_t len, uint8_t * msgData) {

     struct can_msg msg;
     memset(&msg,0,sizeof(struct can_msg));
     msg.ff = FF_NORMAL;
     msg.id = id;
     msg.dlc = len;
     memcpy(msg.data, msgData, len*sizeof(uint8_t));

     canBusReady.wait();
     if ( write(fileDescriptor,&msg,sizeof(struct can_msg)) == -1)
     {
         canBusReady.post();
         CD_ERROR("%s.\n", strerror(errno))
         return false;
     }
     canBusReady.post();

     return true;
}

// -----------------------------------------------------------------------------

int CanBusHico::read_timeout(struct can_msg *buf, unsigned int timeout) {

    fd_set fds;
    struct timeval tv;
    int sec,ret;
    FD_ZERO(&fds);

    sec=timeout/1000;
    tv.tv_sec=sec;
    tv.tv_usec=(timeout-(sec*1000))*1000;

    FD_SET(fileDescriptor,&fds);

    //-- select() returns the number of ready descriptors, or -1 for errors.
    ret=select(fileDescriptor+1,&fds,0,0,&tv);
    if(ret==0)
    {
        CD_DEBUG("select() timeout.\n");
        return 0;  // Return 0 on select timeout.
    }
    else if (ret<0)
    {
        CD_ERROR("select() error: %s.\n", strerror(errno));
        return ret;   // Return <0 on select error.
    }

    assert(FD_ISSET(fileDescriptor,&fds));

    canBusReady.wait();
    //-- read() returns the number read, -1 for errors or 0 for EOF.
    ret=read(fileDescriptor,buf,sizeof(struct can_msg));
    canBusReady.post();

    if (ret<0)
    {
        CD_ERROR("read() error: %s.\n", strerror(errno));
    }

    return ret;  //-- If gets to here, return whatever read() returned.
}

// -----------------------------------------------------------------------------

void CanBusHico::show_er( can_msg * message){

    if (message->id>=0x80 && message->id<0x100){

        if (message->data[0] == 0x01 && message->data[1] == 0xFF)
            return; // PVT control message

        CD_ERROR("Emergency message in id: %d. ", message->id & 0x7F );

        printf("%X %X : ",message->data[1],message->data[0]);
        switch (message->data[1]){
            case 0:
                switch(message->data[0]){
                    case 0: CD_ERROR("Error Reset\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x10:
                switch(message->data[0]){
                    case 0: CD_ERROR("Generic error\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x23:
                switch(message->data[0]){
                    case 0x10: CD_ERROR("Continuous over-current\n");
                    break;
                    case 0x40: CD_ERROR("Short-circuit\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x32:
                switch(message->data[0]){
                    case 0x10: CD_ERROR("DC-link over-voltage\n");
                    break;
                    case 0x20: CD_ERROR("DC-link under-voltage\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x42:
                switch(message->data[0]){
                    case 0x80: CD_ERROR("Over temperature motor\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x43:
                switch(message->data[0]){
                    case 0x10: CD_ERROR("Over temperature drive.\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x54:
                switch(message->data[0]){
                    case 0x41: CD_ERROR("Driver disabled due to enable input.\n");
                    break;
                    case 0x42: CD_ERROR("Negative limit switch active.\n");
                    break;
                    case 0x43: CD_ERROR("Positive limit switch active.\n");
                    break;
                    default: CD_ERROR("Unknown error.\n");
                };
            break;
            case 0x61:
                switch(message->data[0]){
                    case 0x00: CD_ERROR("Invalid stup data.\n");
                    break;
                    default: CD_ERROR("Unknown error.\n");
                };
            break;
            case 0x75:
                switch(message->data[0]){
                    case 0x00: CD_ERROR("Communication error.\n");
                    break;
                    default: CD_ERROR("Unknown error.\n");
                };
            break;
            case 0x81:
                switch(message->data[0]){
                    case 0x10: CD_ERROR("CAN overrun (message lost).\n");
                    break;
                    case 0x30: CD_ERROR("Life guard error or heartbeat error.\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x83:
                switch(message->data[0]){
                    case 0x31: CD_ERROR("I2t protection triggered.\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;        if (message->data[0] == 0x01 && message->data[1] == 0xFF)
                    return; // PVT control message
            case 0x85:
                switch(message->data[0]){
                    case 0x80: CD_ERROR("Position wraparound / Hal sensor missing.\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x86:
                switch(message->data[0]){
                    case 0x11: CD_ERROR("Control error / Following error.\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0x90:
                switch(message->data[0]){
                    case 0x00: CD_ERROR("Command error\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            case 0xFF:
                switch(message->data[0]){
                    case 0x01: CD_ERROR("Generic interpolated position mode error ( PVT / PT error.\n");
                    break;
                    case 0x02: CD_ERROR("Change set acknowledge bit wrong value.\n");
                    break;
                    case 0x03: CD_ERROR("Specified homing method not available.\n");
                    break;
                    case 0x04: CD_ERROR("A wrong mode is set in object 6060h, modes_of_operation.\n");
                    break;
                    case 0x05: CD_ERROR("Specified digital I/O line not available.\n");
                    break;
                    default: CD_ERROR("Unknown error\n");
                };
            break;
            default: CD_ERROR("Unknown error\n");
        }

    }
}

// -----------------------------------------------------------------------------

