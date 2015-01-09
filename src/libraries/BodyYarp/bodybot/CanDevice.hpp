// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_DEVICE__
#define __CAN_DEVICE__

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>  // just for ::write
#include <err.h>
#include <errno.h>
#include <assert.h>
#include <string>

#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

#include "hico_api.h"

#include "ColorDebug.hpp"

#define DELAY 0.001  // Was DELAY2. Required when using same driver.

class CanDevice {

    public:

        /** Initialize the CAN device.
         * @param device is the device path, such as "/dev/can0".
         * @param bitrate is the bitrate, such as BITRATE_100k.
         * @return true/false on success/failure.
         */
        bool init(const std::string device, const int bitrate);

        /** Close the CAN device.
         */
        bool close();


        /**
         * Write message to the CAN buffer.
         * @param id Message's COB-id
         * @param len Data field length
         * @param msgData Data to send
         * @return true/false on success/failure.
         */
        bool sendRaw(uint32_t id, uint16_t len, uint8_t * msgData);

        /** Read data.
         * @return Number on got, 0 on timeout, and errno on fail. */
        int read_timeout(struct can_msg *buf, unsigned int timeout);

        void show_er( can_msg * message);

    protected:

        /** CAN file descriptor */
        int fileDescriptor;

        yarp::os::Semaphore canBusReady;

};
 
#endif  // __CAN_DEVICE__

