/**
 *
 * @ingroup teo_body_lacqueyFetch
 * \defgroup teo_body_MbedFirmware Mbed-Firmware
 *
 * @brief Firmware that allows to recieve CAN messages to open and close the hand
 *
 * @section teo_body_firmware_legal Legal
 *
 * Copyright: 2018 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/people/r-de-santos">Raul de Santos Rico</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section lacqueyFetch_install Installation
 *
 * First you need to program the firmware to the MBED <br>
 * You can use the online compiler (https://os.mbed.com/compiler) to compile the firmware and stored it on the flash mbed memory<br>
 * Steps:
 * - Change the "id" of the device corresponding to the hand used (right-hand: ID 65, left-hand: ID 64) <br>
 * - Press "Compile" and save the .bin file on the flash mbed memory <br>

 * - If you want to check the correct functionality, you can connect the mbed to the serial USB and with a serial monitor program (configured with a baudrate of 115200), see the CAN messages information of the hand. <br>
 *
 *
 * @section lacqueyFetch_running Running (assuming correct installation)
 *
 * Running "launchManipulation" application, you can open or close the hand with this parameters (assuming you know how to run "launchManipulation"): <br>
 * set pos 6 1200   -> open hand  <br>
 * set pos 6 -1200  -> close hand <br>
 * set pos 6 0      -> loose hand <br>
 *
 **/

#include "mbed.h"
#include "Motor.h"

Serial pc(USBTX, USBRX);   // Serial connection
DigitalOut led1(LED1);     // received
CAN can(p30, p29);         // tx,rx
Motor m(p24, p26, p25);    // pwm, fwd, rev (PWM, REVERSE, FORDWARD)
const uint8_t id = 64;     // right-hand: ID 65, left-hand: ID 64

bool receive(float* value) {
    CANMessage msg;

    if (can.read(msg)) {
        if (id == (uint8_t)msg.id) {
            pc.printf("Message received (ID: %d) (size: %d)\n", (int8_t)msg.id, msg.len);

            if (msg.len == 4) {
                pc.printf("Message received %x %x %x %x\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
                *value = *((float*)msg.data);
                led1 = !led1;
                return true;
            }
        }
    }

    return false;
}

int main() {
    // set up the serial usb connection
    pc.baud(115200);          //-- configure PC baudrate
    pc.printf("main()\n");
    // set up the can bus
    can.frequency(1000000);   //-- 1Mbit/s (the same as the drivers configuration)
    can.reset();

    // message received
    float data;

    while (1) {
        if (receive(&data)) {
            pc.printf("value received: %5.2f\n", data);

            if (data >= -100.0f && data <= 100.0f) {
                m.speed(data / 100.0f);
            }
        }
    }
}
