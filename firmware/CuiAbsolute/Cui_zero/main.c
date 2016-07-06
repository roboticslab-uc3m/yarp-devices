// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_body_cuiAbsolute
 * \defgroup teo_body_cuiZero CUI-Zero
 *
 * @brief Firmware that allows to program the zero position of Cui Absolute encoder.
 *
 * @section teo_body_firmware_legal Legal
 *
 * Copyright: 2016 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/people/r-de-santos">Raul de Santos Rico</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section cuiAbsolute_install Installation
 *
 * You need to install MPLAB IDE v8.92 and MPLAB C Compiler for PIC18 MCUs. <br>
 * Put joint in zero position (for homing).
 * Open the proyect with MPLAB and open "main.c" file. Then, build all. <br>
 * Select programmer "MPLAB ICD 2", disconnect CAN-BUS connections of the PIC, connect the MPLAB ICD2 to PIC and program it. <br>
 * To continue with the process, you have to program PIC with PIC-Firmware <br>
 *
 * @section cuiAbsolute_modify Modify
 *
 * This file can be edited at
 * firmware/CuiAbsolute/Cui_zero/main.cpp
 *
 */

#include <p18F2580.h>
#include "ECAN.h"
#include "ECAN.c"
#include <delays.h>
#include "timers.h"
#include <spi.h>
#include <stdlib.h>
#include <stdio.h>


#pragma config OSC = HS, FCMEN = OFF, IESO = OFF
#pragma config PWRT = OFF, BOREN = OFF, BORV = 1
#pragma config WDT = OFF, WDTPS = 32768, DEBUG = ON
#pragma config PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON
#pragma config STVREN = OFF, LVP = OFF, XINST = OFF


void main(void)
{
        
        //Variables que almacenarán los datos a enviar:
        //unsigned long CAN_ID=497;   //ID de la aplicación para la comunicación CAN (ID 118 + 0x180 (PDO1))
        //BYTE val;
        //int Orden1[2], aux;
    	//float pos, grados, div=11.38;           //Se le da el valor de 11.38 a div por ser el resultado dividir (2^12)/360
        //BYTE OrdenLon1=1;
        BYTE x;
        //int i=0;
    	//BYTE send_flag, init_flag;
    	//long id, mask;


        SSPCON1bits.SSPEN=1;
        OpenSPI(SPI_FOSC_4, MODE_00, SMPEND);
        OSCCON=0b11110000;        //Primary oscillator.

         
        TRISCbits.TRISC2=0;     // CS, out
        TRISCbits.TRISC3=0;     // SCL, out
        TRISCbits.TRISC5=0;     // SDO, out
        TRISCbits.TRISC4=1;     // SDI  in
        LATCbits.LATC2=1;       //Disable del encoder al inicio

        //SE EJECUTA SOLO UNA VEZ

                LATCbits.LATC2=0;
                WriteSPI (0b01110000);      //Establece la posición actual como cero (comando 0x70)
                Delay10TCYx(3);                 //Wait 6us
                x=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                LATCbits.LATC2=1;

                LATCbits.LATC2=0;
                WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
                Delay10TCYx(3);                 //Wait 6us
                x=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                LATCbits.LATC2=1;
                
                while(x!=0b10000000) {                //Espera hasta que el encoder esté preparado para enviar algo (Respuesta 0x80)

                                LATCbits.LATC2=0;
                                WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
                                Delay10TCYx(3);                 //Wait 6us
                                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                                LATCbits.LATC2=1;

                                }

}
            
