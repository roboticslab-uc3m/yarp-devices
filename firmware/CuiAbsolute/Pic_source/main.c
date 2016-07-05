// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_body_cuiAbsolute
 * \defgroup teo_body_picFirmware PIC-Firmware
 *
 * @brief Firmware that allows communication with the PIC and the publication of position messages. 
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
 * Open the proyect with MPLAB, open "main.c" file and change the value of "canId" variable (You can see <a class="el" href="http://robots.uc3m.es/index.php/CuiAbsolute_Documentation">correspondence</a>) <br>
 * Also, to increase the time between sending messages,  you can modify "sendDelay" value too (see code documentation in main.c). 
 * Then, build all. <br>
 * Finally, select programmer "MPLAB ICD 2", disconnect CAN-BUS connections of the PIC, connect and program it. <br> 
 *
 * @section cuiAbsolute_running Running (assuming correct installation)
 *
 *
 * To test Cui Aboslute encoders are working properly, you need to start the program named "testCuiAbsolute" and see that the test passed correctly. <br>
 * Previously, you have to manually change the ID of Cui that you want to test in the code (example: #define CAN_ID 124), compile and reinstall. <br>
 * The testCuiAbsolute application can be edited at:
 * tests/testCuiAbsolute.cpp
\verbatim
[on terminal] testCuiAbsolute
\endverbatim
 * And then, in the end of the test you should see this:
\verbatim
[----------] 3 tests from CuiAbsoluteTest (5082 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 1 test case ran. (5082 ms total)
[  PASSED  ] 3 tests.
\endverbatim
 *
 * @section cuiAbsolute_modify Modify
 *
 * This file can be edited at
 * firmware/CuiAbsolute/Pic_source/main.cpp
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


/***** Variables configurables (CAN_ID & SEND_DELAY) *******************************************************/

// CAN ID (Ver correspondecia en http://robots.uc3m.es/index.php/CuiAbsolute_Documentation)
unsigned long canId = 508; //508 (codo 124) 492 (pierna izquierda 108)

/* SEND DELAY (Valor que utilizará Delay10TCYx en el envío. Valor recomendado de 1 a 100)
 * El byte 3 (data[2]) que recibirá el PIC (valor comprendido entre [0-255]) se multiplicará por el tiempo que
 * tarde en ejecutar el delay marcado por la función Delay10TCYx(sendDelay)
 * A tener en cuenta:
	- La velocidad de ejecución de cada ciclo de instrucción son 0.8 microsegundos
        - Delay10TCYx(i) -> 10.Tcy.i genera una demora de 10 ciclos de instrucciones * i . Por tanto Delay10TCYc(1) equivale a 8 microsegundos (10 ciclos de reloj) */
BYTE sendDelay = 1;
/***********************************************************************************************************/

// -- Inicialización de variables para el envío
int Orden1[2], aux;
float pos, grados, div=11.38; //Se le da el valor de 11.38 a div por ser el resultado dividir (2^12)/360
BYTE OrdenLon1=1;
BYTE x, y;
int i=0;
ECAN_TX_MSG_FLAGS txFlags = ECAN_TX_PRIORITY_3 & ECAN_TX_STD_FRAME & ECAN_TX_NO_RTR_FRAME;
int stop_flag=0; // -- flag para saber cuando se ha recibido un stop

// -- Variables que almacenarán los datos a recibir:
unsigned long picId;
BYTE data[8];
BYTE dataLen;
ECAN_RX_MSG_FLAGS rxflags;

// -- Prototipos de funciones
void send(void);
void cleanData(void);



// -- Funcion principal
void main(void)
{
    SSPCON1bits.SSPEN=1;
    OpenSPI(SPI_FOSC_4, MODE_00, SMPEND);

    TRISCbits.TRISC2=0;     // CS, out
    TRISCbits.TRISC3=0;     // SCL, out
    TRISCbits.TRISC5=0;     // SDO, out
    TRISCbits.TRISC4=1;     // SDI  in
    LATCbits.LATC2=1;       //Disable del encoder al inicio

    OSCCON=0b11110000;       //Primary oscillator.

    // Manual (page 15): ECAN.def file must be set up correctly.
    ECANInitialize();

    // Manual (page 38): Must be in Config mode to change many of settings.
    ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

    /* Manual (page 1):
    ECAN provides three modes of operation – Mode 0, Mode 1 and Mode 2. Mode 0 is fully backward compat-
    ible with the legacy CAN module. Applications developed for the legacy CAN module would continue to
    work without any change using ECAN. Mode 1 is the Enhanced Legacy mode with increased buffers and fil-
    ters. Mode 2 has the same resources as Mode 1, but with a hardware managed receive FIFO. Given its fea-
    tures and flexibility, ECAN would prove useful to many CAN-based applications. */
    ECANSetFunctionalMode(ECAN_MODE_0);

    // Manual (page 41): Enable double-buffering
    ECANSetRXB0DblBuffer(ECAN_DBL_BUFFER_MODE_DISABLE);

    /* Manual (page 44):
    This macro sets the value for a mask register. There are a total of two macros, one for each mask.
    For example, for mask RXM0, use ECANSetRXM0Value, for RXM1, use ECANSetRXM1Value and so on
    ECAN_MSG_STD Standard type. 11-bit of value will be used.
    ECAN_MSG_XTD Extended type. 29-bit of value will be used.*/
    ECANSetRXM0Value(-1, ECAN_MSG_STD);

    // Manual (page 42): RXB0 will receive all valid messages
    ECANSetRxBnRxMode(RXB0, ECAN_RECEIVE_ALL_VALID);

    // Manual (page 36): This macro sets the CAN bus Wake-up Filter mode. Specifies that the low-pass filter be disabled
    ECANSetFilterMode(ECAN_FILTER_MODE_DISABLE);

    // Manual (page 38): Return to Normal mode to communicate.
    ECANSetOperationMode(ECAN_OP_MODE_NORMAL);


    /* Mensaje compuesto por 3 bytes
    -- data[0]: (Byte 1)
    	* start (0x01)
    	* stop  (0x02)
    -- data[1]: (Byte 2)
    	* mode1 (0x01): publicación permanente		(necesita indicar velocidad de publicación)
    	* mode2 (0x02): publicación por petición	(no necesita indicar velocidad de publicación)
    -- data[2]: (Byte 3)
    	* (0 - 255)
    	A tener en cuenta:
    		- La frecuencia de oscilación se encuentra definida como Fosc = 5Mhz (20/4)
    		- La velocidad de ejecución de cada ciclo de instrucción son 0.8 microsegundos
    		- Delay10TCYx(i) -> 10.Tcy.i genera una demora de 10 ciclos de instrucciones * i . Por tanto Delay10TCYc(1) equivale a 8 microsegundos (10 ciclos de reloj)
    */

    while(1)
    {
        // --------------- Recibo!!! ------------------
        if(ECANReceiveMessage(&picId, data, &dataLen, &rxflags))
        {
            // ----------- Checkeamos ID Driver -----------
            if(picId == canId-384)  		// -- (canId = picId + 0x180)
            {
                if(data[0]==0x01 && data[1]==0x01)    // -- comienza publicación (start) modo permanente : if(data[0]==0x01 && data[1]==0x01)
                {
                    // -- mientras no mande un Stop, sigue publicando
                    while(!stop_flag)
                    {
                        send();	// -- envia
                        for( i=0; (i<= data[2]) && (!stop_flag) ; i++ )  // data[2] recibirá un valor comprendido en [0 - 255]
                        {
                            Delay10TCYx(sendDelay);
                            ECANReceiveMessage(&picId, data, &dataLen, &rxflags);
                            if((data[0]==0x02 && data[1]==0x01) && (picId == canId-384)) stop_flag=1;
                        }
                    }
                    cleanData(); // -- Se para, limpia las variables
                    stop_flag=0;
                }
                if((data[0]==0x01)&&(data[1]==0x02))  	// -- publica por pulling (petición)
                {
                    send();		 // Manda una único mensaje
                    cleanData(); // Limpia las variables
                }
            } // if(picId == canId-384)
        } // if(ECANReceiveMessage(&picId, data, &dataLen, &flags))
    } // while
} // main


// -- Funcion de envio de mensajes de posicion del Cui:
void send()
{
    LATCbits.LATC2=0;
    WriteSPI (0b00010000);      //Solicitación de posición (comando 0x10)
    Delay10TCYx(3);             //Wait 6us
    y=SSPBUF;                   //Recoge los datos del SSPBUF que provienen del encoder (MISO)
    LATCbits.LATC2=1;

    LATCbits.LATC2=0;
    WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
    Delay10TCYx(3);             //Wait 6us
    y=SSPBUF;                   //Recoge los datos del SSPBUF que provienen del encoder (MISO)
    LATCbits.LATC2=1;

    while(y!=0b00010000)        //Espera hasta que el encoder esté preparado para enviar algo (Respuesta 0x10)
    {
        LATCbits.LATC2=0;
        WriteSPI (0b00000000);  //Espera a que el encoder esté listo (comando 0x00)
        Delay10TCYx(3); 	//Wait 6us
        y=SSPBUF;		//Recoge los datos del SSPBUF que provienen del encoder (MISO)
        LATCbits.LATC2=1;
    }

    LATCbits.LATC2=0;
    WriteSPI(0b00000000);	//Espera para captar la parte alta de la posición
    Delay10TCYx(3);		//Wait 6us
    y=SSPBUF;			//Recoge los datos del SSPBUF que provienen del encoder (MISO)
    Orden1[0]=y;		//Se almacena en el primer elemento del array que se enviará por CAN

    LATCbits.LATC2=1;
    LATCbits.LATC2=0;
    WriteSPI(0b00000000);       //Espera para captar la parte baja del mensage
    Delay10TCYx(3);             //Wait 6us
    y=SSPBUF;                   //Recoge los datos del SSPBUF que provienen del encoder (MISO)
    Orden1[1]=y;                //Se almacena en el segundo elemento del array que se enviará por CAN

    LATCbits.LATC2=1;

    aux=(Orden1[0]<<8)+Orden1[1];  //Se rota el byte alto de la posición 8 bits (2^8=256) y se suma al bajo

    grados = aux / div;            //Se divide para obtener la realacción en grados

    x=0;
    while( !x )
    {
        x=ECANSendMessage(canId, &grados, sizeof(grados), txFlags);
    }
}

// -- Funcion para limpiar los datos recibidos por el PIC
void cleanData()
{
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
}
