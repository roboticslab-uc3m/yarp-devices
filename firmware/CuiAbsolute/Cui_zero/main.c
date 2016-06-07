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
        unsigned long CAN_ID=497;   //ID de la aplicación para la comunicación CAN (ID 118 + 0x180 (PDO1))
        BYTE val;
        int Orden1[2], aux;
    	float pos, grados, div=11.38;           //Se le da el valor de 11.38 a div por ser el resultado dividir (2^12)/360
        BYTE OrdenLon1=1;
        BYTE x;
        BYTE y;
		int i=0;
    	BYTE send_flag, init_flag;
    	long id, mask;

            
        /*
        init_flag = CAN_CONFIG_SAMPLE_THRICE & CAN_CONFIG_PHSEG2_PRG_ON & CAN_CONFIG_STD_MSG & CAN_CONFIG_DBL_BUFFER_ON & CAN_CONFIG_VALID_XTD_MSG & CAN_CONFIG_LINE_FILTER_OFF;
        send_flag = CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME;
        */

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
                val=WriteSPI (0b01110000);      //Establece la posición actual como cero (comando 0x70)
                Delay10TCYx(3);                 //Wait 6us
                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                LATCbits.LATC2=1;

                LATCbits.LATC2=0;
                val=WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
                Delay10TCYx(3);                 //Wait 6us
                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                LATCbits.LATC2=1;
                
                while(y!=0b10000000) {                //Espera hasta que el encoder esté preparado para enviar algo (Respuesta 0x80)

                                LATCbits.LATC2=0;
                                val=WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
                                Delay10TCYx(3);                 //Wait 6us
                                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                                LATCbits.LATC2=1;

                                }

}
            