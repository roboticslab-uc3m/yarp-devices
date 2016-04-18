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

		// Dirección del encoder: CAN_ID= ....

        unsigned long CAN_ID=486;   //ID de la aplicación para la comunicación CAN (ID 118 + 0x180 (PDO1))
        BYTE val;
        int Orden1[2], aux;
    	float pos, grados, div=11.38;           //Se le da el valor de 11.38 a div por ser el resultado dividir (2^12)/360
        BYTE OrdenLon1=1;
        BYTE x;
        BYTE y;
		int i=0;
    	BYTE send_flag, init_flag;
    	long id, mask;

        //BYTE hi_pos;
        //BYTE lo_pos;
        //ECAN_RX_MSG_FLAGS OrdenFlags1=0;

        ECAN_TX_MSG_FLAGS OrdenFlags1;

		// En Brazo Derecho: Con Prioridad "3" (mínima) y sin espera parece que funciona.
		// En Brazo Derecho: Con Prioridad "0" (máxima) y sin espera parece que no funciona.

        OrdenFlags1=ECAN_TX_PRIORITY_3 & ECAN_TX_STD_FRAME & ECAN_TX_NO_RTR_FRAME;
    
        /*
        init_flag = CAN_CONFIG_SAMPLE_THRICE & CAN_CONFIG_PHSEG2_PRG_ON & CAN_CONFIG_STD_MSG & CAN_CONFIG_DBL_BUFFER_ON & CAN_CONFIG_VALID_XTD_MSG & CAN_CONFIG_LINE_FILTER_OFF;
        send_flag = CAN_TX_PRIORITY_0 & CAN_TX_STD_FRAME & CAN_TX_NO_RTR_FRAME;
        */

        SSPCON1bits.SSPEN=1;
        OpenSPI(SPI_FOSC_4, MODE_00, SMPEND);
        OSCCON=0b11110000;        //Primary oscillator.


        ECANInitialize();
        ECANSetOperationMode(ECAN_OP_MODE_CONFIG);
    
        ECANSetRXB0DblBuffer(ECAN_DBL_BUFFER_MODE_ENABLE);
        ECANSetRXM0Value(-1, ECAN_MSG_STD);
        ECANSetFilterMode(ECAN_FILTER_MODE_ENABLE);
        ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

         
        TRISCbits.TRISC2=0;     // CS, out
        TRISCbits.TRISC3=0;     // SCL, out
        TRISCbits.TRISC5=0;     // SDO, out
        TRISCbits.TRISC4=1;     // SDI  in
        LATCbits.LATC2=1;       //Disable del encoder al inicio

		
        while(1){

                LATCbits.LATC2=0;
                val=WriteSPI (0b00010000);        //Solicitación de posición (comando 0x10)
                Delay10TCYx(3);                 //Wait 6us
                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                LATCbits.LATC2=1;

                LATCbits.LATC2=0;
                val=WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
                Delay10TCYx(3);                 //Wait 6us
                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                LATCbits.LATC2=1;
                
                while(y!=0b00010000) {                //Espera hasta que el encoder esté preparado para enviar algo (Respuesta 0x10)

                                LATCbits.LATC2=0;
                                val=WriteSPI (0b00000000);      //Espera a que el encoder esté listo (comando 0x00)
                                Delay10TCYx(3);                 //Wait 6us
                                y=SSPBUF;                       //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                                LATCbits.LATC2=1;

                                }

                LATCbits.LATC2=0;
                val=WriteSPI(0b00000000);                //Espera para captar la parte alta de la posición
                Delay10TCYx(3);                         //Wait 6us
                y=SSPBUF;                               //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                Orden1[0]=y;                            //Se almacena en el primer elemento del array que se enviará por CAN
                //i++;
				//Orden1[0]=i;
				LATCbits.LATC2=1;
                LATCbits.LATC2=0;
                val=WriteSPI(0b00000000);                //Espera para captar la parte baja del mensage
                Delay10TCYx(3);                         //Wait 6us
                y=SSPBUF;                               //Recoge los datos del SSPBUF que provienen del encoder (MISO)
                Orden1[1]=y;                            //Se almacena en el segundo elemento del array que se enviará por CAN
                //Orden1[1]=0b11111111;
				LATCbits.LATC2=1;
							
                aux=(Orden1[0]<<8)+Orden1[1];       //Se rota el byte alto de la posición 8 bits (2^8=256) y se suma al bajo
                
				grados=aux / div;                   //Se divide para obtener la realacción en grados


                x=0;
				while( !x )
				{
					x=ECANSendMessage(CAN_ID, &grados, sizeof(grados), OrdenFlags1);

                }
                
				//  Hemos estado probando con Delay10KTCYx()
				//
					Delay10KTCYx(1);						// 50019	(10,0038 ms)	accepted relative error  
															//  5 -> 10msg.
															// 50 -> 100msg.
				


				//	Delay100TCYx(200);                      // gives a delay of 10 x 256 x 1/6 = 500 us
       			 
                }
}
            