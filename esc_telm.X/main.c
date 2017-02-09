/* 
 * File:   main.c
 * Author: russ
 *
 * Created on January 22, 2017, 9:39 PM
 */

#define FOSC (7370000ULL)
#define FCY  (FOSC/2)
#include "xc.h"
#include <libpic30.h>
#include <p33EV256GM102.h>

/*
 * 
 */

void configUART(void){
    RPINR18bits.U1RXR = 0b0100110;  // set U1 rx to rp38
    RPOR2bits.RP38R = 0b000000;     // 
    U1BRG = 3;       // set baud rate
    U1MODEbits.STSEL = 0;           // 1 stop bit
    U1MODEbits.PDSEL = 0;           // no Parity, 8-data bits
    U1MODEbits.ABAUD = 0;           // auto-baud disabled
    U1MODEbits.BRGH = 0;            // set std speed mode
    U1MODEbits.RXINV = 1;           // rx idle is 0
    U1STAbits.UTXINV = 1;           // tx idle is 0
    U1STAbits.URXISEL = 0;          // interrupt after one RX character is received;
    U1MODEbits.UARTEN = 1;          // enable UART 1        
    U1STAbits.UTXEN = 1;            // enable transmitter
    IEC0bits.U1RXIE = 1;
    TRISBbits.TRISB6 = 1;
}



int main(void) {
    TRISBbits.TRISB15 = 0;
    configUART();
    //PORTBbits.RB15 = 1;
    while(1) {
        //__delay_us(1);
        //PORTBbits.RB15 = 0;
        __delay_ms(1);
        //U1TXREG = 0b00000010;
        //PORTBbits.RB15 = 1;
    }
    return (0);
    
}

unsigned char checksum(unsigned char data_in[7]){
    unsigned int i;
    unsigned char sum=0;
    for (i=0; i<7; i++){
        sum = sum + data_in[i];
    }
    sum = 0xFF - sum;
    U1TXREG = sum;
    return sum;   
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void) {

    unsigned char rxChar;
    unsigned char rxChar2;
    unsigned char data[8];
    unsigned int i;
    unsigned int check;

    U1STAbits.OERR = 0;
    IEC0bits.U1RXIE = 0;
    rxChar = U1RXREG;
    if (rxChar == 0x7E){
        //Start of Frame...
        __delay_us(200);        
        rxChar2 = U1RXREG;
        __delay_us(200);
        if (rxChar2 == 0x6A){
            RPINR18bits.U1RXR = 0b0;
            TRISBbits.TRISB6 = 0;
            RPOR2bits.RP38R = 0b000001;
            PORTBbits.RB15 = 1;
            __delay_us(2);
            PORTBbits.RB15 = 0;
            
            data[0] = 0x10;
            data[1] = 0x00;
            data[2] = 0x0C;
            data[3] = 0x00;
            data[4] = 0x0A;
            data[5] = 0x00;
            data[6] = 0x00;
            check = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6];
            check = 0xFF - (check % 255);
            data[7] = check;
            __delay_ms(4);

            for (i=0; i<8; i++){
                U1TXREG = data[i];
                __delay_us(200);
                
            }
            


            RPINR18bits.U1RXR = 0b0100110;
            RPOR2bits.RP38R = 0b000000;
            TRISBbits.TRISB6 = 1;
            
        }
        
    }
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
}

