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


union data_int{
    signed long val;
    unsigned char val_array[4];
};

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

void configADC(void){
//    ADC Config:
    AD1CON1bits.ADON = 0;   // turn off module before config 
    AD1CON1bits.AD12B = 1;  // 12 bit mode
    AD1CON1bits.FORM = 0;   // data format unsigned int
    AD1CON1bits.SSRC = 0;   // clear samp to start conversion
    
    AD1CON2bits.CHPS = 0;   // select ch 0
    AD1CON2bits.VCFG = 0;   // select AVdd & AVss for ref
    
    AD1CON3bits.ADCS = 0;   // use system clock
    AD1CON3bits.ADRC = 0;   // use 1x clock
    
    AD1CHS0bits.CH0NA = 0;  // use VReffL
    AD1CHS0bits.CH0SA = 0;  // select AN0
    
    TRISAbits.TRISA0 =1;    // set AN0 input
    TRISAbits.TRISA1 =1;    // set AN1 input
    TRISBbits.TRISB0 =1;    // set AN2 input
    TRISBbits.TRISB1 =1;    // set AN3 input
    TRISBbits.TRISB2 =1;    // set AN4 input
    TRISBbits.TRISB3 =1;    // set AN5 input

    AD1CON1bits.ADON = 1;   // turn on ADC module    
}

int main(void) {
    TRISBbits.TRISB15 = 0;
    configADC();
    configUART();

    while(1) {
        __delay_ms(1);
    }
    
    return (0);
    
}

unsigned int readADC(char chan){
    unsigned int data = 0;
    
    AD1CHS0bits.CH0SA = chan;
    AD1CON1bits.SAMP = 1;
    __delay_us(100);
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE);
    data = ADC1BUF0;
    return data;
}

unsigned char checksum(unsigned char data_in[8]){
    unsigned int i;
    unsigned int sum=0;
    unsigned char out;
    for (i=0; i<7; i++){
        sum = sum + data_in[i];
    }
    sum = 0xFF - (sum % 255);
    out = sum;
    return out;   
}

void sendData(){
    unsigned char data[8];
    unsigned int i;
    union data_int data_out;
    
    RPINR18bits.U1RXR = 0b0;
    TRISBbits.TRISB6 = 0;
    RPOR2bits.RP38R = 0b000001;
    PORTBbits.RB15 = 1;
    __delay_us(2);
    PORTBbits.RB15 = 0;

    //data_out.val = (5L*readADC(0)*2550L)>>12;
    data_out.val = (2100L<<20) | (1850L<<8) | (2L<<4);

    data[0] = 0x10;
    data[1] = 0x00;
    data[2] = 0x03;
    data[3] = data_out.val_array[0];
    data[4] = data_out.val_array[1];
    data[5] = data_out.val_array[2];
    data[6] = data_out.val_array[3];
    data[7] = checksum(data);
    __delay_ms(4);

    for (i=0; i<8; i++){
        U1TXREG = data[i];
        __delay_us(200);
    }
    
    RPINR18bits.U1RXR = 0b0100110;
    RPOR2bits.RP38R = 0b000000;
    TRISBbits.TRISB6 = 1;
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void) {

    unsigned char rxChar;
    unsigned char rxChar2;

    U1STAbits.OERR = 0;
    IEC0bits.U1RXIE = 0;
    rxChar = U1RXREG;
    if (rxChar == 0x7E){
        //Start of Frame...
        __delay_us(200);        
        rxChar2 = U1RXREG;
        __delay_us(200);
        if (rxChar2 == 0x6A){
            sendData();
        }
    }
    IFS0bits.U1RXIF = 0;        // clear rx interupt
    IEC0bits.U1RXIE = 1;        // enable rx interupt
}

