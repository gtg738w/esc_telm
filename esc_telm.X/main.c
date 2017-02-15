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

struct cells{
    unsigned long packet_id     :  4;
    unsigned long cell_count    :  4;
    unsigned long cell_1        : 12;
    unsigned long cell_2        : 12;
};

union data_int{
    signed long val;
    unsigned char val_array[4];
    struct cells cell_struct;
};

struct data_array{
    unsigned char data[8];
};

unsigned int sport_field = 0;
unsigned char data_ret[8];
unsigned int ic_val1=0, ic_val2=0;
unsigned long esc_current = 0;
unsigned long esc_voltage = 0;
unsigned long esc_temp = 0;
unsigned int esc_field = 0;

unsigned char ic_sync = 0;
unsigned char ic_field = 0;

unsigned int ic_data[12];


/*
 * 
 */

void configUART(void){
    RPINR18bits.U1RXR = 0b0100110;  // set U1 rx to rp38
    RPOR2bits.RP38R = 0b000000;     // clear tx pin
    U1BRG = 3;                      // set baud rate
    U1MODEbits.STSEL = 0;           // 1 stop bit
    U1MODEbits.PDSEL = 0;           // no Parity, 8-data bits
    U1MODEbits.ABAUD = 0;           // auto-baud disabled
    U1MODEbits.BRGH = 0;            // set std speed mode
    U1MODEbits.RXINV = 1;           // rx idle is 0
    U1STAbits.UTXINV = 1;           // tx idle is 0
    U1STAbits.URXISEL = 0;          // interrupt after one RX character is received;
    U1MODEbits.UARTEN = 1;          // enable UART 1        
    U1STAbits.UTXEN = 1;            // enable transmitter
    IEC0bits.U1RXIE = 1;            // enable rx interupts
    TRISBbits.TRISB6 = 1;           // set rx pin to input
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

void configIC(void){
    TRISBbits.TRISB12 = 1;
    RPINR7bits.IC1R = 0b0101100;
    
    T2CONbits.TCKPS = 0b00;
    T2CONbits.TON = 1;
    
    
    IFS0bits.IC1IF = 0;         // Clear the IC1 interrupt status flag
    IEC0bits.IC1IE = 1;         // Enable IC1 interrupts
    IPC0bits.IC1IP = 1;         // Set module interrupt priority as 1
    
    IC1CON1bits.ICTSEL = 0b001; // use timer2 for clock
    IC1CON1bits.ICM = 0b001;    // capture every edge
    
    IC1CON2 = 0;       // 16bit timer mode
    
//    IFS0bits.IC1IF = 0; // Clear the IC1 interrupt status flag
//    IEC0bits.IC1IE = 1; // Enable IC1 interrupts
//    IPC0bits.IC1IP = 1; // Set module interrupt priority as 1
//    //IC1CON1bits.ICSDL = 0; // Input capture will continue to operate in CPU idle mode
//    IC1CON1bits.ICTSEL = 0b111; // Peripheral (FP) is the clock source for the IC1 module
//    IC1CON1bits.ICI = 0; // Interrupt on every capture event
//    IC1CON1bits.ICBNE = 0; // Input capture is empty
//    IC1CON1bits.ICM = 0b011; // Capture mode; every fourth rising edge (Prescaler Capture mode)
//    IC1CON2bits.IC32 = 0; // Cascade module operation is disabled
//    IC1CON2bits.ICTRIG = 0; // Input source used to synchronize the input capture timer of
//    // another module (Synchronization mode)
//    IC1CON2bits.TRIGSTAT = 0; // IC1TMR has not been triggered and is being held clear
//    IC1CON2bits.SYNCSEL = 0; // No Sync or Trigger source for the IC1 module
}



int main(void) {
    TRISBbits.TRISB15 = 0;
    configADC();
    configUART();
    configIC();

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

struct data_array cels1(){
    union data_int data_out;
    struct data_array data_array_struct;
    unsigned long cell1, cell2;
    
    cell1 = readADC(0)*2550L/4096L;
    cell2 = readADC(1)*2550L/4096L;
    
    data_out.val = (cell2<<20) | (cell1<<8) | (6L<<4);

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x00;
    data_array_struct.data[2] = 0x03;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array cels2(){
    union data_int data_out;
    struct data_array data_array_struct;
    unsigned long cell1, cell2;
    
    cell1 = readADC(2)*2550L/4096L;
    cell2 = readADC(3)*2550L/4096L;
    
    data_out.val = (cell2<<20) | (cell1<<8) | (6L<<4) | 2L;

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x00;
    data_array_struct.data[2] = 0x03;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array cels3(){
    union data_int data_out;
    struct data_array data_array_struct;
    unsigned long cell1, cell2;
    
    cell1 = readADC(4)*2550L/4096L;
    cell2 = readADC(5)*2550L/4096L;
    
    data_out.val = (cell2<<20) | (cell1<<8) | (6L<<4) | 4L;

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x00;
    data_array_struct.data[2] = 0x03;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_voltage(){
    union data_int data_out;
    struct data_array data_array_struct;
    
    
    data_out.val = ((ic_data[2]-500L)*20L*2550L)/1000L;

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x00;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_temp(){
    union data_int data_out;
    struct data_array data_array_struct;
    
    
    data_out.val = ((ic_data[10]-500L)*30L*2550L)/1000L;

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x01;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

void sendData(){
    unsigned int i;
    struct data_array data_struct;
    
    
    RPINR18bits.U1RXR = 0b0;
    TRISBbits.TRISB6 = 0;
    RPOR2bits.RP38R = 0b000001;
    //PORTBbits.RB15 = 1;
    //__delay_us(2);
    //PORTBbits.RB15 = 0;
    
    switch (sport_field){
        case 0:         // cels1
            data_struct = cels1();
            break;
        case 1:
            data_struct = cels2();
            break;
        case 2:
            data_struct = cels3();
            break;
        case 3:
            data_struct = calc_esc_voltage();
            break;
        case 4:
            data_struct = calc_esc_temp();
            break;
    }
    sport_field++;
    if (sport_field == 5){
        sport_field = 3;
    }

    //data_out.val = (5L*readADC(0)*2550L)>>12;

    __delay_ms(4);

    for (i=0; i<8; i++){
        U1TXREG = data_struct.data[i];
        __delay_us(200);
    }
    
    RPINR18bits.U1RXR = 0b0100110;
    RPOR2bits.RP38R = 0b000000;
    TRISBbits.TRISB6 = 1;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {

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

void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt(void){
    
    unsigned long value,delay;
    
    //IEC0bits.IC1IE = 0;   // disable int
    
    IFS0bits.IC1IF = 0;     // Reset respective interrupt flag
    
    while (IC1CON1bits.ICBNE){
    
        ic_val2 = IC1BUF;       // Read and save off first capture entry
        value = ic_val2 - ic_val1;
        delay = (value*1L/3.685);
        ic_val1 = ic_val2;

//            RPINR18bits.U1RXR = 0b0;
//            TRISBbits.TRISB6 = 0;
//            RPOR2bits.RP38R = 0b000001;
            
            

        
        if (delay < 250L){
            
            ic_sync = 1;
        }
        else if (ic_sync == 0){
            ic_sync = 0;
        }
        else {
//            U1TXREG = delay>>8;
//            U1TXREG = delay % 0xFF;
            ic_sync = ic_sync + 1;
        }

        switch (ic_sync){
            case 2:
                
                break;
            case 3:
                break;
            case 4:
                if (delay > 5500){
                    PORTBbits.RB15 = 1;
            __delay_us(1);
            PORTBbits.RB15 = 0;
                    ic_sync = 2;
                    ic_field = 1;
                    
                }
                else if (ic_field == 0){
                    ic_field = 0;
                }
                else {
                    ic_data[ic_field] = delay;
                    ic_field++;
                }
                break;
        }
    }

    
    //RPINR18bits.U1RXR = 0b0100110;
    //RPOR2bits.RP38R = 0b000000;
    //TRISBbits.TRISB6 = 1;
    //value = IC1BUF;
    //PORTBbits.RB15 = 1;
    
//    if (value <10000){
//        PORTBbits.RB15 = 1;
//        __delay_us(value);
//        PORTBbits.RB15 = 0;
//    }
    
    //TMR2 = 0;
    //T2CONbits.TON =1;
    IEC0bits.IC1IE = 1;
}
