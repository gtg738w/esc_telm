/* 
 * File:   main.c
 * Author: russ
 *
 * Created on January 22, 2017, 9:39 PM
 */

#define FOSC (7370000ULL*8LL)
#define FCY  (FOSC/2)
#include "xc.h"
#include <libpic30.h>
#include <p33EV256GM102.h>
#include <math.h>

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);
// Enable Clock Switching and Configure Primary Oscillator in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);

struct cells{
    unsigned long packet_id     :  4;
    unsigned long cell_count    :  4;
    unsigned long cell_1        : 12;
    unsigned long cell_2        : 12;
};

union data_int{
    signed long val;
    unsigned char val_array[4];

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

long esc_offset = 0;
long esc_scale = 1;

unsigned char ic_sync = 0;
unsigned char ic_field = 0;

unsigned int ic_data[12];

unsigned char cell_count = 2;


/*
 * 
 */

void configPWM(void){

    PTPER = 590;
    //MDC = 0x80;
    PDC3 = PTPER >> 2;
    //IOCON1bits.PENH = 1;
    //TRISBbits.TRISB13 = 0;
    //PTCONbits.PTEN = 1;
    
}

void configUART(void){
    RPINR18bits.U1RXR = 0b0100110;  // set U1 rx to rp38
    RPOR2bits.RP38R = 0b000000;     // clear tx pin
    U1BRG = 31;                      // set baud rate
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
    
    T2CONbits.TCKPS = 0b01;
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

unsigned int readADC(unsigned int chan){
    unsigned int data = 0;
    
    AD1CHS0bits.CH0SA = chan;
    AD1CON1bits.SAMP = 1;
    __delay_us(100);
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE);
    data = ADC1BUF0;
    return data;
}

void get_cell_count(){
    cell_count = 0;
    if (readADC(0) > 1000){
        cell_count = 1;
        if (readADC(2) > 1000){
            cell_count = 2;
            if (readADC(3) > 1000){
                cell_count = 3;
                if (readADC(4) > 1000){
                    cell_count = 4;
                    if (readADC(5) > 1000){
                        cell_count = 5;
                        if (readADC(6) > 1000){
                            cell_count = 6;
                        }
                    }
                }
            }
        }
    }
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

void calc_esc_cal(){
    
    unsigned long low;
    unsigned long high;
    
    high = ic_data[1];
    
    if (ic_data[10] < ic_data[11]){
        low = ic_data[10];
    }
    else {
        low = ic_data[11];
    }
    esc_offset = high-2*low;
    esc_scale = (low + esc_offset) * 2L;
    //esc_scale = low;
    
    
    
}


struct data_array cels1(){
    union data_int data_out;
    struct data_array data_array_struct;
    unsigned long cell1, cell2;
    
    cell1 = (unsigned long) readADC(0)*5L*500L/4096L;
    cell2 = (unsigned long) readADC(2)*5L*500L/4096L;
    
    data_out.val = ((cell2<<20) | (cell1<<8) | (cell_count<<4) ) + 0;

    //data_out.val = cell1;
    
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
    
    cell1 = readADC(3)*2550L/4096L;
    cell2 = readADC(4)*2550L/4096L;
    
    data_out.val = ((cell2<<20) | (cell1<<8) | (3L<<4)) + 2;

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
    
    cell1 = readADC(5)*2550L/4096L;
    cell2 = readADC(6)*2550L/4096L;
    
    data_out.val = ((cell2<<20) | (cell1<<8) | (3L<<4)) + 4;

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
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[2]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*20L*2550L)/esc_scale;

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

struct data_array calc_esc_vrip(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[3]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*4L*2550L)/esc_scale;

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

struct data_array calc_esc_I(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[4]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*50L*2550L)/esc_scale;

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x02;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_throttle(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[5]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*1L*2550L)/esc_scale;

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x03;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_power(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[6]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*2550L)/(esc_scale*4L);

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x04;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_rpm(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[7]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*20416L)/(esc_scale);

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x05;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_becV(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[8]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*4L*2550L)/(esc_scale);

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x06;
    data_array_struct.data[2] = 0x0c;
    data_array_struct.data[3] = data_out.val_array[0];
    data_array_struct.data[4] = data_out.val_array[1];
    data_array_struct.data[5] = data_out.val_array[2];
    data_array_struct.data[6] = data_out.val_array[3];
    data_array_struct.data[7] = checksum(data_array_struct.data);
    
    return data_array_struct;
}

struct data_array calc_esc_becI(){
    union data_int data_out;
    struct data_array data_array_struct;
    signed long pulse;
    calc_esc_cal();
    pulse = ic_data[9]-500L + esc_offset;
    if (pulse < 0){
        pulse = 0;
    }
    data_out.val = (pulse*4L*2550L)/(esc_scale);

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x07;
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
    long pulse;
    float pulseFp;
    float R0 = 10000, R2 = 10200, B = 3455;

    if (ic_data[10] < ic_data[11]){
        pulse = ic_data[11]-500L + esc_offset;
        if (pulse < 0){
        pulse = 0;
        }
        pulseFp = (float) pulse / ((float) esc_scale);
        pulseFp = pulseFp * 63.8125;
        pulseFp = 1.0 / (log(pulseFp*R2 / (255.0-pulseFp) / R0) / B + 1/298.0);
        pulseFp = pulseFp - 273.0;
        data_out.val = (long) (pulseFp*2550.0);
    }
    else {
        pulse = ic_data[10]-500L + esc_offset;
        if (pulse < 0){
        pulse = 0;
        }
        data_out.val = (pulse*30L*2550L)/esc_scale;
    }
    

    

    data_array_struct.data[0] = 0x10;
    data_array_struct.data[1] = 0x08;
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
            data_struct = cels1();
            break;
        case 2:
            data_struct = cels1();
            break;
        case 3:
            data_struct = calc_esc_voltage();
            break;
        case 4:
            data_struct = calc_esc_vrip();
            break;
        case 5:
            data_struct = calc_esc_I();
            break;
        case 6:
            data_struct = calc_esc_throttle();
            break;
        case 7:
            data_struct = calc_esc_power();
            break;
        case 8:
            data_struct = calc_esc_rpm();
            break;
        case 9:
            data_struct = calc_esc_becV();
            break;
        case 10:
            data_struct = calc_esc_becI();
            break;
        case 11:
            data_struct = calc_esc_temp();
            break;
    }
    sport_field++;
    if (sport_field == 12){
        sport_field = 0;
    }
    //sport_field = 0;

    //data_out.val = (5L*readADC(0)*2550L)>>12;

    __delay_ms(2);

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
                    //PORTBbits.RB15 = 1;
                    //__delay_us(1);
                    //PORTBbits.RB15 = 0;
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
int main(void) {
    PLLFBD=30; // M=65
CLKDIVbits.PLLPOST=0; // N2=2
CLKDIVbits.PLLPRE=0; // N1=3
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b001);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK!= 1);
    TRISBbits.TRISB15 = 0;
    configADC();
    configUART();
    configIC();
    configPWM();
    get_cell_count();
    while(1) {
        __delay_ms(100);
        get_cell_count();
//            RPINR18bits.U1RXR = 0b0;
//    TRISBbits.TRISB6 = 0;
//    RPOR2bits.RP38R = 0b000001;
//        U1TXREG = 255;
//        
    }
    
    return (0);
    
}