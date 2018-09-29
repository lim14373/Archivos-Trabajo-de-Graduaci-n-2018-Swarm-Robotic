/*
 * File:   main.c
 * Author: MARLON
 *
 * Created on 22 de octubre de 2017, 08:26 PM
 */

//#include "C:\Program Files (x86)\Microchip\xc32\v1.44\pic32mx\include\p32xxxx.h"
//#include "C:\Program Files (x86)\Microchip\xc32\v1.44\pic32mx\include\xc.h"
#include <string.h>
#include <stdio.h>
//#include <stdlib.h>
#include <p32xxxx.h>

#include "../sys/attribs.h"
// PIC32MX250F128B Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = ON            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = ON            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SYSTEM CLOCK 40MHz
// Peripheral Clock 40MHz
#include <xc.h>

#define SHORT_DELAY (39062)
#define LONG_DELAY	(39062)

void Start_U1(void);
void SendChar_U1(char c);
char ReceiveChar_U1(void);
void SendString_U1(char *c);
char *ReceiveString_U1(char *s);


void main(void) {
    
    //CLOCK INTERNO CON 40kHz en PBCLK
    
    OSCTUNbits.TUN  = 0b000000;         // Center Freq. Oscilator at 8MHz
    OSCCONbits.PLLODIV = 0b001;         // PLL out divided by 2
    OSCCONbits.FRCDIV = 0b000;          // FRC divided by 1
    OSCCONbits.SOSCRDY = 0b0;           // Secondary Osc is warming or turned off
    OSCCONbits.PBDIVRDY = 0b1;          // PBDIV bits can be written
    OSCCONbits.PBDIV = 0b00;            // PBCLK is SYSCLK divided by 1
    OSCCONbits.PLLMULT = 0b101;         // Clock is multiplied by 20
    OSCCONbits.COSC = 0b001;            // FRCPLL
    OSCCONbits.NOSC = 0b001;            // FRCPLL
    OSCCONbits.CLKLOCK = 0b0;           // Clock and PLL can be modify
    OSCCONbits.ULOCK = 0b0;             // USB is out of lock or is disabled
    OSCCONbits.SLOCK = 0b0;             // PLL Lock Status
    OSCCONbits.SLPEN = 0b0;             // Will enter in idle mode when WAIT is executed
    OSCCONbits.CF = 0b0;                // No clock failure has been detected
    OSCCONbits.UFRCEN = 0b0;            // Use the primary Osc or USB PLL as the USB clock source
    OSCCONbits.SOSCEN = 0b0;            // Disable secondary oscilator
    OSCCONbits.OSWEN = 0b0;             // Oscilator switch is complete
    
    T1CONbits.ON = 0b0;                 // Timer 1 is disabled
    T1CONbits.SIDL = 0b0;
    T1CONbits.TWDIS = 0b0;
    T1CONbits.TWIP = 0b0;
    T1CONbits.TGATE = 0b0;
    T1CONbits.TCKPS = 0b11;             // Prescaler at 1:256
    T1CONbits.TSYNC = 0b0;
    T1CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source    
    
    ANSELAbits.ANSA0 = 0; //pin digital
    ANSELAbits.ANSA1 = 0; //pin digital
    ANSELBbits.ANSB0 = 0; //pin digital
    ANSELBbits.ANSB1 = 0; //pin digital
    TRISAbits.TRISA0 = 0; //salida
    TRISAbits.TRISA1 = 0; //salida
    TRISBbits.TRISB0 = 0; //salida
    TRISBbits.TRISB1 = 0; //salida
    
    CFGCONbits.IOLOCK = 0b0;            // IOLock is disabled.
    
    RPB7Rbits.RPB7R = 0b0101;           // Se asigna OC1 a RPB7
    RPB8Rbits.RPB8R = 0b0101;           // Se asigna OC2 a RPB8
    RPB10Rbits.RPB10R = 0b0101;         // Se asigna OC3 a RPB10
    
    //TIMER 2 ES PARA CONTROLAR LOS MOTORES
    
    T2CONbits.ON = 0b0;                 // Timer 2 is disabled
    T2CONbits.SIDL = 0b0;
    T2CONbits.TGATE = 0b0;
    T2CONbits.TCKPS = 0b100;            // Prescaler at 1:16
    T2CONbits.T32 = 0b0;
    T2CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    //TIMER 3 ES PARA CONTROLAR AL SERVO
    
    T3CONbits.ON = 0b0;                 // Timer 3 is disabled
    T3CONbits.SIDL = 0b0;
    T3CONbits.TGATE = 0b0;
    T3CONbits.TCKPS = 0b100;            // Prescaler at 1:16
    T3CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    //PWM 1 PARA CONTROLAR MOTOR A
    
    OC1CONbits.ON = 0b0;                // OC1 is disabled
    OC1CONbits.SIDL = 0b0;
    OC1CONbits.OC32 = 0b0;
    OC1CONbits.OCFLT = 0b0;
    OC1CONbits.OCTSEL = 0b0;            // Timer 2 is the clock source
    OC1CONbits.OCM = 0b110;             // Estandar PWM
    
    //PWM 2 PARA CONTROLAR MOTOR B
    
    OC2CONbits.ON = 0b0;                // OC2 is disabled
    OC2CONbits.SIDL = 0b0;
    OC2CONbits.OC32 = 0b0;
    OC2CONbits.OCFLT = 0b0;
    OC2CONbits.OCTSEL = 0b0;            // Timer 2 is the clock source
    OC2CONbits.OCM = 0b110;             // Estandar PWM
    
    //PWM 3 PARA CONTROLAR SERVO
    
    OC3CONbits.ON = 0b0;                // OC3 is disabled
    OC3CONbits.SIDL = 0b0;
    OC3CONbits.OC32 = 0b0;
    OC3CONbits.OCFLT = 0b0;
    OC3CONbits.OCTSEL = 0b1;            // Timer 3 is the clock source
    OC3CONbits.OCM = 0b110;             // Estandar PWM
    
    PR1 = 156;
    PR2 = 5000;                         // Periodo de 500Hz (Motores)
    PR3 = 50000;                        // Periodo de 50Hz (Servo) 2500-5000 / 0° - 180°
    
    OC1RS = 2500;
    OC2RS = 2500;
    OC3RS = 44000;                      // PR3 * 85%
    
    OC1CONbits.ON = 0b1;                // OC1 is enabled
    OC2CONbits.ON = 0b1;                // OC2 is enabled
    OC3CONbits.ON = 0b1;                // OC3 is enabled  
    T1CONbits.ON = 0b1;                 // Timer 1 is enabled
    T2CONbits.ON = 0b1;                 // Timer 2 is enabled
    T3CONbits.ON = 0b1;                 // Timer 3 is enabled
    
    int i;
    int dif = 4750;

    while(1)
    {
        for(i=0; i<=dif; i++){
            while (TMR1 < 78)
            {                
            }
            OC3RS += 1;
            TMR1 = 0;
        }
        for(i=0; i<=dif; i++){
            while (TMR1 < 78)
            {                
            }
            OC3RS -= 1;
            TMR1 = 0;
        }
//        while ( TMR1 < 25)
//        {
//        }
//        OC1RS += 1;
//        OC2RS += 1;
//        OC3RS += 1;
//        OC4RS += 1;
//        if(OC1RS == PR3){
//            OC1RS = 0;
//        }
//        if(OC2RS == PR3){
//            OC2RS = 0;
//        }
//        if(OC3RS == PR3){
//            OC3RS = 0;
//        }
//        if(OC4RS == PR3){
//            OC4RS = 0;
//        }
//        TMR1 = 0;
    }
    return;
}

//INICIALIZAR UART
void Start_U1(void) {
    U1BRG = 86;                        //baud rate=115200 a Fpb = 40 MHZ

    U1MODEbits.ON = 0b1;               // UART Enabled
    U1MODEbits.SIDL = 0b0;
    U1MODEbits.IREN = 0b0;
    U1MODEbits.RTSMD = 0b1;            // Simplex Mode
    U1MODEbits.UEN = 0b00;
    U1MODEbits.WAKE = 0b0;
    U1MODEbits.LPBACK = 0b0;
    U1MODEbits.ABAUD = 0b0;
    U1MODEbits.RXINV = 0b0;
    U1MODEbits.BRGH = 0b1;             // High Speed Mode
    U1MODEbits.PDSEL = 0b00;           // No parity
    U1MODEbits.STSEL = 0b0;            // 1 stop bit   
    
    U1STAbits.URXEN =0;                //deshabilitar recepcion
    U1STAbits.UTXEN=1;                 //habilitar transmision
} 

//ENVIAR CARACTER POR SERIAL
void SendChar_U1(char c) {
    U1STAbits.URXEN =0;//deshabilitar recepcion
    U1STAbits.UTXEN=1;//habilitar transmision
    U1TXREG = c;
    while (!U1STAbits.TRMT); //espera a que termine la transimision
}

//RECIBE CARACTER POR SERIAL
char ReceiveChar_U1(void) {
    U1STAbits.UTXEN = 0;//deshabilitar transmision
    U1STAbits.URXEN = 1;//habilitar recepcion
    while (!U1STAbits.URXDA); //espera a que se reciva el caracter
    return U1RXREG; //devolver dato recibido
}

//ENVIAR STRING POR SERIAL
void SendString_U1(char *c){
    while(*c){
        //si no funciona probar cambiando por estas lineas
        //char a=*(c++);
        //SendChar_U1(a); 
        SendChar_U1(*(c++)); 
    }
    //SendChar_U1('\n'); //salto de linea
}

//RECIBE STRING POR SERIAL
char *ReceiveString_U1(char *s){
    char *p = s; //copiar puntero
    while(1){
        *s = ReceiveChar_U1(); //recive y almacena caracter
        if (*s== '\n'){ break; } //verifica el final de linea
        s++; //incrementa puntero de buffer
    }
    s++;
    *s = '\0';  //agrega caracter de final de cadena
    return p; //regresa el puntero del string
}