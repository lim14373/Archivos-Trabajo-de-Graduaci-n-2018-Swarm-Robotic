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
void main(void) {
    
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
    
    T1CONbits.ON = 0b1;                 // Timer 1 is enabled
    T1CONbits.SIDL = 0b0;
    T1CONbits.TWDIS = 0b0;
    T1CONbits.TWIP = 0b0;
    T1CONbits.TGATE = 0b0;
    T1CONbits.TCKPS = 0b11;             // Prescaler at 1:256
    T1CONbits.TSYNC = 0b0;
    T1CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source    
    
    ANSELAbits.ANSA0 = 0; //pin digital
    ANSELAbits.ANSA1 = 0; //pin digital
    TRISAbits.TRISA0 = 0; //salida
    TRISAbits.TRISA1 = 0; //salida
    
    CFGCONbits.IOLOCK = 0b0;            // IOLock is disabled.
    
    RPB7Rbits.RPB7R = 0b0101;           // Se asigna OC1 a RPB7
    RPB8Rbits.RPB8R = 0b0101;           // Se asigna OC2 a RPB8
    
    OC1CONbits.ON = 0b0;                // OC1 is disabled
    OC1CONbits.SIDL = 0b0;
    OC1CONbits.OC32 = 0b0;
    OC1CONbits.OCFLT = 0b0;
    OC1CONbits.OCTSEL = 0b0;            // Timer 2 is the clock source
    OC1CONbits.OCM = 0b110;             // Estandar PWM
    
    OC1CONbits.ON = 0b0;                // OC1 is disabled
    OC1CONbits.SIDL = 0b0;
    OC1CONbits.OC32 = 0b0;
    OC1CONbits.OCFLT = 0b0;
    OC1CONbits.OCTSEL = 0b0;            // Timer 2 is the clock source
    OC1CONbits.OCM = 0b110;             // Estandar PWM
    
    OC2CONbits.ON = 0b0;                // OC2 is disabled
    OC2CONbits.SIDL = 0b0;
    OC2CONbits.OC32 = 0b0;
    OC2CONbits.OCFLT = 0b0;
    OC2CONbits.OCTSEL = 0b1;            // Timer 3 is the clock source
    OC2CONbits.OCM = 0b110;             // Estandar PWM
    
    T2CONbits.ON = 0b0;                 // Timer 2 is disabled
    T2CONbits.SIDL = 0b0;
    T2CONbits.TGATE = 0b0;
    T2CONbits.TCKPS = 0b111;            // Prescaler at 1:256
    T2CONbits.T32 = 0b0;
    T2CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    T3CONbits.ON = 0b0;                 // Timer 3 is disabled
    T3CONbits.SIDL = 0b0;
    T3CONbits.TGATE = 0b0;
    T3CONbits.TCKPS = 0b111;            // Prescaler at 1:256
    T3CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    PR2 = 3124;
    PR3 = 3124;
    
    OC1RS = 156;
    OC2RS = 312;
    
    OC1CONbits.ON = 0b1;                // OC1 is enabled
    OC2CONbits.ON = 0b1;                // OC2 is disabled
    T2CONbits.ON = 0b1;                 // Timer 2 is enabled
    T3CONbits.ON = 0b1;                 // Timer 3 is enabled
    
    
//    int i;

    while(1)
    {
    }
    return;
}