#define _XTAL_FREQ 16000000

#include <pic16f1503.h>
#include <xc.h>

#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF
#pragma config WRT = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOR = OFF
#pragma config LVP = ON

void main(void)
{
    // 16 MHz speed.
    OSCCONbits.IRCF = 0xf;
    OSCCONbits.SCS = 0x3;
//    OSCCON = 0b01111011;
    PORTA = 0;
    PORTC = 0;
    LATA = 0;
    LATC = 0;
    ANSELA = 0;
    ANSELC = 0;
    TRISA = 0b00111111;
    TRISC = 0b00111111;
    
    PWM1CON = 0;
    PWM2CON = 0;
    PR2 = 0x7c;
    PWM1DCH = 0;
    PWM1DCL = 0;
    PWM2DCH = 0;
    PWM2DCL = 0;
    
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b01;
    T2CONbits.TMR2ON = 1;
    while (PIR1bits.TMR2IF == 0);
    
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    
    PWM1DCH = 0x1f;
    PWM2DCH = 0x44;

    PWM1CONbits.PWM1OE = 1;
    PWM2CONbits.PWM2OE = 1;
    
    PWM1CON = 0b11000000;
    PWM2CON = 0b11000000;
  
//    PWM1DCH = 0b10001000;
//    PWM1DCL = 0b10000000;
//    PWM2DCH = 0b01000100;
//    PWM2DCL = 0b01000000;

    while(1) {
        __delay_ms(10);
        PORTCbits.RC4 ^= 1;
    }
}
