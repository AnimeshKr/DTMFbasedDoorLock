/***********DO NOT REMOVE **********/
#define USE_AND_OR          // Macro to use Periheral Lib
/***********DO NOT REMOVE **********/
#define F_CPU 32000000        // 32Mhz ??
#define microsecondsToClockCycles(a) ( ((a/1000L) * (F_CPU / 1000L))  )
#define clockCyclesToMicroseconds(a) ( (a * 1000L) / (F_CPU / 1000L) )
/*** MICROCHIP LIBRARY ***/
#include <stdio.h>
#include <stdlib.h>
#include <p24FJ256GB206.h>

/*** BRIGOSHA LIBRARY ***/
#include "Define.h"
#include "CONFIGbits.h"

#include "ELBv21_HardwareConfig.h"
#include "ELB_OSC.h"
#include "ELB_I2C.h"
#include "ELB_LCD.h"
#include "ELB_Timer.h"
#include <time.h>
#define PIR _RE4

/*** EXTERNAL VARIABLES  ***/
extern  ts_ELB_Hardware Hardware;                   //Hardware Global Structure
extern U8 V_DTMFIntFlag_U8;
extern U8 V_T1IntFlag_U8;
extern U8 V_T2IntFlag_U8;

/*** GLOBAL VARIABLES ***/
U16 v_PrintData_U16=0;
U8 A_Str_U8[50];
U8 doorIsOpen = 0;
U8 PIRisActive=1;
int readChannel(int channel);

void enableInterruptForDoorClosure()
{
    TIMER2_INIT(10000,TMR_INT_PRI1);
}

int readChannel(int channel)
{
    int ADCValue;
    AD1CHS = channel;
    AD1CON1bits.SAMP = 1;
    int ii = 1000000;
    while(ii--);
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE){};
    ADCValue = ADC1BUF0;
    return ADCValue;
}

void init()
{
    /*** CONFIGURE OSCILLATOR ***/
    SET_FreqOsc( FRCDIV_250KHZ );                     //Set Frequency of Oscillator

    /*** CONFIGURE HARDWARE ****/
    Hardware_INIT();                                //Intialize Hardware
    Hardware.ConfigPins_Default();                  //Configure Hardware


    /*** INITIALIZE PERIPHERAL ***/
    LCD_INIT();                                     //Initialize LCD

    AD1CON1 = 0x0000;
    AD1CSSL = 0;
    AD1CON3 = 0x0002;
    AD1CON2 = 0;
    AD1CON1bits.ADON = 1;

    _TRISE4 = 1;
    _RE4=0;

    TIMER1_INIT(1000,TMR_INT_PRI2);
}


void closeDoor()
{
    Motor_B_Anticlockwise();
    DELAY_mSec(5000);
    Motor_B_Stop();
    Motor_A_Anticlockwise();
    DELAY_mSec(2000);
    Motor_A_Stop();
}

void openDoor()
{
    Motor_A_Clockwise();
    DELAY_mSec(2000);        // make delay function
    Motor_A_Stop();
    Motor_B_Clockwise();
    DELAY_mSec(4000);
    Motor_B_Stop();
}

void Motor_A_Clockwise()
{
    MotA1 = C_ON;
    MotA2 = C_OFF;
}

void Motor_A_Anticlockwise()
{
    MotA1 = C_OFF;
    MotA2 = C_ON;
}

void Motor_B_Clockwise()
{
    MotB1 = C_ON;
    MotB2 = C_OFF;
}

void Motor_B_Anticlockwise()
{
    MotB1 = C_OFF;
    MotB2 = C_ON;
}

void Motor_A_Stop()
{
    MotA1 = C_OFF;
    MotA2 = C_OFF;
}

void Motor_B_Stop()
{
    MotB1 = C_OFF;
    MotB2 = C_OFF;
}

U16 getDecimal(U16 a,U16 b,U16 c,U16 d)
{
    return(d + 2*c + 4*b + 8*a);
}



int main(void)
{
    init();
    U16 word = 0;
    U16 password = 123123;
    int th0=600,th2=600;
    while(1)
    {
        if(V_DTMFIntFlag_U8)
        {
            V_DTMFIntFlag_U8= C_FALSE;
            U16 digit = getDecimal(DTMF1,DTMF2,DTMF3,DTMF4);
            if(digit == 11/*for '*' */)
                word = 0;
            else
                word = word*10+digit;

            if(word==password)
            {
                if(!doorIsOpen)
                {
                        openDoor();
                        doorIsOpen = 1;
                        enableInterruptForDoorClosure();       //initialize timer for automatic closure of door
                }
                else
                {
                    closeDoor();
                    doorIsOpen = 0;
                    T2CONbits.TON = 0;
                }
            }
        }

        if(V_T2IntFlag_U8)  /*timer interrupt for closing door*/
        {
            closeDoor();
            doorIsOpen = 0;
            V_T2IntFlag_U8 = C_FALSE;
            T2CONbits.TON = 0;
        }

        if((readChannel(0)>th0||readChannel(2)>th2)&& !doorIsOpen)
        {
            sprintf(A_Str_U8,"Break In!!");
            LED1 = LED2 = LED3 = 1;
            LCD_WriteString(1, 0, A_Str_U8);
        }
        //if(/*High Temperature interrupt*/)
        //{/*High alert*/}


        //////////////////////////////////////////////PIR//////////////////////////////////////////
        if (PIRisActive==1 && PIR == 1) {                             // check if the input is HIGH
            LED1 = 1;
            sprintf(A_Str_U8,"detected!");
            LCD_WriteString(2, 5, A_Str_U8);    // We only want to print on the output change, not state
            if(!doorIsOpen)
            {
                openDoor();
                doorIsOpen = 1;
                enableInterruptForDoorClosure();     //initialize timer for automatic closure of door
            }
        }
    }
}
