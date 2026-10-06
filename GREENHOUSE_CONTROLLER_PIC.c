/*
======TRANSMITTING CIRCUIT (PIC MICROCONTROLLER)======================
*/
/*-----------------------------------------------------------------------------
    File Name:    GREENHOUSE CONTROLLER PIC.c
    Author:       Francis Nzekwe
    Date:         3/12/2023
    Modified:    None
    

Description:    This Program does the following:
                1. To fetch temperature, Humidity and Co2 samples from their respective sensors.
                    This sample is fetched every ten(10)seconds and used for the control of some Greenhouse variables.

                2.To sample sensors average from their respective sensors,
                    Push buttons were introduced to adjust the setpoints of the sensors and these sensor setpoints
                    are used to control a vent, sprinkler, fan, cooler and heater.
                    The result of the sensors sample are displayed on the display terminal.

                3.Generates a sentence and prints it to the serial port 1 (Printf).
                The sentence is genereated on a highlimit or low limit  switch trigger.
                The generated message persists on the display for 5 after button press.

                4.Sends the generated sentence from the serial port2 to the MBED Microcontroller

-----------------------------------------------------------------------------*/

/* Preprocessor ---------------------------------------------------------------
   Hardware Configuration Bits ----------------------------------------------*/
#pragma config FOSC = INTIO67
#pragma config PLLCFG = OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRTEN = OFF
#pragma config BOREN = ON
#pragma config BORV = 285
#pragma config WDTEN = OFF
#pragma config PBADEN = OFF
#pragma config LVP = OFF
#pragma config MCLRE = EXTMCLR

// Libraries ------------------------------------------------------------------
#include <p18f45k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usart.h>

// Constants  -----------------------------------------------------------------
#define TRUE 1
#define FALSE 0
#define SAMPLESIZE 10
#define ONESEC 10
#define SENSORS 3
#define TEMP 0
#define HUMID 1
#define CO2 2
#define ADCRES 0.0048828
#define TEMPOFFSET 0.5
#define TEMPCOEFF 0.01      // 10mv per degree temp
#define CO2COEFF 0.00034782 // 347.82uv per ppm
#define HUMIDCOEFF 0.05     // 50mv per degree percent(%)
#define HLHUMID 60
#define HLTEMP 35
#define HLCO2 1200
#define LLCO2 800
#define LLTEMP 15
#define LLHUMID 40
#define ONE TRUE
#define TWO 2
#define NINETYDEG 90
#define SIXDEG 6
#define SIXTYSIXDEG 66
#define NINEDEG 9
#define TWELVEDEG 12
#define FIVESECS 5
#define STOPCOUNT 6

// ADC REGISTERS DEFINES=============================================================
#define CHS ADCON0bits.CHS
#define ADON ADCON0bits.ADON
#define PVCFG ADCON1bits.PVCFG
#define NVCFG ADCON1bits.NVCFG
#define ACQT ADCON2bits.ACQT
#define ADFM ADCON2bits.ADFM
#define ADCS ADCON2bits.ADCS
#define TEMPM 0.0222
#define TEMPB 2.2222
#define ADCRESV 0.0049

// TIMER0 DEFINES==========================================================
#define T08BIT T0CONbits.T08BIT
#define T0CS T0CONbits.T0CS
#define PSA T0CONbits.PSA
#define T0PS T0CONbits.T0PS
#define TMR0ON T0CONbits.TMR0ON
#define TMR0IF INTCONbits.TMR0IF

// EUSART REGISTER DEFINES==========================================================================
#define BRG16 BAUDCON2bits.BRG16
#define BRGH TXSTA2bits.BRGH
#define TXEN TXSTA2bits.TXEN
#define SYNC TXSTA2bits.SYNC
#define SPEN RCSTA2bits.SPEN
#define CREN RCSTA2bits.CREN

// STEPPER DEFINES===========================================================================
#define THREEDEGREE 3
#define PATTERNCOUNT 4

// PB Defines==============================================================================
#define PBMASK 0xF0
#define STEPPERMASK 0x0F
#define NOPRESS 0xF0
#define READPBS (PORTA & PBMASK)
#define STEPPERPORT PORTB // Define a mask value for setting the stepper motor data lines.
#define MODE 0xE0
#define CHANNELSEL 0xD0
#define INCREMENT 0xB0
#define DECREMENT 0x70
#define LIGHT LATCbits.LATC0
#define COOLER LATCbits.LATC1
#define HEATER LATCbits.LATC2
#define FAN LATCbits.LATC3
#define SPRKLR LATCbits.LATC4
// Define the LEDs control for the Fan, Heater, Cooler (Chiller) and Lighting outputs.

// DEFINES FOR COMMUNICATION==========================================================
#define BUFSIZE 35
#define TOKENSIZE 3
#define CONTROLLER 1

#define ADDRESSTO 6
#define MYADDRESS 148
#define CMDSTATEMENT 8
#define LIMIT 9
#define CHANNEL 10
#define AVERAGEVAL 11
//==================================================================================

// Global Variables  ----------------------------------------------------------

char buf[BUFSIZE];
typedef int sensor_t;
typedef char flag_t;
char countTime = 0;
int countFiveSecs = 0;
int index, chId;
int sum = 0;
int stepperPattern[PATTERNCOUNT] = {0x01, 0x02, 0x04, 0x08};
// PushButton Global Variable Definitions=========================================
typedef struct
{
    char channelSelect; // a variable that holds the selected user sensor channel.
    char modeSelect;    // TRUE OR FALSE indicating that the High or Low limit of the selected channel.
    char pbState;       // The current read value from the pushbutton states.
    char lastState;     // The PB states read on the last sweep of read

} pb_t;
pb_t pbs;

// Sensor sample Global Variable Definitions=========================================

typedef struct
{
    sensor_t samples[SAMPLESIZE];
    float average;
    int insertPoint;
    int highLimit;
    int lowLimit;
    flag_t avgReady;
} sensorCh_t;
sensorCh_t sensors[SENSORS];

// PushButton Global Variable Definitions=========================================
typedef struct
{
    flag_t limitChange;
    flag_t sentenceRdy;
    char dispTiime;
} sysEvnt_t;

sysEvnt_t evtFlag;

// Stepper Global Variable Declarations=========================================
typedef struct
{
    char countPattern;
    char currentPosition;
    char currentPattern;
    char setPosition;
    flag_t isMoving;

} stepper_t;

stepper_t vent;

// Functions  -----------------------------------------------------------------

/*>>> updateDisplay: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used print out to the Serial port1
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void updateDisplay()
{
    printf("\033[2J\033[H Greenhouse Controller System (148)\n\r Channel:%i   Mode:%S \n\r Temp:%i%cC  Humid:%i%%  CO2:%i ppm\n\r HL:%i%cC  HL:%i%%  HL:%i ppm\n\r LL:%i%cC  LL:%i%%  LL:%i ppm\n\r ", pbs.channelSelect, pbs.modeSelect ? "High" : "Low", (int)(sensors[0].average), 248, (int)(sensors[1].average), (int)(sensors[2].average), sensors[0].highLimit, 248, sensors[1].highLimit, sensors[2].highLimit, sensors[0].lowLimit, 248, sensors[1].lowLimit, sensors[2].lowLimit);
    printf("\nHeat:%S Cool:%S Fan:%S \n\r Lights:%S SPKLR:%S\n\r", HEATER ? "ON" : "OFF", COOLER ? "ON" : "OFF", FAN ? "ON" : "OFF", LIGHT ? "ON" : "OFF", SPRKLR ? "ON" : "OFF");
    printf(" Vent\n\r setPosition:%i%cC CurrentPosition:%i%cC\n\r Data Pattern:0x%02X", (int)vent.setPosition, 248, (int)vent.currentPosition, 248, stepperPattern[vent.countPattern]);
    if (evtFlag.dispTiime <= FIVESECS)
    {
        puts2USART(buf);
        // printf("\n\r %s",buf);
    }

} // eo:updateDisplay::

/*>> > calculateCheckSum: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
char calculateCheckSum(char *ptr)
{
    char checkSum = 0;
    while (*ptr)
    {
        checkSum ^= *ptr;
        ptr++;
    }
    return checkSum;
} // eo calculateCheckSum::

/*>>> serialTransmitConfig: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to setup the serial ports for transmitting and reception
Input:         char fosc, used to set the baud rate using 8 or 4MHZ clocks
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void serialTransmitConfig(char fosc)
{
    // Set bits associated with TX/RX
    ANSELD = 0x00;
    TRISDbits.RD7 = 1;
    TRISDbits.RD6 = 1;

    // generate the baud rate for UART 9600Baud rate; 8MHZ
    BRG16 = 1; // 16-bit Baud Rate Generator is used (SPBRGHx:SPBRGx)
    BRGH = 0;  // low speed Asynchronous
    // for 9600 baudrate==========
    switch (fosc)
    {
    // for 4MHZ==========
    case 4:
        SPBRGH2 = (0x0C & 0XFF00); // 12
        SPBRG2 = (0x0C & 0X00FF);
        break;
    // for 4MHZ==========
    case 8:
        SPBRGH1 = 0x00; // 51
        SPBRG1 = 0x33;
        break;
    default:
        break;
    }

    TXEN = 1;
    SYNC = 0;
    SPEN = 1;
    CREN = 1;

} // eo serialTransmitConfig::

/*>>> set_osc: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to setup the PIC oscillator to your desired frequency(4 & 8MHZ only)
Input:         char clock, this is used to input the frequency value to set the oscillation frequency
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void set_osc(char clock)
{
    switch (clock)
    {
    case 4:
        OSCCON = 0x53;
        OSCCON2 = 0x04;
        OSCTUNE = 0x80;
        break;
    case 8:
        OSCCON = 0x63;
        OSCCON2 = 0x04;
        OSCTUNE = 0x80;
        break;
    default:
        break;
    }
    while (OSCCONbits.HFIOFS != 1)
        ; // wait for osc to become stable
} // eo: set_osc::

/*>>> configADC: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to configure the ADC for use
Input:         unsigned char adcs, to input conversion Clock Select bits
        char adon, to enable ADC conversion
        unsigned char acqTime, this is used to set configurre the TAD time for conversion
        char resultADFM,this input is used to set the rresult format
        char voltRef, used to set the positive voltage reference configuration bits
        char nVoltRef, used to set the Negative voltage reference configuration bits
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void configADC(unsigned char adcs, char adon, unsigned char acqTime, char resultADFM, char voltRef, char nVoltRef)
{
    // ADCON REG CONFIG===============================================
    ADON = 1;
    switch (voltRef) // positive voltage reference configuration bits
    {
    case 0:
        PVCFG = voltRef; // A/D Vref+ connected to internal signal AVDD
        break;
    case 1:
        PVCFG = voltRef; // A/D Vref+ connected to external pin, Vref+
        break;
    case 2:
        PVCFG = voltRef; // A/D Vref+ connected to internal signal FVR BUF2
        break;
    case 3:
        PVCFG = voltRef; // Reserved (by default A/D Vref+ connected to internal signal AVDD)
        break;
    default: // dont cares goes here
        break;
    }

    switch (nVoltRef) // Negative voltage reference configuration bits
    {
    case 0:
        NVCFG = nVoltRef; // A/D Vref- connected to internal signal AVSS
        break;
    case 1:
        NVCFG = nVoltRef; // A/D Vref- connected to external pin, Vref-
        break;
    case 2:
        NVCFG = nVoltRef; // Reserved (by default A/D Vref- connected to internal signal AVSS)
        break;
    case 3:
        NVCFG = nVoltRef; // Reserved (by default A/D Vref- connected to internal signal AVSS)
        break;
    default: // dont cares goes here
        break;
    }

    // ADCON2 REGISTER CONFIG =====================================
    switch (adcs) // A/D Conversion Clock Select bits
    {
    case 0: // FOSC/2
        ADCS = adcs;
        break;
    case 1: // FOSC/8
        ADCS = adcs;
        break;
    case 2: // FOSC/32
        ADCS = adcs;
        break;
    case 3: // FRC (clock derived from dedicated internal oscillator =600kHZ)
        ADCS = adcs;
        break;
    case 4: // FOSC/4
        ADCS = adcs;
        break;
    case 5: // FOSC/16
        ADCS = adcs;
        break;
    case 6: // FOSC/64
        ADCS = adcs;
        break;
    case 7: // FRC (clock derived from dedicated internal oscillator =600kHZ)
        ADCS = adcs;
        break;
    default:
        break;
    }
    // TAD TIME configuration=======================================================
    switch (acqTime)
    {
    case 0: // 0 TAD
        ACQT = 000;
        break;
    case 1: // 2 TAD
        ACQT = 001;
        break;
    case 2: // 4 TAD
        ACQT = 010;
        break;
    case 3: // 6 TAD
        ACQT = 011;
        break;
    case 4: // 8 TAD
        ACQT = 100;
        break;
    case 5: // 12 TAD
        ACQT = 101;
        break;
    case 6: // 16 TAD
        ACQT = 110;
        break;
    case 7: // 20 TAD
        ACQT = 111;
        break;
    default:
        break;
    }
    ////Result format selection bit=======================================================
    switch (resultADFM)
    {
    case 1:
        ADFM = 1; // A/D Conversion Result Format Select bit(Right Justified)
        break;
    case 0:
        ADFM = 0; // A/D Conversion Result Format Select bit(Right Justified)
        break;
    default:
        break;
    }
} // eo configADC::

/*>>> AdcResult: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function returns the result of ADC conversion
Input:         None
Returns:    ADRES, a 10-bit memory space where the result of conversion is stored
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
int adcResult(char channel)
{
    CHS = channel;
    ADCON0bits.GO = TRUE;
    while (ADCON0bits.GO)
        ;
    return ADRES;
} // eo AdcResult::

/*>>> resetTimer0: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        28/06/2023
Modified:    None
Desc:        To reset Timer0 to preset count and clear the Timer0 flag for next count for 100ms using 4mHz FCY
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void resetTimer0(void)
{
    TMR0H = 0x3C;
    TMR0L = 0xB0;
    TMR0IF = FALSE;

} // eo resetTimer0::

/*>> > timer0Config: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function sets up the Timer0 registers for use
Input : char enableTmr0, used to enable the timer0
        char t0Bit, used to select timer0 max count
        char t0cs,
        char pSa, to enable prescaler services
        char t0Ps, to select a prescalar value(0~256)
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void timer0Config(char enableTmr0, char t0Bit, char t0cs, char pSa, char t0Ps)
{
    T08BIT = t0Bit;
    T0CS = t0cs;
    PSA = pSa;
    T0PS = t0Ps;
    TMR0ON = enableTmr0;
} // eo timer0Config::

/*>>> portConfig: -----------------------------------------------------------
Author:        Francis Nzekwe
Date:        19/09/2023
Modified:    None
Desc:        This function configures the ports
Input:         None
Returns:    None
 ----------------------------------------------------------------------------*/
void portConfig()
{
    ANSELC = 0x00;
    TRISC = 0x00;
    LATC = 0x00;

    ANSELA = 0x07;
    TRISA = 0xE7;
    PORTA = 0xF0;

    ANSELB = 0x00;
    TRISB = 0xF0;
    LATB = 0xF0;

    ANSELD = 0x00;
    TRISD = 0xFF;
    LATD = 0xFF;

    ANSELE = 0x00;
    TRISE = 0xFF;
    LATE = 0xFF;
} // eo portConfig::

/*>>> initDSsensorCh: -----------------------------------------------------------
Author:        Nzekwe Francis
Date:        19/09/2023
Modified:    None
Desc:        This function is used to initialize SENSORS data members
Input:         sensorCh_t *dsptr, This data strcuture pointer is needed
            to address sensorCh_t type sensors of SENSORS size.
Returns:    None
 ----------------------------------------------------------------------------*/
void initDSsensorCh(sensorCh_t *dsptr)
{
    int index = 0, outIndex = 0;

    for (outIndex = 0; outIndex < SENSORS; outIndex++)
    {
        for (index = 0; index < SAMPLESIZE; index++)
        {
            dsptr->samples[index] = 0;
        }
        dsptr->average = 0;
        dsptr->insertPoint = 0;
        if (outIndex == 0)
        {
            dsptr->highLimit = HLTEMP;
            dsptr->lowLimit = LLTEMP;
        }
        else if (outIndex == ONE)
        {
            dsptr->highLimit = HLHUMID;
            dsptr->lowLimit = LLHUMID;
        }
        else if (outIndex == TWO)
        {
            dsptr->highLimit = HLCO2;
            dsptr->lowLimit = LLCO2;
        }
        dsptr->avgReady = FALSE;
        dsptr++;
    }
} // eo initDSsensorCh::

/*>>> initStepperMembers:: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to initialize the stepper members
Input:         stepper_t *stepperPointer, This pointer object is used to reference stepper data members
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void initStepperMembers(stepper_t *stepperPointer)
{
    stepperPointer->currentPosition = 0;
    stepperPointer->setPosition = 0;
    stepperPointer->currentPattern = 0x01;
    stepperPointer->countPattern = 0;
    stepperPointer->isMoving = FALSE;

} // eo initStepperMembers::

/*>>> moveStepperToPosition: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to move stepper to forward or backward after setting position
Input:         float position, this address is used to store the position that will be assigned to stepper
            stepper_t * motor,This pointer is used to address stepper objects(vent is one object)
            char status, this is used to update the status of the stepper to TRUE or FALSE
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void moveStepperToPosition(stepper_t *motor)
{

    if (motor->setPosition != motor->currentPosition)
    {
        if (motor->setPosition > motor->currentPosition)
        {
            motor->countPattern++;
            if (motor->countPattern >= PATTERNCOUNT)
            {
                motor->countPattern = 0;
            }
            motor->currentPosition += THREEDEGREE;
        }
        else if (motor->setPosition < motor->currentPosition)
        {
            motor->countPattern--;
            if (motor->countPattern < 0)
            {
                motor->countPattern = (PATTERNCOUNT - 1);
            }
            motor->currentPosition -= THREEDEGREE;
        }
    }
    motor->currentPattern = stepperPattern[motor->countPattern];
    STEPPERPORT = motor->currentPattern & STEPPERMASK;
} // eo moveStepperToPosition::

/*>>> initPbs: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to initialize the new pushbutton object
Input:       pb_t * pbPtr, used to address and assign initial values to each data memebr
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void initPbs(pb_t *pbPtr)
{
    pbPtr->channelSelect = 0;
    pbPtr->modeSelect = FALSE;
    pbPtr->pbState = 0;
    pbPtr->lastState = 0;
} // eo initPbs::

/*>>> toggleMode: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to Toggle between High or Low limit
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void toggleMode()
{
    pbs.modeSelect = !pbs.modeSelect;

} // eo toggleMode::

/*>>> changeChannel: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to Select Sensors (TEMP, HUMID and CO2)
Input:       None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void changeChannel()
{
    pbs.channelSelect++;
    if (pbs.channelSelect >= SENSORS)
    {
        pbs.channelSelect = 0;
    }
} // eo changeChannel::

/*>>> eventAction: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to generate a command sentence and is controlled by HighLimit and LowLimit push button trigger
Input:       None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void eventAction()
{
    if (evtFlag.limitChange)
    {
        evtFlag.limitChange = FALSE;
        if (pbs.modeSelect)
        {
            sprintf(buf, "$CONLIM,%i,%i,%i,H,%i", CONTROLLER, MYADDRESS, pbs.channelSelect, sensors[pbs.channelSelect].highLimit);
            sprintf(buf, "%s,%i \r", buf, calculateCheckSum(buf));
        }
        else
        {
            sprintf(buf, "$CONLIM,%i,%i,%i,L,%i", CONTROLLER, MYADDRESS, pbs.channelSelect, sensors[pbs.channelSelect].lowLimit);
            sprintf(buf, "%s,%i \r", buf, calculateCheckSum(buf));
        }
    }
    evtFlag.dispTiime = FALSE;

} // eo eventAction::

/*>>> increaseLimit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to Increase the setPoint of SENSORS (HighLimit and LowLimit)
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void increaseLimit()
{
    if (pbs.modeSelect)
    {
        sensors[pbs.channelSelect].highLimit++;
    }
    else
    {
        sensors[pbs.channelSelect].lowLimit++;
    }
    evtFlag.limitChange = TRUE;
    eventAction();

} // eo increaseLimit::

/*>>> decreaseLimit: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to Decrease the setPoint of SENSORS (HighLimit and LowLimit)
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void decreaseLimit()
{
    if (pbs.modeSelect)
    {
        sensors[pbs.channelSelect].highLimit--;
    }
    else
    {
        sensors[pbs.channelSelect].lowLimit--;
    }
    evtFlag.limitChange = TRUE;
    eventAction();
} // eo decreaseLimit::

/*--- MAIN: FUNCTION ----------------------------------------------------------
 ----------------------------------------------------------------------------*/
void main(void)
{
    evtFlag.dispTiime = FIVESECS;
    evtFlag.limitChange = 0;
    evtFlag.sentenceRdy = 0;
    set_osc(4);
    portConfig();
    initDSsensorCh(&sensors);
    initPbs(&pbs);
    serialTransmitConfig(4);
    initStepperMembers(&vent);
    configADC(4, 1, 5, 1, 0, 0);
    timer0Config(1, 0, 0, 0, 0);

    while (1)
    {
        pbs.pbState = READPBS;
        if (pbs.pbState != pbs.lastState)
        {
            pbs.lastState = pbs.pbState;
            switch (pbs.pbState)
            {
            case MODE:
                toggleMode();
                break;
            case CHANNELSEL:
                changeChannel();
                break;
            case INCREMENT:
                increaseLimit();
                break;
            case DECREMENT:
                decreaseLimit();
                break;
            default:
                break;
            }
        }
        else if (pbs.pbState == NOPRESS)
        {
            pbs.lastState = NOPRESS;
        }

        if (TMR0IF)
        {
            resetTimer0();
            countTime++;
            moveStepperToPosition(&vent);
            switch (countTime)
            {
            case ONESEC:
                countTime = 0;
                evtFlag.dispTiime++;
                if (evtFlag.dispTiime > FIVESECS)
                {
                    evtFlag.dispTiime = STOPCOUNT;
                }
                for (chId = 0; chId < SENSORS; chId++)
                {
                    sensors[chId].samples[sensors[chId].insertPoint] = adcResult(chId);
                    sensors[chId].insertPoint++;
                    if (sensors[chId].insertPoint >= SAMPLESIZE)
                    {
                        sensors[chId].insertPoint = 0;
                        sensors[chId].avgReady = TRUE;
                    }
                    if (sensors[chId].avgReady)
                    {
                        sum = 0;
                        for (index = 0; index < SAMPLESIZE; index++)
                        {
                            sum += sensors[chId].samples[index];
                        }
                        sensors[chId].average = (sum / SAMPLESIZE);
                    }
                    switch (chId)
                    {
                    case TEMP:
                        sensors[chId].average *= ADCRES;
                        sensors[chId].average -= TEMPOFFSET;
                        sensors[chId].average /= TEMPCOEFF;
                        if (sensors[chId].average > sensors[chId].highLimit)
                        {
                            COOLER = TRUE;
                            HEATER = FALSE;
                            FAN = TRUE;
                            vent.setPosition = NINETYDEG;
                        }
                        else if (sensors[chId].average < sensors[chId].lowLimit)
                        {
                            COOLER = FALSE;
                            HEATER = TRUE;
                            FAN = TRUE;
                            vent.setPosition = SIXDEG;
                        }
                        else
                        {
                            COOLER = FALSE;
                            HEATER = FALSE;
                            FAN = FALSE;
                        }
                        break;
                    case HUMID:
                        sensors[chId].average *= ADCRES;
                        sensors[chId].average /= HUMIDCOEFF;
                        if (sensors[chId].average > sensors[chId].highLimit)
                        {
                            SPRKLR = FALSE;
                            vent.setPosition = SIXTYSIXDEG;
                        }
                        else if (sensors[chId].average < sensors[chId].lowLimit)
                        {
                            SPRKLR = TRUE;
                            vent.setPosition = TWELVEDEG;
                        }
                        else
                        {
                            SPRKLR = FALSE;
                        }
                        break;
                    case CO2:
                        sensors[chId].average *= ADCRES;
                        sensors[chId].average /= CO2COEFF;
                        if (sensors[chId].average > sensors[chId].highLimit)
                        {
                            FAN = TRUE;
                            vent.setPosition = NINEDEG;
                        }
                        else if (sensors[chId].average < sensors[chId].lowLimit)
                        {
                            FAN = TRUE;
                            vent.setPosition = NINETYDEG;
                        }

                        break;
                    default:
                        break;
                    }
                } // eo CHid for loop
                updateDisplay();
            default:
                break;
            } // eo oneSec switch
        } // eo tmr0 overflow if
    } // eo while loop
}

/*
