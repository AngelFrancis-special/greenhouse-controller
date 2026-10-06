== == == == == == == = RECIEVING PROGRAM FOR MBED LPC1768 == == == == == == == == == == == == == ==
                       */
/*-----------------------------------------------------------------------------
    File Name:    GREENHOUSE CONTROLLER MBED.c
    Author:       Francis Nzekwe
    Date:          3/12/2023
    Modified:    None
    ? 

Description: This Program collects a message from the serial portx (PIC),
            validates it, parses it, checks that some conditions are met
            and prints it to the serial port 1.
*/

#include "mbed.h"
#include <cstring>
#include "cstring"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// GLOBAL DEFINES==================================================================================
#define BLINKING_RATE 500ms
#define BUFSIZE 35
#define LIMIT 4
#define CHANNEL 3
#define CMDSTATEMENT 0
#define ADDFROM 1
#define MYADDY 2
#define CONTROLLER 1
#define SENSORADDY 148
#define AVERAGEVAL 5
#define ADDYTO 2
#define SENSORS 3
#define TRUE 1
#define FALSE 0
#define SIZE 6
#define HLHUMID 60
#define HLTEMP 35
#define HLCO2 1200
#define LLCO2 800
#define LLTEMP 15
#define LLHUMID 40
#define ONE TRUE
#define TWO 2

                           UnbufferedSerial pc(USBTX, USBRX, 19200);
UnbufferedSerial pic(p9, p10, 19200);

// GLOBAL DEFINITION OF VARIABLES===================================================================
char *tokens[SIZE];
char conlim[] = {"CONLIM"};
char rcvBuf[BUFSIZE];
char *rcvPtr = rcvBuf;
typedef int sensor_t;
typedef char flag_t;

typedef struct
{
    float average;
    int highLimit;
    int lowLimit;
} sensorCh_t;

sensorCh_t sensors[SENSORS];
char SentenceRdy = FALSE;

// FUNCTIONS======================================================================

/*>>> initDSsensorCh: -----------------------------------------------------------
Author:      Nzekwe Francis
Date:        19/09/2023
Modified:    None
Desc:       This function is used to initialize SENSORS data members
Input:      sensorCh_t *dsptr, This data strcuture pointer is needed
            to address sensorCh_t type sensors of SENSORS size.
Returns:    None
 ----------------------------------------------------------------------------*/
void initDSsensorCh(sensorCh_t *dsptr)
{
    int index = 0, outIndex = 0;

    for (outIndex = 0; outIndex < SENSORS; outIndex++)
    {
        dsptr->average = 0;
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
        // dsptr->avgReady=FALSE;
        dsptr++;
    }
} // eo initDSsensorCh::

/*>>> updateDisplay: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Francis Nzekwe
Date:		01/07/2023
Modified:	None
Desc:		This function is used print to the display
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void updateDisplay()
{
    int sensorChannel = atoi(tokens[CHANNEL]);
    int limitVal = atoi(tokens[AVERAGEVAL]);
    char limitLetter = *tokens[LIMIT];
    char *command = tokens[0];
    char controller = atoi(tokens[1]);
    char address = atoi(tokens[2]);
    printf("\033[2J\033[H mbed:Greenhouse Monitor 148\n\rTemp:25%cC Humd:65%% CO2:880ppm\n\rHL:%i%cC HL:%i%% HL:%ippm\n\rLL:%i%cC LL:%i%% LL:%ippm", 248, sensors[0].highLimit, 248, sensors[1].highLimit, sensors[2].highLimit, sensors[0].lowLimit, 248, sensors[1].lowLimit, sensors[2].lowLimit);
    fflush(stdout);
} // eo display::

/*>> > getMessage: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function get a byte from the recieve buffer and saves it into a buffer
Input :none
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void getMessage()
{
    char tempVar = 0;
    pic.read(&tempVar, 1);

    switch (tempVar)
    {
    case '$':
        rcvPtr = rcvBuf;
        break;
    case '\r':
        tempVar = 0x00;
        SentenceRdy = TRUE;
        break;
    default:
        break;
    }
    *rcvPtr = tempVar;
    rcvPtr++;
} // eo getMessage::

/*>> > calculateCheckSum: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function calculates the checksum of the message
Input :char *ptr, this is a reference for the sentence/recieved sentence
Returns : char checksum, the calculated checksum value for the sentence
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

/*>> > validateMessage: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function is used to validate the received sentence.
Input :char *ptr, this is a reference for the recieved buffer.
Returns : True/False. true if the checksum value of the recieved buffer is equal to the calculated value of same recieved buffer
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

char validateMessage(char *ptr)
{
    int length = strlen(ptr);
    char newCheckSum = 0, recCheckSum = 0;
    while (*(ptr + length) != ',')
    {
        length--;
    }
    *(ptr + length) = 0x00;
    recCheckSum = atoi(ptr + length + 1);
    newCheckSum = calculateCheckSum(rcvBuf);
    if (recCheckSum == newCheckSum)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }

} // eo validateMessage::

/*>> > parseMessage: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function is used to split the messages into separate useable tokens
Input :char *ptr, this is a reference for the validated buffer.
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void parseMessage(char *ptr)
{
    int increase = 0;
    while (*ptr)
    {
        if ((*ptr == '$') || (*ptr == ','))
        {
            *ptr = 0x00;
            tokens[increase] = (ptr + 1);
            increase++;
        }
        ptr++;
    }
} // eo parseMessage::

/*>> > executeMessage: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function is used to excecute recieved sentence
Input : none
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void executeMessage()
{
    if (atoi(tokens[ADDYTO]) == SENSORADDY)
    {
        if (atoi(tokens[ADDFROM]) == CONTROLLER)
        {
            if (strcmp(conlim, tokens[CMDSTATEMENT]) == 0)
            {
                if (*tokens[LIMIT] == 'H')
                {
                    sensors[atoi(tokens[CHANNEL])].highLimit = atoi(tokens[AVERAGEVAL]);
                }
                else if (*tokens[LIMIT] == 'L')
                {
                    sensors[atoi(tokens[CHANNEL])].lowLimit = atoi(tokens[AVERAGEVAL]);
                }
            }
        }
    }
} // eo executeMessage::

int main()
{
    pic.attach(&getMessage, SerialBase::RxIrq);
    initDSsensorCh(sensors);
    while (true)
    {
        if (SentenceRdy)
        {
            SentenceRdy = FALSE;
            if (validateMessage(rcvBuf))
            {
                parseMessage(rcvBuf);
                executeMessage();
                updateDisplay();
            } // eo validatemessageCheck

        } // eo sentenceRdy
    } // eo while
} // eo main::
