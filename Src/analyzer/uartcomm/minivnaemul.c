/*
 *   KD8CEC
 *   kd8cec@gmail.com
 *
 *   this code not used, just experiment with Cellphone and VNA/J
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "aauart.h"
#include "dsp.h"
#include "config.h"
#include "ctype.h"
#include "stdlib.h"
#include "gen.h"

//==========================================================================================
// Serial remote control protocol state machine implementation (emulates RigExpert AA-170)
// Runs in main window only
//==========================================================================================
#define RXB_SIZE 64

extern void Sleep(uint32_t);
extern void DSP_Measure_DET(uint32_t freqHz, int applyErrCorr, int applyOSL, int nMeasurements);
/*
static char rxstr[RXB_SIZE+1];
static uint32_t rx_ctr = 0;
static char* rxstr_p;
static uint32_t _fCenter = 10000000ul;
static uint32_t _fSweep = 100000ul;
static const char *ERR = "ERROR\r";
static const char *OK = "OK\r";
static char* _trim(char *str)
{
    char *end;
    while(isspace((int)*str))
        str++;
    if (*str == '\0')
        return str;
    end = str + strlen(str) - 1;
    while(end > str && isspace((int)*end))
        end--;
    *(end + 1) = '\0';
    return str;
}

static void Wait_proc(void){

    Sleep(500);
}

static int PROTOCOL_RxCmd(void)
{
    int ch = AAUART_Getchar();
    if (ch == EOF)
        return 0;
    else if (ch == '\r' || ch == '\n')
    {
        // command delimiter
        if (!rx_ctr)
            rxstr[0] = '\0';
        rx_ctr = 0;
        return 1;
    }
    else if (ch == '\0') //ignore it
        return 0;
    else if (ch == '\t')
        ch = ' ';
    else if (rx_ctr >= (RXB_SIZE - 1)) //skip all characters that do not fit in the rx buffer
        return 0;
    //Store character to buffer
    rxstr[rx_ctr++] = (char)(ch & 0xFF);
    rxstr[rx_ctr] = '\0';
    return 0;
}

void PROTOCOL_Reset(void)
{
    rxstr[0] = '\0';
    rx_ctr = 0;
    //Purge contents of RX buffer
    while (EOF != AAUART_Getchar());
}
*/

int VnaComMode; //0:Reflection, 1 :Transmission
unsigned long int StartFreq;
unsigned int NumberF;
unsigned long int StepF;
//int VNAMode = 0;

char DecodeMiniVNA(void)
{
    int ch, i, err=0;
    char Param[11];

    ch = AAUART_Getchar();

    if (ch == EOF)
        return 0;

    switch (ch)
    {
        case '0':
            VnaComMode = 0;
            break;
        case '1':
            VnaComMode = 1;
            break;
        default:
            return 0;
    }

    //Read One more char
    ch = AAUART_Getchar();

    if  (ch !=0x0D)
        return 0;

    //COMMAND PROTOCOL MUST xx, 0x0d, xx 0x0d
    for (i = 0; i <= 10; i++)   //Read Start Frequency
    {
        ch = AAUART_Getchar();

        if (ch == 0x0D)
            break;
        if (isdigit(ch) == 1)    //Is Numeric Char
            Param[i] = ch;
        else        //Not Numeric
            return 0;
    }

    //Not Valid Length
    if  ((i==0) | (i>10))
        return 0;

    Param[i] = 0;   //Add String null For Convert

    StartFreq = atol(Param);

    //Read numbers of request point
    for (i = 0; i <= 5; i++)
    {
        ch = AAUART_Getchar();

        if (ch == 0x0D)   //End of command line
            break;

        if (isdigit(ch) ==1)
            Param[i] = ch;
        else
            return 0;
    }

    if  ((i==0) | (i>5))        //Not valid Length
        return 0;

    Param[i] = 0;
    NumberF = atoi(Param);

    //Step Count
    for (i = 0; i <= 10; i++)
    {
        ch = AAUART_Getchar();
        if (ch == 0x0D) break;

        if (isdigit(ch) ==1)
            Param[i] = ch;
        else
            return 0;
    }

    if  ((i==0) || (i>10))      //Not valid Length
        return 0;

    Param[i] = 0;                                // коне?строки

    StepF = atol(Param);

    return 1;
}

void PROTOCOL_Handler_VNA(void)
{
    int rst = DecodeMiniVNA();

    if (rst != 1)
    {
        return;
    }

#ifdef _DEBUG_UART
        DBG_Printf("REQUEST:%d, STARTF:%u, STEP:%u, NUMF:%u  ", VnaComMode, StartFreq, StepF, NumberF);
#endif

    //Process Mini VNA Protocol
    //if (StartFreq == 0 || StepF == 0 || NumberF == 0)
    //{
    //    return;
    //}

    uint32_t steps = 0;
    uint32_t fint;
    uint32_t fstep;
    unsigned char Temp;

    uint8_t returnVals[10];

    const float floatToADCOffset = 2;
    const float floatToADCScale = 1000;
    uint32_t adcamp = 0;
    uint32_t adcphs = 0;
    float mag1 = 0;
    float phase1 = 0;
    uint32_t chkFreq = 0;

    fint =StartFreq;

    for (int i = 0; i < NumberF; i++)
    {
        chkFreq = (uint32_t)(0.093132242 * ((float)fint));

        if (chkFreq < 100000 || chkFreq > 1000000000)
        {
            mag1 = 0;
            phase1 = 0;
            GEN_SetMeasurementFreq(0);
        }
        else
        {
            if (VnaComMode == 0)
            {
                DSP_Measure(chkFreq, 1, 0, CFG_GetParam(CFG_PARAM_MEAS_NSCANS));
            }
            else{
                DSP_Measure_DET(chkFreq, 1, 0, CFG_GetParam(CFG_PARAM_MEAS_NSCANS));
            }
            mag1 = DSP_MeasuredDiff();
            phase1 = DSP_MeasuredPhase();
        }

        //uint32_t adcamp = (crealf(rx) + floatToADCOffset) * floatToADCScale;
        //uint32_t adcphs = (cimagf(rx) + floatToADCOffset) * floatToADCScale;
        adcamp = (mag1 + floatToADCOffset) * floatToADCScale;
        adcphs = (phase1 + floatToADCOffset) * floatToADCScale;

        if (adcphs < 0)
            adcphs = 0;
        else if (adcphs >= 4096)
            adcphs = 4095;

        if (adcamp < 0)
            adcamp = 0;
        else if (adcamp >= 4096)
            adcphs = 4095;

        //DSP_RX rx = DSP_MeasuredZ();
        while(AAUART_IsBusy())
            Sleep(0); //prevent overwriting the data being transmitted

#ifdef _DEBUG_UART
//        sprintf(DebugBuff, "I:%d, Freq:%u, PH:%f, %u, MAG:%f, %u ",i, chkFreq, phase1, adcphs, mag1, adcamp);
//        DBG_Str(DebugBuff);
#endif

        returnVals[0] = 0;
        returnVals[1] = 0;
        returnVals[2] = 0;
        returnVals[3] = 0;
        returnVals[4] = 0;
        returnVals[5] = 0;

        Temp = (unsigned char)adcphs;
        returnVals[0] = Temp;
        adcphs = adcphs>>8;
        Temp = (unsigned char)adcphs;
        returnVals[1] = Temp;

        Temp = (unsigned char)adcamp;
        returnVals[2] = Temp;
        adcamp = adcamp>>8;
        Temp = (unsigned char)adcamp;
        returnVals[3] = Temp;

        AAUART_PutBytes(returnVals, 4);
        fint = fint + StepF;

        //sprintf(txstr, "%.6f,%.2f,%.2f\r", ((float)fint) / 1000000., crealf(rx), cimagf(rx));
        //AAUART_PutString(txstr);
    }

    /*
    if (!PROTOCOL_RxCmd())
        return;
    //Command received
    rxstr_p = _trim(rxstr);
    strlwr(rxstr_p);

    if (rxstr_p[0] == '\0') //empty command
    {
        AAUART_PutString(ERR);
        return;
    }

    if (0 == strcmp("ver", rxstr_p))
    {
        AAUART_PutString("AA-600 401\r");
        return;
    }

    if (0 == strcmp("on", rxstr_p))
    {
        GEN_SetMeasurementFreq(GEN_GetLastFreq());
        AAUART_PutString(OK);
        return;
    }

    if (0 == strcmp("off", rxstr_p))
    {
        GEN_SetMeasurementFreq(0);
        AAUART_PutString(OK);
        return;
    }

    if (rxstr_p == strstr(rxstr_p, "am"))
    {//Amplitude setting
        AAUART_PutString(OK);
        return;
    }

    if (rxstr_p == strstr(rxstr_p, "ph"))
    {//Phase setting
        AAUART_PutString(OK);
        return;
    }

    if (rxstr_p == strstr(rxstr_p, "de"))
    {//Set delay
        AAUART_PutString(OK);
        return;
    }

    if (rxstr_p == strstr(rxstr_p, "fq"))
    {
        uint32_t FHz = 0;
        if(isdigit((int)rxstr_p[2]))
        {
            FHz = (uint32_t)atoi(&rxstr_p[2]);
        }
        else
        {
            AAUART_PutString(ERR);
            return;
        }
        if (FHz < BAND_FMIN)
        {
            AAUART_PutString(ERR);
        }
        else
        {
            _fCenter = FHz;
            AAUART_PutString(OK);
        }
        return;
    }

    if (rxstr_p == strstr(rxstr_p, "sw"))
    {
        uint32_t sw = 0;
        if(isdigit((int)rxstr_p[2]))
        {
            sw = (uint32_t)atoi(&rxstr_p[2]);
        }
        else
        {
            AAUART_PutString(ERR);
            return;
        }
        _fSweep = sw;
        AAUART_PutString(OK);
        return;
    }

    if (rxstr_p == strstr(rxstr_p, "frx"))
    {
        uint32_t steps = 0;
        uint32_t fint;
        uint32_t fstep;

        if(isdigit((int)rxstr_p[3]))
        {
            steps = (uint32_t)atoi(&rxstr_p[3]);
        }
        else
        {
            AAUART_PutString(ERR);
            return;
        }
        if (steps == 0)
        {
            AAUART_PutString(ERR);
            return;
        }
        if (_fSweep / 2 > _fCenter)
            fint = 10;
        else
            fint = _fCenter - _fSweep / 2;
        fstep = _fSweep / steps;
        steps += 1;
        char txstr[64];
        while (steps--)
        {
            DSP_RX rx;

            DSP_Measure(fint, 1, 1, CFG_GetParam(CFG_PARAM_MEAS_NSCANS));
            rx = DSP_MeasuredZ();
            while(AAUART_IsBusy())
                Sleep(0); //prevent overwriting the data being transmitted
            sprintf(txstr, "%.6f,%.2f,%.2f\r", ((float)fint) / 1000000., crealf(rx), cimagf(rx));
            AAUART_PutString(txstr);
            fint += fstep;
        }
        AAUART_PutString(OK);

        return;
    }
    AAUART_PutString(ERR);
    */
}


