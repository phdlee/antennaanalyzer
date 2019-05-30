/*
  Weaksignal TX for STM32F746 with si5351_HS Library
  kd8cec@gmail.com

  by KD8CEC
  -----------------------------------------------------------------------------
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <complex.h>
#include <string.h>
#include "config.h"
#include "LCD.h"
#include "font.h"
#include "touch.h"
#include "hit.h"
#include "dsp.h"
#include "gen.h"
#include "stm32f746xx.h"
#include "oslfile.h"
#include "stm32746g_discovery_lcd.h"
#include "match.h"
#include "num_keypad.h"
#include "screenshot.h"
//#include "smith.h"
//#include "measurement.h"
#include "panfreq.h"
#include "bitmaps/bitmaps.h"

#include "DS3231.h"

#include "si5351_hs.h"
#include "JTEncode.h"
#include "keyboard.h"
#include "gpio_control.h"

#define BACK_COLOR LCD_RGB(10, 52, 74)
#define SELECT_FREQ_COLOR LCD_RGB(255, 127, 39)
#define SELECT_PROTOCOL_COLOR LCD_RGB(255, 30, 30)


#include "audioirq.h"

//#define AUDIO_OUT_SIZE   ((uint32_t)(8000 * (0.68267) * 4))
//#define AUDIO_OUT_SIZE   21845
//#define AUDIO_OUT_SIZE   ((uint32_t)(8000 * (0.68267) * 2))   //just use full transfer irq
//#define AUDIO_OUT_SIZE   21845
#define AUDIO_OUT_SIZE   10923
//static int16_t __attribute__((section (".user_sdram"))) AUDIO_BUFFER_OUT[AUDIO_OUT_SIZE];
//int16_t *AUDIO_BUFFER_OUT = AUDIO_BUFFER_RAM;
static int16_t __attribute__((section (".user_sdram"))) AUDIO_BUFFER_OUT[AUDIO_OUT_SIZE];

#define WSPR_TONE_SPACING       1.4648          // ~1.46 Hz


extern void Sleep(uint32_t ms);
extern void TRACK_Beep(int duration);

extern void JTEncode(void); //for JTEncode converted C++ -> C
extern uint32_t GetInternTime(uint8_t *secondsx);   //at mainwind.c
//=============================================================================
//FREQUENCYS for Digital Mode
//By KD8CEC
//From WSJ-X Version 2.0
//-----------------------------------------------------------------------------
#define BAND_LENGTH 15
#define BAND_MAX_INDEX (BAND_LENGTH - 1)
const char* BAND_NAME[] =      {"2190", "630m", "160m",   "80m",   "40m",  "30m",  "20m",      "17m",   "15m",    "12m",    "10m",    "6m",     "4m",     "2m",       "XCVR"};
const uint32_t FREQS_WSPR[] =  {136000, 474200, 1836600, 3568600, 7038600, 10138700, 14095600, 18104600, 21094600, 24924600, 28124600, 50293000, 70091000, 144489000};
const uint32_t FREQS_FT8[] =   {136130, 474200, 1840000,  3573000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000, 70100000, 144174000};
const uint32_t FREQS_JT65[] =  {136130, 474200, 1838000,  3570000, 7076000, 10138000, 14076000, 18102000, 21076000, 24917000, 28076000, 50276000, 70102000, 144120000};
const uint32_t FREQS_JT9[] =   {136130, 474200, 1839000,  3572000, 7078000, 10140000, 14078000, 18104000, 21078000, 24919000, 28078000, 50312000, 70104000, 144120000};
const uint32_t FREQS_FT4[] =   {136130, 474200, 1839000,  3568000, 7047000, 10140000, 14080000, 18104000, 21140000, 24919000, 28180000, 50318000, 70104000, 144170000};

uint32_t WSFreq = 814074000;

//Draw ImageButton
//BOTTOM
#define MENU_EXIT         0
#define MENU_SAVECONFIG   1
#define MENU_SETCONTINUE  2
#define MENU_MSEND        3      //Probe Select Down
#define MENU_RTCSEND      4      //Probe Select Up

//RIGHT
#define MENU_BANDDOWN     5    //Prior Band
#define MENU_BANDUP       6      //Next Band
#define MENU_AFFREQDOWN   7  //-10Hz
#define MENU_AFFREQUP     8    //+10Hz

//FREQ SELECT MENU
#define MENU_TOP1         9
#define MENU_TOP2        10
#define MENU_TOP3        11
#define MENU_TOP4        12

#define wsMenus_Length   13

#define WSPR_AUDIO_VOLUME 70


const int wsMenus[wsMenus_Length][4] = {
#define FREQ_MENU_TOP  95
#define TOP_MENU_TOP    0
#define FREQ_INFO_TOP  80

//BOTTOM MENU (5)
{0,   237,     0 + 80,    280},    //Exit
{80,  237,    80 + 100,   280},    //Menu1
{180, 237,   180 + 100,   280},    //Menu2
{280, 237,   280 + 100,   280},    //Menu3
{380, 237,   380 + 100,   280},    //Menu4

//FREQ MENU (4)
{ 80, FREQ_MENU_TOP, 180, FREQ_MENU_TOP + 37},    //FREQ2 //Before Band
{180, FREQ_MENU_TOP, 280, FREQ_MENU_TOP + 37},    //FREQ3 //Next Band
{280, FREQ_MENU_TOP, 380, FREQ_MENU_TOP + 37},    //FREQ4 //-10Hz
{380, FREQ_MENU_TOP, 480, FREQ_MENU_TOP + 37},    //FREQ5 //+10Hz

//TOP MENU PROTOCOL (5)
{ 80, TOP_MENU_TOP, 180, TOP_MENU_TOP + 37},    //FREQ2 //WSPR
{180, TOP_MENU_TOP, 280, TOP_MENU_TOP + 37},    //FREQ3 //FT8
{280, TOP_MENU_TOP, 380, TOP_MENU_TOP + 37},    //FREQ4 //JT65
{380, TOP_MENU_TOP, 480, TOP_MENU_TOP + 37},    //FREQ5 //JT9

};

//uint8_t isOnAir = 1;

#define PR_WSPR 0
#define PR_FT8  1
#define PR_FT4  2
#define PR_JT65 3
#define PR_JT9  4

//uint8_t nowPR = PR_WSPR;  //
//uint8_t nowBandIndex = 4;

static char g_ws_data[59] = {0};
char *wsCallSign      = &g_ws_data[0];   //11
char *wsLocation      = &g_ws_data[12];   //9

uint8_t wsDBM   = 0;
char *wsDBMStr        = &g_ws_data[23];   //7
char *wsMessage       = &g_ws_data[31];   //21 = 52
uint8_t *nowPR        = &g_ws_data[53];
uint8_t *nowBandIndex = &g_ws_data[54];

uint8_t *wsTXPower    = &g_ws_data[55];
int16_t *wsAUDIOFreq  = (int16_t *)&g_ws_data[57];

char wsKeyboardTmp[21];

//void char *GetBandName()
//{

//}

#define WS_ST_NONE        0    //Not Status
#define WS_ST_READY       1    //RTC Send Ready
#define WS_ST_SENDMANUAL1 2    //Send By Manual
#define WS_ST_SENDMANUAL2 3    //Send By Manual and Continue (not support)
#define WS_ST_SENDRTC1    4    //Send by RTC
#define WS_ST_SENDRTC2    5    //Send By Manual and Continue

uint8_t NowStatus = WS_ST_NONE;


#include "ff.h"
static const char *g_ws_fpath = "/aa/wsignal.bin";

void WS_StoredInformatoin(void)
{
    FRESULT res;
    FIL fo = { 0 };
    res = f_open(&fo, g_ws_fpath, FA_OPEN_ALWAYS | FA_WRITE);
    if (FR_OK == res)
    {
        UINT bw;
        res = f_write(&fo, g_ws_data, sizeof(g_ws_data), &bw);
        res = f_close(&fo);
    }
}

void SetDefaultFrequency()
{
    if (*nowBandIndex == BAND_MAX_INDEX)
    {
        WSFreq = 0;
    }
    else
    {
        if (*nowPR == PR_WSPR)
            WSFreq = FREQS_WSPR[*nowBandIndex];
        else if (*nowPR == PR_FT8)
            WSFreq = FREQS_FT8[*nowBandIndex];
        else if (*nowPR == PR_JT65)
            WSFreq = FREQS_JT65[*nowBandIndex];
        else if (*nowPR == PR_JT9)
            WSFreq = FREQS_JT9[*nowBandIndex];
        else if (*nowPR == PR_FT4)
            WSFreq = FREQS_FT4[*nowBandIndex];
    }
}

#include "crash.h"

void WS_LoadInformation(void)
{
    FRESULT res;
    FIL fo = { 0 };

    FILINFO finfo;
    res = f_stat(g_ws_fpath, &finfo);
    if (FR_NOT_ENABLED == res || FR_NOT_READY == res)
        CRASH("No SD card");

    if (FR_OK == res)
    {
        res = f_open(&fo, g_ws_fpath, FA_READ);

        UINT br;
        f_read(&fo, g_ws_data, sizeof(g_ws_data), &br);
        f_close(&fo);

        wsDBM = atoi(wsDBMStr);

        if (*wsTXPower == 0)
        {
            //Default Value
            *wsTXPower = 0x08;
        }
    }
    else
    {
        *wsTXPower = 0x08;

        //*wsDBMStr        = &g_ws_data[23];   //7
        //*wsMessage       = &g_ws_data[31];   //21 = 52
        wsDBM         = 10;
        *nowPR        = PR_WSPR;  //WSPR
        *nowBandIndex = 4;  //40m band
    }

    SetDefaultFrequency();
}

//==============================================================================
//DISPLAY Protocol Information
//------------------------------------------------------------------------------
#define INFO_TOP   130
#define INFO_LINE2 183

//#define TITLE_COLOR LCD_RGB(0, 63, 119)
#define TITLE_COLOR LCD_RGB(2, 26, 39)

extern uint32_t RTCpresent;

uint8_t PR_Left_Time = 0;

static void CheckTime(void)
{
    short AMPM1;
    char text1[20];
    uint32_t nowTime;
    uint8_t tmpSecond1;
    uint8_t tmpMinute1;

    if(RTCpresent)
    {
        getTime(&nowTime, &tmpSecond1, &AMPM1, 0);
    }
    else
    {
        nowTime = GetInternTime(&tmpSecond1);
    }

    tmpMinute1 = nowTime % 100;
    sprintf(text1, "%02d:%02d:%02d ", nowTime / 100, nowTime % 100, tmpSecond1);

    //CLEAR Area
    LCD_FillRect(LCD_MakePoint(260, INFO_LINE2 + 12),LCD_MakePoint(477, INFO_LINE2 + 42), BACK_COLOR); //LCD_BLACK);

    //AUDIO Frequency
    LCD_FillRect(LCD_MakePoint(258, INFO_LINE2 + 1), LCD_MakePoint(479, INFO_LINE2 + 15), TITLE_COLOR); //LCD_BLACK);
    LCD_Rectangle(LCD_MakePoint(258, INFO_LINE2 + 1), LCD_MakePoint(478, INFO_LINE2 + 44), TITLE_COLOR);
    FONT_Write(FONT_FRAN, TextColor, 0, 268, INFO_LINE2, "CURRENT TIME & LEFT TIME");

    //SELECT_FREQ_COLOR
    FONT_Write(FONT_FRANBIG, TextColor, 0, 261, INFO_LINE2 + 12, text1);

    //Calculate LeftTime for Selected Protocol
    switch (*nowPR)
    {
        case PR_WSPR :  //even minute, 0 second
            tmpSecond1 = 60 - tmpSecond1;
            if (tmpMinute1 % 2)
                tmpMinute1 = 0;
            else
                tmpMinute1 = 1;
        break;
        case PR_FT8 :   //every minute, mod 15 = 0 second
            tmpMinute1 = 0;
            tmpSecond1 = 15 - tmpSecond1 % 15;
            if (tmpSecond1 >= 15)
                tmpSecond1 = 0;
        break;
        case PR_FT4 :   //every minute, mod 6 = 0 second
            tmpMinute1 = 0;
            tmpSecond1 = 6 - tmpSecond1 % 6;
            if (tmpSecond1 >= 6)
                tmpSecond1 = 0;
        break;
        case PR_JT65 :      //every minute, 0 second
        case PR_JT9 :      //every minute, 0 second
            tmpMinute1 = 0;
            tmpSecond1 = 60 - tmpSecond1;
            if (tmpSecond1 >= 60)
                tmpSecond1 = 0;
        break;
    }

    PR_Left_Time = tmpMinute1 * 60 + tmpSecond1;

    if (PR_Left_Time >= 120) //for WSPR
    {
        PR_Left_Time = 0;
    }

    //Trigger
    sprintf(text1, "%03d ", PR_Left_Time);

    FONT_Write_RightAlign(FONT_FRANBIG, TextColor, 0, 261, INFO_LINE2 + 12, 477, text1);
}

void SetWSStatus()
{
    //TOP MENU
    //On-Air Marked
    int32_t OnAirBackColor;
    int32_t OnAirForeColor;
    char str[10] = "";

    if (NowStatus == WS_ST_NONE)
    {
        OnAirBackColor = LCD_BLUE;
        OnAirForeColor = LCD_WHITE;
        sprintf(str, "Ready");
    }
    else if (NowStatus == WS_ST_READY)
    {
        OnAirBackColor = LCD_RGB(255, 102, 102);
        OnAirForeColor = LCD_WHITE;
        sprintf(str, "Wait...");
    }
    else
    {
        OnAirBackColor = LCD_RED;
        OnAirForeColor = LCD_WHITE;
        sprintf(str, "On air");
    }

    LCD_FillRect(LCD_MakePoint(0, TOP_MENU_TOP),LCD_MakePoint(77, TOP_MENU_TOP + 32), OnAirBackColor); //LCD_BLACK);
    FONT_Write(FONT_FRANBIG, OnAirForeColor, 0, 4, TOP_MENU_TOP, str);
}

//Show AF Frequency
void UpdateAFFreq(void)
{
    char str[10] = "";
    //Clear
    LCD_FillRect(LCD_MakePoint(128, INFO_LINE2 + 1), LCD_MakePoint(255, INFO_LINE2 + 43), BACK_COLOR);

    //AUDIO Frequency
    LCD_FillRect(LCD_MakePoint(128, INFO_LINE2 + 1), LCD_MakePoint(255, INFO_LINE2 + 15), TITLE_COLOR);
    LCD_Rectangle(LCD_MakePoint(128, INFO_LINE2 + 1), LCD_MakePoint(254, INFO_LINE2 + 44), TITLE_COLOR);
    FONT_Write(FONT_FRAN, TextColor, 0, 138, INFO_LINE2, "AF Freq");
    sprintf(str, "%d", 1500 + *wsAUDIOFreq);
    FONT_Write_RightAlign(FONT_FRANBIG, SELECT_FREQ_COLOR, 0, 128, INFO_LINE2 + 12, 250, str);
}

static void DrawWSInformation(void)
{
    char str[40] = "";

    uint32_t tempFreq = WSFreq;
    uint32_t freqMhs = tempFreq / 1000000;
    tempFreq = tempFreq % 1000000;
    uint32_t freqKhz = tempFreq / 1000;
    uint32_t freqHz = tempFreq % 1000;

    //Clear Frequency Window
    LCD_FillRect(LCD_MakePoint(4, 42),LCD_MakePoint(479, 90),BACK_COLOR);

    if (*nowBandIndex == BAND_MAX_INDEX)
    {
        uint32_t fore_freq_color = LCD_RED;
        //Display Inforamtion for Via Transceiver Mode
        if (*nowPR == PR_WSPR)
            sprintf(str, "Transmit via Transceiver/Use Phone Port");
        else
        {
            sprintf(str, "Available in WSPR mode only");
            fore_freq_color = LCD_GRAY;
        }

        FONT_Write(FONT_FRANBIG, fore_freq_color, BACK_COLOR, 4, 45, str);
    }
    else
    {
        //Frequency
        sprintf(str, "%u.%03u.%03u ", freqMhs, freqKhz, freqHz);

        //DISPLAY Result of Measure
        FONT_Write_RightAlign(FONT_BDIGITS, LCD_WHITE, BACK_COLOR, 4, 42, 475, str);
    }

    LCD_HLine(LCD_MakePoint(1, 36), 479, LCD_BLACK);
    LCD_VLine(LCD_MakePoint(1, 36),  57, LCD_BLACK);
    LCD_HLine(LCD_MakePoint(1, 92), 479, LCD_WHITE);
    LCD_VLine(LCD_MakePoint(479, 36),  57, LCD_WHITE);

    LCD_FillRect(LCD_MakePoint(0, FREQ_MENU_TOP),LCD_MakePoint(77, FREQ_MENU_TOP + 32), LCD_BLUE); //LCD_BLACK);
    FONT_Write(FONT_FRANBIG, LCD_WHITE, 0, 4, FREQ_MENU_TOP, BAND_NAME[*nowBandIndex]);


    SetWSStatus();

    //Clear
    LCD_FillRect(LCD_MakePoint(1, INFO_TOP),LCD_MakePoint(479, 237), BACK_COLOR); //LCD_BLACK);

    if (*nowPR == PR_WSPR)
    {
        //WSPR : Callsign, Location, dBm
        //wsCallSign
        LCD_FillRect(LCD_MakePoint(1, INFO_TOP + 1), LCD_MakePoint(255, INFO_TOP + 15), TITLE_COLOR); //LCD_BLACK);
        LCD_Rectangle(LCD_MakePoint(1, INFO_TOP + 1), LCD_MakePoint(254, INFO_TOP + 44), TITLE_COLOR);
        FONT_Write(FONT_FRAN, TextColor, 0, 14, INFO_TOP, "CALL SIGN ('_' Converted to '/')");
        FONT_Write(FONT_FRANBIG, TextColor, 0, 7, INFO_TOP + 12, wsCallSign);

        LCD_FillRect(LCD_MakePoint(258, INFO_TOP + 1),LCD_MakePoint(397, INFO_TOP + 15), TITLE_COLOR); //LCD_BLACK);
        LCD_Rectangle(LCD_MakePoint(258, INFO_TOP + 1), LCD_MakePoint(396, INFO_TOP + 44), TITLE_COLOR);
        FONT_Write(FONT_FRAN, TextColor, 0, 268, INFO_TOP, "LOCATION");
        FONT_Write(FONT_FRANBIG, TextColor, 0, 261, INFO_TOP + 12, wsLocation);

        LCD_FillRect(LCD_MakePoint(400, INFO_TOP + 1),LCD_MakePoint(479, INFO_TOP + 15), TITLE_COLOR); //LCD_BLACK);
        LCD_Rectangle(LCD_MakePoint(400, INFO_TOP + 1), LCD_MakePoint(478, INFO_TOP + 44), TITLE_COLOR);
        FONT_Write(FONT_FRAN, TextColor, 0, 410, INFO_TOP, "dBm");
        sprintf(str, "%d", wsDBM);
        FONT_Write(FONT_FRANBIG, TextColor, 0, 403, INFO_TOP + 12, str);
    }
    else
    {
        LCD_FillRect(LCD_MakePoint(1, INFO_TOP + 1), LCD_MakePoint(479, INFO_TOP + 15), TITLE_COLOR); //LCD_BLACK);
        LCD_Rectangle(LCD_MakePoint(1, INFO_TOP + 1), LCD_MakePoint(478, INFO_TOP + 44), TITLE_COLOR);
        FONT_Write(FONT_FRAN, TextColor, 0, 14, INFO_TOP, "TEXT MESSAGE for FT8, JT65, JT9");
        FONT_Write(FONT_FRANBIG, TextColor, 0, 7, INFO_TOP + 12, wsMessage);
    }


    //TX POWER
    LCD_FillRect(LCD_MakePoint(1, INFO_LINE2 + 1), LCD_MakePoint(125, INFO_LINE2 + 15), TITLE_COLOR); //LCD_BLACK);
    LCD_Rectangle(LCD_MakePoint(1, INFO_LINE2 + 1), LCD_MakePoint(124, INFO_LINE2 + 44), TITLE_COLOR);
    FONT_Write(FONT_FRAN, TextColor, 0, 14, INFO_LINE2, "TX Power(Port)");
    sprintf(str, "[S%d] %dmA", ((*wsTXPower & 0x10) == 0x10  ? 1 : 2), (*wsTXPower & 0x0F));
    FONT_Write(FONT_FRANBIG, TextColor, 0, 7, INFO_LINE2 + 12, str);

    UpdateAFFreq();

    CheckTime();
}

uint8_t isContinuousTX = 0;

void wsMenuDraw(void){
    uint32_t LCSaveColor = TextColor;

    //Bottom Menu
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_EXIT  ][BUTTON_LEFT], wsMenus[MENU_EXIT  ][BUTTON_TOP]), imgbtn_home2, imgbtn_home2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_SAVECONFIG  ][BUTTON_LEFT], wsMenus[MENU_SAVECONFIG  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_SETCONTINUE  ][BUTTON_LEFT], wsMenus[MENU_SETCONTINUE  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_MSEND  ][BUTTON_LEFT], wsMenus[MENU_MSEND  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_RTCSEND    ][BUTTON_LEFT], wsMenus[MENU_RTCSEND    ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);

    //Draw Text on Empty Button
    FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_SAVECONFIG][BUTTON_LEFT] + 10, wsMenus[MENU_SAVECONFIG][BUTTON_TOP] + 10, "Save Config");
    //FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_SETCONTINUE][BUTTON_LEFT] + 10, wsMenus[MENU_SETCONTINUE][BUTTON_TOP] + 10, isContinuousTX ? "Continuous TX" : "Once TX");
    //FONT_Write(FONT_FRAN, isContinuousTX ? LCD_BLUE : TextColor, 0, wsMenus[MENU_SETCONTINUE][BUTTON_LEFT] + 10, wsMenus[MENU_SETCONTINUE][BUTTON_TOP] + 10, "Continuous TX");
    FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_SETCONTINUE][BUTTON_LEFT] + 20, wsMenus[MENU_SETCONTINUE][BUTTON_TOP] + 10, "Continuous TX");
    LCD_FillRect(LCD_MakePoint(wsMenus[MENU_SETCONTINUE][BUTTON_LEFT] + 5, wsMenus[MENU_SETCONTINUE][BUTTON_TOP] + 8),
                 LCD_MakePoint(wsMenus[MENU_SETCONTINUE][BUTTON_LEFT] + 15, wsMenus[MENU_SETCONTINUE][BUTTON_TOP] + 27), isContinuousTX ? LCD_BLUE : LCD_RGB(50, 50, 50));

    FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, 0, wsMenus[MENU_MSEND][BUTTON_LEFT] + 10, wsMenus[MENU_MSEND][BUTTON_TOP] + 10, "Manual Send");
    FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, 0, wsMenus[MENU_RTCSEND][BUTTON_LEFT] + 10, wsMenus[MENU_RTCSEND][BUTTON_TOP] + 10, "RTC Send");

    //Freq Menu
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_BANDDOWN][BUTTON_LEFT], wsMenus[MENU_BANDDOWN][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_BANDUP][BUTTON_LEFT], wsMenus[MENU_BANDUP][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_AFFREQDOWN][BUTTON_LEFT], wsMenus[MENU_AFFREQDOWN][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_AFFREQUP][BUTTON_LEFT], wsMenus[MENU_AFFREQUP][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_BANDDOWN][BUTTON_LEFT] + 10, wsMenus[MENU_BANDDOWN][BUTTON_TOP] + 10, "Prior Band");
    FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_BANDUP][BUTTON_LEFT] + 10, wsMenus[MENU_BANDUP][BUTTON_TOP] + 10, "Next Band");
    FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_AFFREQDOWN][BUTTON_LEFT] + 10, wsMenus[MENU_AFFREQDOWN][BUTTON_TOP] + 10, "AF -10Hz");
    FONT_Write(FONT_FRAN, TextColor, 0, wsMenus[MENU_AFFREQUP][BUTTON_LEFT] + 10, wsMenus[MENU_AFFREQUP][BUTTON_TOP] + 10, "AF +10Hz");

    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_TOP1 ][BUTTON_LEFT], wsMenus[MENU_TOP1 ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_TOP2 ][BUTTON_LEFT], wsMenus[MENU_TOP2 ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_TOP3 ][BUTTON_LEFT], wsMenus[MENU_TOP3 ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_TOP4 ][BUTTON_LEFT], wsMenus[MENU_TOP4 ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);

    FONT_Write(FONT_FRANBIG, (*nowPR == PR_WSPR ? SELECT_PROTOCOL_COLOR : TextColor), 0, wsMenus[MENU_TOP1][BUTTON_LEFT] + 10, wsMenus[MENU_TOP1][BUTTON_TOP] + 0, "WSPR");
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_FT8 ? SELECT_PROTOCOL_COLOR : TextColor), 0, wsMenus[MENU_TOP2][BUTTON_LEFT] + 10, wsMenus[MENU_TOP2][BUTTON_TOP] + 0, "FT8");

    //Changed / Version 0.07,  Merge JT65 and JT9 buttons, Added FT4
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_FT4 ? SELECT_PROTOCOL_COLOR : TextColor), 0, wsMenus[MENU_TOP3][BUTTON_LEFT] + 10, wsMenus[MENU_TOP3][BUTTON_TOP] + 0, "FT4");
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_JT65 || *nowPR == PR_JT9 ? SELECT_PROTOCOL_COLOR : TextColor), 0, wsMenus[MENU_TOP4][BUTTON_LEFT] + 10, wsMenus[MENU_TOP4][BUTTON_TOP] + 0, *nowPR == PR_JT9 ? "JT9" : "JT65");

    /*
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_JT65 ? SELECT_PROTOCOL_COLOR : TextColor), 0, wsMenus[MENU_TOP3][BUTTON_LEFT] + 10, wsMenus[MENU_TOP3][BUTTON_TOP] + 0, "JT65");
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_JT9 ? SELECT_PROTOCOL_COLOR : TextColor), 0, wsMenus[MENU_TOP4][BUTTON_LEFT] + 10, wsMenus[MENU_TOP4][BUTTON_TOP] + 0, "JT9");
    */

    //LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_TOP5 ][BUTTON_LEFT], wsMenus[MENU_TOP5 ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    //LCD_DrawBitmap(LCD_MakePoint(wsMenus[MENU_TOP6 ][BUTTON_LEFT], wsMenus[MENU_TOP6 ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);

//#define SAVED_COLOR LCD_BLUE
//#define WORK_COLOR  LCD_YELLOW
//#define WORKED_COLOR  LCD_RED

}

LCDPoint pt;


void ClearTmpBuffer()
{
    for (int i = 0; i < 20; i++)
        wsKeyboardTmp[i] = ' ';
    wsKeyboardTmp[20] = 0;
}

void TmpBuffToBuff(char * targetBuff, int length)
{
    for (int i = 0; i < length; i++)
        targetBuff[i] = wsKeyboardTmp[i];

    targetBuff[length] = 0;
    for (int i = length -1; i >= 0; i--)
    {
        if (targetBuff[i] == ' ')
            targetBuff[i] = 0;
        else
            break;
    }
}

uint8_t wsUserStop = 0;

#define JT9_DELAY               576          // Delay value for JT9-1
#define JT65_DELAY              371          // Delay in ms for JT65A
#define JT4_DELAY               229          // Delay value for JT4A
#define WSPR_DELAY              683          // Delay value for WSPR
#define FSQ_2_DELAY             500          // Delay value for 2 baud FSQ
#define FSQ_3_DELAY             333          // Delay value for 3 baud FSQ
#define FSQ_4_5_DELAY           222          // Delay value for 4.5 baud FSQ
#define FSQ_6_DELAY             167          // Delay value for 6 baud FSQ
#define FT8_DELAY               159          // Delay value for FT8
#define FT4_DELAY               43           // Delay value for FT4


#define JT65_SYMBOL_COUNT                   126
#define JT9_SYMBOL_COUNT                    85
#define JT4_SYMBOL_COUNT                    207
#define WSPR_SYMBOL_COUNT                   162
#define FT8_SYMBOL_COUNT                    79
#define FT4_SYMBOL_COUNT                    103

//Not use , prevent flicking, reduce used stack
/*
int DrawSendingRateAndCheckStop(int progressPercent, int showProgress)
{
    char textPercent[7];

    //Progress Draw
    if (showProgress)
    {
        sprintf(textPercent, "%d%%", progressPercent);
        LCD_FillRect(LCD_MakePoint(261, INFO_LINE2 + 20), LCD_MakePoint(477, INFO_LINE2 + 40), BACK_COLOR); //LCD_BLACK);
        LCD_FillRect(LCD_MakePoint(268, INFO_LINE2 + 20), LCD_MakePoint(268 + progressPercent * 2, INFO_LINE2 + 40), LCD_BLUE); //LCD_BLACK);
        FONT_Write(FONT_FRANBIG, TextColor, 0, 340, INFO_LINE2 + 12, textPercent);
    }
    else
    {
        //Percent Draw
        //FONT_Write(FONT_FRANBIG, TextColor, BACK_COLOR, 400, INFO_LINE2 + 12, textPercent);
    }
    //LCD_FillRect(LCD_MakePoint(268, INFO_LINE2 + 25), LCD_MakePoint(268 + progressPercent, INFO_LINE2 + 30), LCD_BLUE); //LCD_BLACK);


    if (TOUCH_Poll(&pt))
    {
        if ((pt.y > wsMenus[MENU_MSEND][BUTTON_TOP]) && (pt.x > wsMenus[MENU_MSEND][BUTTON_LEFT]))
        {
            TRACK_Beep(1);
            //while(TOUCH_IsPressed());
            wsUserStop = 1;
            return 1;
        }
    }

    return 0;
}
*/


void CreateSineWave(double freq1, int16_t* buff, int buffLength)
{
	int samplesPerSecond = 16000;
	const double TAU = 2 * M_PI;
	int16_t volume = 16383;

	double theta1 = freq1 * TAU / (double)samplesPerSecond;
	//double theta2 = freq2 * TAU / (double)samplesPerSecond;
	double amp = volume; // so we simply set amp = volume / 2

	for (int step = 0; step < buffLength; step++)
	{
			short s1 = (short)(amp * (sin(theta1 * (double)step) + 1));
			buff[step] = (uint16_t)(s1);
	}
}

void SendingStart(void)
{
    char textPercent[10];
    uint8_t tx_buffer[255];
    uint8_t symbol_count;
    uint16_t tone_delay;
    uint16_t protocol_BW = 0;
    uint16_t protocol_DIV = 0;

    int progressPercent = 0;
    uint32_t tickstart  = 0;
    int nowStep = 0;

    //Apply Configuration
    /*
HS_CLK2
HS_SetPower(uint8_t clkNum, uint8_t txPower, uint8_t isApply)
    */
    uint8_t tx_output_Port = ((*wsTXPower & 0x10) == 0x10  ? HS_CLK0 : HS_CLK2);
    //uint8_t tx_output_Port = HS_CLK2;

    memset(tx_buffer, 0, 255);

    //PTT ON
    SET_PTT(1);

    switch(*nowPR)
    {
        case PR_WSPR :
            symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
            tone_delay = WSPR_DELAY;
            protocol_BW =   6;      //1.4648 * 4(4FSK) = 6Hz https://en.wikipedia.org/wiki/WSPR_(amateur_radio_software)
            protocol_DIV = 4;       //4 (FSK)

            wspr_encode(wsCallSign, wsLocation, wsDBM, tx_buffer);
        break;

        case PR_FT8 :
            symbol_count = FT8_SYMBOL_COUNT; // From the library defines
            tone_delay = FT8_DELAY;
            protocol_BW =   50;      //6.25hz * 8FSK = 50hz http://physics.princeton.edu/pulsar/k1jt/ft8.txt
            protocol_DIV = 8;       //8 (FSK)

            ft8_encode_msg(wsMessage, tx_buffer);	//Change to ft8_encode -> ft8_encode_msg
        break;

        case PR_FT4 :
            symbol_count = FT4_SYMBOL_COUNT; // From the library defines
            tone_delay = FT4_DELAY;
            protocol_BW =   90;     //from WSJT-X 2.1.0 RC & http://physics.princeton.edu/pulsar/k1jt/FT4_Protocol.pdf
            protocol_DIV = 4;       //4 (FSK) http://physics.princeton.edu/pulsar/k1jt/FT4_Protocol.pdf

            ft4_encode_msg(wsMessage, tx_buffer);	//Change to ft8_encode -> ft8_encode_msg
        break;

        case PR_JT65 :
            symbol_count = JT65_SYMBOL_COUNT; // From the library defines
            tone_delay = JT65_DELAY;
            //protocol_BW =   175;      //2.7hZ * 65 FSK = 175.5 https://www.sigidwiki.com/wiki/JT65
            //protocol_DIV =  65;       //65 (JT65A)
            protocol_BW =   189;      //2.7hZ * 65 FSK = 175.5 https://www.sigidwiki.com/wiki/JT65 modified for make divide = 2.7Hz
            protocol_DIV =  70;       //65 (JT65A)

            jt65_encode(wsMessage, tx_buffer);
        break;

        case PR_JT9 :
            symbol_count = JT9_SYMBOL_COUNT; // From the library defines
            tone_delay = JT9_DELAY;
            //protocol_BW =   16;      // 16 BandWidth https://en.wikipedia.org/wiki/WSJT_(amateur_radio_software)#JT9
            //protocol_DIV =  9;       //9 FSK
            protocol_BW =   14;      // 16 BandWidth https://en.wikipedia.org/wiki/WSJT_(amateur_radio_software)#JT9
            protocol_DIV =  8;       //9 FSK
            jt9_encode(wsMessage, tx_buffer);
        break;
    }

    unsigned long TX_P1, TX_P2, TX_P3;
	unsigned long TXA_P2;


	//=================================================================================
	// Test SI5351_HS
	//---------------------------------------------------------------------------------
	//3Khz(Step 1Hz) Swing Test (Using P1, P2)
	//Freq : 24100000Hz ~ 24103000Hz
	unsigned long tmp_P1, tmp_P2, tmp_P3;
	//Band and Protocol Frequency + Basic AF Frequency + Select Adjust AF Frequency
	unsigned long freqFrom = WSFreq + 1500 + *wsAUDIOFreq;  //WSPR, FT8, JT65, JT9 ALL USB => Basic Frequency + Audio Frequency
    unsigned int swingStep = 0;

    //Prepare for Sending Data
    if (*nowBandIndex == BAND_MAX_INDEX)
    {
        //Via Transceiver using Audio Chip
        uint32_t fore_freq_color = LCD_RED;
        //Display Inforamtion for Via Transceiver Mode
        if (*nowPR == PR_WSPR)
        {
            //BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 50, I2S_AUDIOFREQ_8K);
            if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, WSPR_AUDIO_VOLUME, I2S_AUDIOFREQ_8K) == 0)
            {
                //DBG_Printf("  AUDIO RECORD INIT OK  ");
            }
            else
            {
                //DBG_Printf("  AUDIO RECORD INIT FAIL");
                //DBG_Printf(" Try to reset board ");
            }

            memset(AUDIO_BUFFER_OUT, 0, AUDIO_OUT_SIZE * 2);
            Audio_Play_Status = AUDIO_TRANSFER_NONE;	//Receive Start
            BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

			//BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
        }
        else
        {
            return;
        }
    }
    else
    {
        //Via RF using Si5351 HS Libraty
        //FT8 6.25hz 8 = 50hz Bandwidth
        //Calculate Start
        HS_CalcPLLParam(HS_MAX_C_VAL, freqFrom, &TX_P1, &TX_P2, &TX_P3);

        //Calculate End
        HS_CalcPLLParam(HS_MAX_C_VAL, freqFrom + protocol_BW, &tmp_P1, &tmp_P2, &tmp_P3);

        //Swing Count (Increase or Decrease 1Hz)
        swingStep = (tmp_P2 - TX_P2)  / protocol_DIV;

        //All clock down
        HS_AllClockDown();

        //Set Start Frequency to PLL
        HS_ApplyPLLParam(HS_PLLA, TX_P1, TX_P2, TX_P3);

        //Out Frequncy to CLK Port (frequency is same with PLL)
        HS_BindCLKToPLL(tx_output_Port, HS_PLLA, freqFrom);

        //CLK Port Enabled & CLK UP
        //HS_SetClockEnabled(HS_CLK0, HS_TRUE, HS_TRUE);
        //HS_SetClockUp(HS_CLK0, HS_TRUE, HS_TRUE);
        HS_SetClockEnabled(tx_output_Port, HS_FALSE, HS_TRUE);
        HS_SetClockUp(tx_output_Port, HS_TRUE, HS_TRUE);
        HS_SetClockEnabled(tx_output_Port, HS_TRUE, HS_TRUE);

        HS_SetPower(tx_output_Port, HS_MAToParam(*wsTXPower & 0x0F), HS_TRUE);

    }

    //Button Text Change
    FONT_Write(FONT_FRAN, LCD_RED, LCD_RGB(87, 87, 87), wsMenus[MENU_MSEND][BUTTON_LEFT] + 10, wsMenus[MENU_MSEND][BUTTON_TOP] + 10, "  STOP TX   ");
    FONT_Write(FONT_FRAN, LCD_RED, LCD_RGB(87, 87, 87), wsMenus[MENU_RTCSEND][BUTTON_LEFT] + 10, wsMenus[MENU_RTCSEND][BUTTON_TOP] + 10, "   STOP TX   ");

    wsUserStop = 0;

    if (*nowBandIndex == BAND_MAX_INDEX)
    {
        //Draw Progress Window
        //Title Draw
        LCD_FillRect(LCD_MakePoint(258, INFO_LINE2 + 1), LCD_MakePoint(479, INFO_LINE2 + 43), BACK_COLOR); //LCD_BLACK);
        LCD_Rectangle(LCD_MakePoint(258, INFO_LINE2 + 1), LCD_MakePoint(478, INFO_LINE2 + 44), LCD_RED);

        FONT_Write(FONT_FRANBIG, LCD_RED, 0, 270, INFO_LINE2 + 12, "VIA TRANSCEIVER");

        //Text Progress for prevent flicking
        //LCD_FillRect(LCD_MakePoint(261, INFO_LINE2 + 20), LCD_MakePoint(477, INFO_LINE2 + 40), BACK_COLOR); //LCD_BLACK);
        //LCD_FillRect(LCD_MakePoint(268, INFO_LINE2 + 20), LCD_MakePoint(268 + progressPercent * 2, INFO_LINE2 + 40), LCD_BLUE); //LCD_BLACK);
        //FONT_Write(FONT_FRANBIG, TextColor, 0, 340, INFO_LINE2 + 12, textPercent);


        //Via Transceiver using Audio Chip
        nowStep = 0;
        //BSP_AUDIO_OUT_SetVolume(100);
        BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_OUT, AUDIO_OUT_SIZE * 2);    //

        while (1)
        {
            //Create SineWave
            CreateSineWave(((double)1500 + *wsAUDIOFreq) + (WSPR_TONE_SPACING * tx_buffer[nowStep]), AUDIO_BUFFER_OUT, AUDIO_OUT_SIZE);
            nowStep++;

            while(Audio_Play_Status != AUDIO_TRANSFER_COMPLETE)
            {
                HAL_Delay(1);
                __NOP();
            }

            Audio_Play_Status = AUDIO_TRANSFER_NONE;

            progressPercent = nowStep * 100 / 163;

            //Check Complete for WSPR protocol
            //if (DrawSendingRateAndCheckStop(progressPercent, 0) || nowStep >= 163)
            //Not use function
            sprintf(textPercent, "%d%%", progressPercent);
            LCD_FillRect(LCD_MakePoint(261, INFO_LINE2 + 20), LCD_MakePoint(477, INFO_LINE2 + 40), BACK_COLOR); //LCD_BLACK);
            LCD_FillRect(LCD_MakePoint(268, INFO_LINE2 + 20), LCD_MakePoint(268 + progressPercent * 2, INFO_LINE2 + 40), LCD_BLUE); //LCD_BLACK);
            FONT_Write(FONT_FRANBIG, TextColor, 0, 340, INFO_LINE2 + 12, textPercent);


            if (TOUCH_Poll(&pt))
            {
                if ((pt.y > wsMenus[MENU_MSEND][BUTTON_TOP]) && (pt.x > wsMenus[MENU_MSEND][BUTTON_LEFT]))
                {
                    TRACK_Beep(1);
                    wsUserStop = 1;
                }
            }


            if (wsUserStop || nowStep >= 163)
            {
                //Add below lines to Init section
                BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
                break;
            }
        }   //end of while

    }
    else
    {
        //Via Transceiver using Si5351 HS Library
        LCD_FillRect(LCD_MakePoint(258, INFO_LINE2 + 1), LCD_MakePoint(479, INFO_LINE2 + 15), LCD_RED); //LCD_BLACK);
        LCD_Rectangle(LCD_MakePoint(258, INFO_LINE2 + 1), LCD_MakePoint(478, INFO_LINE2 + 44), LCD_RED);
        FONT_Write(FONT_FRAN, TextColor, 0, 268, INFO_LINE2, "Transmission status");

        for(int i = 0; i < symbol_count; i++)
        {
            TXA_P2 = TX_P2 + tx_buffer[i] * swingStep;
            HS_SetPLLP1P2(HS_PLLA, TX_P1 + TXA_P2 / TX_P3, TXA_P2 % TX_P3, TX_P3);

            //Custom HAL_Delay Routine for reduce loss time
            //HAL_Delay(tone_delay);
            tickstart = HAL_GetTick(); //Start Dealy Time

            //========================================================
            //Idle Time Display Update
            //DisplaySendProgress(i + 1, symbol_count);
            progressPercent = i * 100 / symbol_count;

            //reduce stack
            //if (DrawSendingRateAndCheckStop(progressPercent, 1))
            //    break;
            //Not use function
            sprintf(textPercent, "%d%%", progressPercent);
            LCD_FillRect(LCD_MakePoint(261, INFO_LINE2 + 20), LCD_MakePoint(477, INFO_LINE2 + 40), BACK_COLOR); //LCD_BLACK);
            LCD_FillRect(LCD_MakePoint(268, INFO_LINE2 + 20), LCD_MakePoint(268 + progressPercent * 2, INFO_LINE2 + 40), LCD_BLUE); //LCD_BLACK);
            FONT_Write(FONT_FRANBIG, TextColor, 0, 340, INFO_LINE2 + 12, textPercent);

            if (TOUCH_Poll(&pt))
            {
                if ((pt.y > wsMenus[MENU_MSEND][BUTTON_TOP]) && (pt.x > wsMenus[MENU_MSEND][BUTTON_LEFT]))
                {
                    TRACK_Beep(1);
                    wsUserStop = 1;
                }
            }

            if (wsUserStop || nowStep >= 163)
                break;

            //Check DelayTime
            while ((HAL_GetTick() - tickstart) < tone_delay)
            {
            //__NOP();
            }
        }   //end of for

        HS_SetClockEnabled(tx_output_Port, HS_FALSE, HS_TRUE);
    }

    //while(TOUCH_IsPressed());
	//Update Time
	CheckTime();
    SET_PTT(0);
}

void WeakSignal_Proc(void)
{
    int checkCount = 0;     //Check Measure Interval

    SetColours();
    BSP_LCD_SelectLayer(0);
    LCD_FillAll(BACK_COLOR);
    BSP_LCD_SelectLayer(1);
    LCD_FillAll(BACK_COLOR);
    LCD_ShowActiveLayerOnly();

    WS_LoadInformation();

    while(TOUCH_IsPressed());

    uint32_t si5351_XTAL_FREQ = (uint32_t)((int)CFG_GetParam(CFG_PARAM_SI5351_XTAL_FREQ) + (int)CFG_GetParam(CFG_PARAM_SI5351_CORR));
    HS_SI5351_Init(si5351_XTAL_FREQ, (CFG_GetParam(CFG_PARAM_SI5351_CAPS) & 3));

    JTEncode();
    wsMenuDraw();           //Draw Menu
    DrawWSInformation();    //Draw Information

    //Init Start Frequency
    //NowMeasureFreq = MeasureStartFreq;

    for(;;)
    {
        if (TOUCH_Poll(&pt))
        {
            //Check Text (Callsign, TX dBm, Message, Loc)
            if ((pt.y > INFO_TOP && pt.y < INFO_LINE2) || (pt.y > INFO_LINE2 && pt.y < INFO_LINE2 + 35 && pt.x < 125))
            {
                TRACK_Beep(1);
                while(TOUCH_IsPressed());

                //Check Line 1
                if (pt.y < INFO_LINE2)
                {
                    if (*nowPR == PR_WSPR)
                    {
                        if (pt.x < 255)
                        {
                            ClearTmpBuffer();
                            if(KeyboardWindow(wsKeyboardTmp, 20, "Call Sign(_=/)"))
                            {
                                for (int i = 0; i < 20; i++)
                                {
                                    if (wsKeyboardTmp[i] == '_')
                                        wsKeyboardTmp[i] = '/';
                                }

                                TmpBuffToBuff(wsCallSign, 10);
                            }
                        }
                        else if (pt.x < 397)    //LOCATION
                        {
                            ClearTmpBuffer();
                            if(KeyboardWindow(wsKeyboardTmp, 20, "Enter Your Location"))
                            {
                                TmpBuffToBuff(wsLocation, 7);
                            }
                        }
                        else    //DB
                        {
                            ClearTmpBuffer();
                            if(KeyboardWindow(wsKeyboardTmp, 20, "dBm (only Numeric)"))
                            {
                                //Check Validataion
                                TmpBuffToBuff(wsDBMStr, 5);
                                wsDBM = atoi(wsDBMStr);
                            }

                        }
                    }   //end of WSPR
                    else
                    {   //FT8, JT65, JT9

                        ClearTmpBuffer();
                        if(KeyboardWindow(wsKeyboardTmp , 20, "Message for FT8, JT"))
                        {
                            TmpBuffToBuff(wsMessage, 20);
                        }
                    }

                }
                else
                {
                    //TX POWER SELECT
                    if ((*wsTXPower & 0x0F) == 0x02)   //2mA (minimum)
                        *wsTXPower = *wsTXPower == 0x02 ? 0x18 : 0x08;
                    else
                        *wsTXPower -= 2;
                }

                DrawWSInformation();
                continue;
            }

            int touchIndex = GetTouchIndex(pt, wsMenus, wsMenus_Length);

            //Play Beep
            if (touchIndex != -1)
            {
                TRACK_Beep(1);

                //Chbeck AF Frequency Up And Down / for Continues change
                if (touchIndex == MENU_AFFREQDOWN || touchIndex == MENU_AFFREQUP)  //+- 10Hz
                {
                    *wsAUDIOFreq += (touchIndex == MENU_AFFREQDOWN ? -10 : 10);
                    UpdateAFFreq();
                    Sleep(100);
                    continue;
                }
                else
                {
                    while(TOUCH_IsPressed());
                }
            }

            if (touchIndex == MENU_EXIT)    //EXIT
            {
                break;
            }
            else if (touchIndex == MENU_SAVECONFIG)
            {
                WS_StoredInformatoin();
            }
            else if (touchIndex == MENU_SETCONTINUE)
            {
                isContinuousTX = !isContinuousTX;
            }
            else if (touchIndex == MENU_MSEND)
            {
                NowStatus = WS_ST_SENDMANUAL1;
                SetWSStatus();

                SendingStart();

                NowStatus = WS_ST_NONE;
                SetWSStatus();

                while(TOUCH_IsPressed());
            }
            else if (touchIndex == MENU_RTCSEND)
            {
                if (WS_ST_READY == NowStatus)
                    NowStatus = WS_ST_NONE;
                else
                    NowStatus = WS_ST_READY;
            }
            else if (touchIndex == MENU_BANDDOWN || touchIndex == MENU_BANDUP)  //Prior Band, Next Band
            {
                if (touchIndex == MENU_BANDDOWN)
                {
                    if (*nowBandIndex == 0)
                        *nowBandIndex = BAND_MAX_INDEX;
                    else
                        (*nowBandIndex)--;
                }
                else
                {
                    if (*nowBandIndex == BAND_MAX_INDEX)
                        *nowBandIndex = 0;
                    else
                        (*nowBandIndex)++;
                }

                SetDefaultFrequency();

            }
            else if (touchIndex >= MENU_TOP1 && touchIndex <= MENU_TOP4)  //WSPR ~ JT9, Protocol Change
            {
                if (touchIndex == MENU_TOP4)
                {
                    if (*nowPR == PR_JT65)
                        *nowPR = PR_JT9;
                    else
                        *nowPR = PR_JT65;
                }
                else
                {
                    *nowPR = touchIndex - MENU_TOP1;
                }

                SetDefaultFrequency();
            }   //end of else if

            DrawWSInformation();
            wsMenuDraw();
        }

        Sleep(2);

        //Check for Timer
        if (checkCount++ > 100)
        {
            checkCount = 0;
            CheckTime();

            if (NowStatus == WS_ST_READY && PR_Left_Time == 0)
            {

                NowStatus = WS_ST_SENDRTC1;
                SetWSStatus();

                SendingStart();

                NowStatus = isContinuousTX && (! wsUserStop) ? WS_ST_READY : WS_ST_NONE;
                SetWSStatus();
                wsMenuDraw();

                while(TOUCH_IsPressed());
            }
        }
    }   //end of for

    //Release Memory
    //free(MeasureIM);
    //free(MeasureFreq);
    SET_PTT(0);
    GEN_SetMeasurementFreq(0);
    DSP_Init();
    return;

}
