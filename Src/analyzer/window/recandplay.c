/*
  Record and Play for SSB QSO with PTT Control
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

//#define SLIDEBAR_COLOR LCD_RGB(100, 137, 240)
//#define SLIDEBAR_COLOR LCD_RGB(128, 142, 182)
#define SLIDEBAR_COLOR LCD_RGB(119, 128, 153)


#include "audioirq.h"

#define AUDIO_REC_BUFF_SIZE 1024
//int16_t *AUDIO_BUFFER_INOUT = AUDIO_BUFFER_RAM;
int16_t __attribute__((section (".user_sdram"))) AUDIO_BUFFER_INOUT[AUDIO_REC_BUFF_SIZE];

extern void Sleep(uint32_t ms);
extern void TRACK_Beep(int duration);

static const char *g_recplay_dir = "/aa/recplay";
static LCDPoint pt;


/*
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
*/

#define AUDIO_ST_NONE        0    //Not Status
#define AUDIO_ST_READY       1    //
#define AUDIO_ST_PLAY        2    //
#define AUDIO_ST_REC         3    //

uint32_t sndFileSizes[14] = {0};
int lastUpdateDisplayTime = 0;

static char g_rec_data[12] = {0};

uint8_t *isRepeatPlay        = &g_rec_data[1];
uint8_t *selectedSlotIndex   = &g_rec_data[2];
uint8_t *RecPlayStatus       = &g_rec_data[3];
uint8_t *RecVol              = &g_rec_data[4];
uint8_t *PlayVol             = &g_rec_data[5];
uint8_t *pttControl          = &g_rec_data[6];
uint8_t *rptDelayTime        = &g_rec_data[7];
uint8_t *playUserStop        = &g_rec_data[8];

/*
uint8_t (*isRepeatPlay) = 0;
uint8_t (*selectedSlotIndex) = 3;
uint8_t (*RecPlayStatus) = AUDIO_ST_NONE;
uint8_t (*RecVol) = 100;
uint8_t (*PlayVol) = 100;
uint8_t (*pttControl) = 1;   //
uint8_t (*rptDelayTime) = 76;  //100msec
uint8_t (*playUserStop) = 0;
*/



#include "ff.h"
//static const char *g_rec_fpath = "/aa/wsignal.bin";
static const char *g_rec_fpath = "/aa/recplay.bin";

void REC_StoredInformatoin(void)
{
    FRESULT res;
    FIL fo = { 0 };
    res = f_open(&fo, g_rec_fpath, FA_OPEN_ALWAYS | FA_WRITE);
    if (FR_OK == res)
    {
        UINT bw;
        res = f_write(&fo, g_rec_data, sizeof(g_rec_data), &bw);
        res = f_close(&fo);
    }
}

#include "crash.h"

void REC_LoadInformation(void)
{
    FRESULT res;
    FIL fo = { 0 };

    FILINFO finfo;
    res = f_stat(g_rec_fpath, &finfo);
    if (FR_NOT_ENABLED == res || FR_NOT_READY == res)
        CRASH("No SD card");

    if (FR_OK == res)
    {
        res = f_open(&fo, g_rec_fpath, FA_READ);

        UINT br;
        f_read(&fo, g_rec_data, sizeof(g_rec_data), &br);
        f_close(&fo);


        if (*RecVol > 100)
            *RecVol = 100;

        if (*PlayVol > 100)
            *PlayVol = 100;

        if (*selectedSlotIndex < 1 || *selectedSlotIndex > 14)
            *selectedSlotIndex = 1;

        if (*rptDelayTime > 200)
            *rptDelayTime = 200;
    }
    else
    {
        (*isRepeatPlay)      = 0;
        (*selectedSlotIndex) = 1;
        (*RecPlayStatus) = AUDIO_ST_NONE;
        (*RecVol)           = 100;
        (*PlayVol)          = 100;
        (*pttControl)       = 0;   //
        (*rptDelayTime)     = 50;  //50 * 200msec = 10sec
        (*playUserStop)     = 0;
    }

}

//==============================================================================
//DISPLAY Protocol Information
//------------------------------------------------------------------------------
#define INFO_TOP   130
#define INFO_LINE2 183

//#define TITLE_COLOR LCD_RGB(0, 63, 119)
#define TITLE_COLOR LCD_RGB(2, 26, 39)
/*

void SetWSStatus()
{
    //TOP MENU
    //On-Air Marked
    int32_t OnAirBackColor;
    int32_t OnAirForeColor;
    char str[10] = "";

    if ((*RecPlayStatus) == AUDIO_ST_NONE)
    {
        OnAirBackColor = LCD_BLUE;
        OnAirForeColor = LCD_WHITE;
        sprintf(str, "Ready");
    }
    else if ((*RecPlayStatus) == AUDIO_ST_READY)
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
*/


//Draw ImageButton
//BOTTOM
#define MENU_EXIT         0
#define MENU_SAVECONFIG   1
#define MENU_SETPTT  2
#define MENU_PLAY         3  //Probe Select Down
#define MENU_RECORD       4  //Probe Select Up

//RIGHT
#define MENU_BANDDOWN     5  //Prior Band
#define MENU_BANDUP       6  //Next Band
#define MENU_AFFREQDOWN   7  //-10Hz
#define MENU_AFFREQUP     8  //+10Hz

//FREQ SELECT MENU
#define MENU_SLOT1         9
#define MENU_SLOT2        10
#define MENU_SLOT3        11
#define MENU_SLOT4        12

#define MENU_SLOT5        13
#define MENU_SLOT6        14
#define MENU_SLOT7        15
#define MENU_SLOT8        16
#define MENU_SLOT9        17
#define MENU_SLOT10       18
#define MENU_SLOT11       19
#define MENU_SLOT12       20
#define MENU_SLOT13       21
#define MENU_SLOT14       22

#define recMenus_Length   23

#define WSPR_AUDIO_VOLUME 70


const int recMenus[recMenus_Length][4] = {
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
{ 80, FREQ_MENU_TOP, 80, FREQ_MENU_TOP + 37},    //FREQ2 //Before Band
{180, FREQ_MENU_TOP, 180, FREQ_MENU_TOP + 37},    //FREQ3 //Next Band
{280, FREQ_MENU_TOP, 280, FREQ_MENU_TOP + 37},    //FREQ4 //-10Hz
{380, FREQ_MENU_TOP, 380, FREQ_MENU_TOP + 37},    //FREQ5 //+10Hz

//TOP MENU PROTOCOL (5)
#define SHIFT_BOTTOM_SLOT 3
{280,  0 + SHIFT_BOTTOM_SLOT, 380,  33 + SHIFT_BOTTOM_SLOT},    //Slot1
{379,  0 + SHIFT_BOTTOM_SLOT, 480,  33 + SHIFT_BOTTOM_SLOT},    //Slot2

{280, 33 + SHIFT_BOTTOM_SLOT, 380, 65 + SHIFT_BOTTOM_SLOT},    //Slot3
{379, 33 + SHIFT_BOTTOM_SLOT, 480, 65 + SHIFT_BOTTOM_SLOT},    //Slot4

{280, 66 + SHIFT_BOTTOM_SLOT, 380, 97 + SHIFT_BOTTOM_SLOT},    //Slot5
{379, 66 + SHIFT_BOTTOM_SLOT, 480, 97 + SHIFT_BOTTOM_SLOT},    //Slot6

{280, 97 + SHIFT_BOTTOM_SLOT,  380, 128 + SHIFT_BOTTOM_SLOT},    //Slot7
{379, 97 + SHIFT_BOTTOM_SLOT,  480, 128 + SHIFT_BOTTOM_SLOT},    //Slot8

{280, 128 + SHIFT_BOTTOM_SLOT,  380, 159 + SHIFT_BOTTOM_SLOT},    //Slot9
{379, 128 + SHIFT_BOTTOM_SLOT,  480, 159 + SHIFT_BOTTOM_SLOT},    //Slot10

{280, 159 + SHIFT_BOTTOM_SLOT,  380, 190 + SHIFT_BOTTOM_SLOT},    //Slot11
{379, 159 + SHIFT_BOTTOM_SLOT,  480, 190 + SHIFT_BOTTOM_SLOT},    //Slot12

{280, 190 + SHIFT_BOTTOM_SLOT,  380, 221 + SHIFT_BOTTOM_SLOT},    //Slot11
{379, 190 + SHIFT_BOTTOM_SLOT,  480, 221 + SHIFT_BOTTOM_SLOT},    //Slot12
};

void GetTimeStrFromFilePos(uint32_t totalTimeMSec, char *str)
{
    int amiliSec = totalTimeMSec % 10;
    totalTimeMSec = totalTimeMSec / 10;
    int aSec = totalTimeMSec % 60 ;
    int aMin = totalTimeMSec / 60;
    sprintf(str, "%1d:%02d.%d", aMin, aSec, amiliSec);
}

void GetTimeStrByFileIndex(int fileIndex, char *str)
{
    if (fileIndex < 1 || fileIndex > 14)
        return;
    GetTimeStrFromFilePos((int)((float)sndFileSizes[fileIndex - 1] / 2048 / 3.125), str);
}

void recMenuDraw(void){
    char strBuff[10];
    char timeBuff[10];
    uint32_t LCSaveColor = TextColor;

    //Bottom Menu
    LCD_DrawBitmap(LCD_MakePoint(recMenus[MENU_EXIT  ][BUTTON_LEFT], recMenus[MENU_EXIT  ][BUTTON_TOP]), imgbtn_home2, imgbtn_home2_size);
    LCD_DrawBitmap(LCD_MakePoint(recMenus[MENU_SAVECONFIG  ][BUTTON_LEFT], recMenus[MENU_SAVECONFIG  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(recMenus[MENU_SETPTT  ][BUTTON_LEFT], recMenus[MENU_SETPTT  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(recMenus[MENU_PLAY  ][BUTTON_LEFT], recMenus[MENU_PLAY  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(recMenus[MENU_RECORD    ][BUTTON_LEFT], recMenus[MENU_RECORD    ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);

    //Draw Text on Empty Button
    FONT_Write(FONT_FRAN, TextColor, 0, recMenus[MENU_SAVECONFIG][BUTTON_LEFT] + 10, recMenus[MENU_SAVECONFIG][BUTTON_TOP] + 10, "Save Config");
    //FONT_Write(FONT_FRAN, (*isRepeatPlay) ? LCD_BLUE : TextColor, 0, recMenus[MENU_SETPTT][BUTTON_LEFT] + 10, recMenus[MENU_SETPTT][BUTTON_TOP] + 10, "Continuous TX");
    FONT_Write(FONT_FRAN, TextColor, 0, recMenus[MENU_SETPTT][BUTTON_LEFT] + 20, recMenus[MENU_SETPTT][BUTTON_TOP] + 10, "PTT Control");

    LCD_FillRect(LCD_MakePoint(recMenus[MENU_SETPTT][BUTTON_LEFT] + 5, recMenus[MENU_SETPTT][BUTTON_TOP] + 8),
                 LCD_MakePoint(recMenus[MENU_SETPTT][BUTTON_LEFT] + 15, recMenus[MENU_SETPTT][BUTTON_TOP] + 27), (*pttControl) ? LCD_BLUE : LCD_RGB(50, 50, 50));

    if ((*RecPlayStatus) == AUDIO_ST_NONE)
    {
        FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, 0, recMenus[MENU_PLAY][BUTTON_LEFT] + 10, recMenus[MENU_PLAY][BUTTON_TOP] + 10, "Play");
        FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, 0, recMenus[MENU_RECORD][BUTTON_LEFT] + 10, recMenus[MENU_RECORD][BUTTON_TOP] + 10, "Record");
    }
    else
    {
        FONT_Write(FONT_FRANBIG, (*RecPlayStatus) == AUDIO_ST_PLAY ? LCD_BLUE : LCD_RED, 0, recMenus[MENU_PLAY][BUTTON_LEFT] + 10, recMenus[MENU_PLAY][BUTTON_TOP] + 1, "STOP");
        FONT_Write(FONT_FRANBIG, (*RecPlayStatus) == AUDIO_ST_PLAY ? LCD_BLUE : LCD_RED, 0, recMenus[MENU_RECORD][BUTTON_LEFT] + 10, recMenus[MENU_RECORD][BUTTON_TOP] + 1, "STOP");
    }

    for (int i = MENU_SLOT1; i <= MENU_SLOT14; i++ )
    {
        int fileIndex = i - MENU_SLOT1 + 1;
        if (sndFileSizes[fileIndex - 1] != 0)
        {
            GetTimeStrByFileIndex(fileIndex, timeBuff);
            sprintf(strBuff, "[%2d] %s", fileIndex, timeBuff);
        }
        else
        {
            //sprintf(strBuff, "Slot%2d", fileIndex);
            sprintf(strBuff, "[%2d] Empty", fileIndex);
        }

        LCD_DrawBitmap(LCD_MakePoint(recMenus[i][BUTTON_LEFT], recMenus[i][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
        //FONT_Write(FONT_FRAN, ((*selectedSlotIndex) == fileIndex ? SELECT_PROTOCOL_COLOR : TextColor), 0, recMenus[i][BUTTON_LEFT] + 20, recMenus[i][BUTTON_TOP] + 10, strBuff);
        FONT_Write(FONT_FRAN, TextColor, 0, recMenus[i][BUTTON_LEFT] + 20, recMenus[i][BUTTON_TOP] + 10, strBuff);

        LCD_FillRect(LCD_MakePoint(recMenus[i][BUTTON_LEFT] + 5, recMenus[i][BUTTON_TOP] + 8),
            LCD_MakePoint(recMenus[i][BUTTON_LEFT] + 15, recMenus[i][BUTTON_TOP] + 27), ((*selectedSlotIndex) == fileIndex) ? LCD_RED : LCD_RGB(50, 50, 50));
    }

    LCD_Line(LCD_MakePoint(275, 2), LCD_MakePoint(275, 225), LCD_BLACK);
    LCD_Line(LCD_MakePoint(275, 2), LCD_MakePoint(479, 2), LCD_BLACK);
    LCD_Line(LCD_MakePoint(275, 230), LCD_MakePoint(479, 230), LCD_WHITE);

/*
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_WSPR ? SELECT_PROTOCOL_COLOR : TextColor), 0, recMenus[MENU_SLOT1][BUTTON_LEFT] + 10, recMenus[MENU_SLOT1][BUTTON_TOP] + 0, "WSPR");
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_FT8 ? SELECT_PROTOCOL_COLOR : TextColor), 0, recMenus[MENU_SLOT2][BUTTON_LEFT] + 10, recMenus[MENU_SLOT2][BUTTON_TOP] + 0, "FT8");
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_JT65 ? SELECT_PROTOCOL_COLOR : TextColor), 0, recMenus[MENU_SLOT3][BUTTON_LEFT] + 10, recMenus[MENU_SLOT3][BUTTON_TOP] + 0, "JT65");
    FONT_Write(FONT_FRANBIG, (*nowPR == PR_JT9 ? SELECT_PROTOCOL_COLOR : TextColor), 0, recMenus[MENU_SLOT4][BUTTON_LEFT] + 10, recMenus[MENU_SLOT4][BUTTON_TOP] + 0, "JT9");
*/
}

//Draw REC/PLAY Status Information
static void DrawStatusInfo(int justTimeUpdate, int totalTimeMSec)
{
    char str[10] = "";

    #define BACK_TIME_COLOR LCD_RGB(4, 26, 37)
    #define STATUS_COLOR_DISABLE LCD_RGB(57, 70, 76)
    //#define STATUS_COLOR_PLAY LCD_RGB(121, 129, 255)
    #define STATUS_COLOR_PLAY LCD_RGB(200, 250, 40)
    #define STATUS_COLOR_REC LCD_RGB(255, 100, 100)

    //Convert freame Count to 100Milisecond
    totalTimeMSec = (int)((float)totalTimeMSec / 3.125);

    if (justTimeUpdate)
    {
        if (lastUpdateDisplayTime == totalTimeMSec)
            return;

        LCD_FillRect(LCD_MakePoint(4, 37),LCD_MakePoint(270, 91), BACK_TIME_COLOR);
    }
    else
    {
        //Clear Frequency Window
        LCD_FillRect(LCD_MakePoint(2, 3),LCD_MakePoint(272, 91), BACK_TIME_COLOR);
        LCD_HLine(LCD_MakePoint(1, 2), 272, LCD_BLACK);
        LCD_VLine(LCD_MakePoint(1, 2),  90, LCD_BLACK);
        LCD_HLine(LCD_MakePoint(1, 92), 272, LCD_WHITE);
        LCD_VLine(LCD_MakePoint(272, 2),  90, LCD_WHITE);
        //(*RecPlayStatus) = AUDIO_ST_REC;
        FONT_Write(FONT_FRAN, (*RecPlayStatus) == AUDIO_ST_PLAY ? STATUS_COLOR_PLAY : STATUS_COLOR_DISABLE, BACK_TIME_COLOR, 5, 3, "PLAY");
        FONT_Write(FONT_FRAN, (*RecPlayStatus) == AUDIO_ST_REC ? STATUS_COLOR_REC : STATUS_COLOR_DISABLE, BACK_TIME_COLOR, 50, 3, "REC");

        sprintf(str, "%3d%%", (*PlayVol));
        FONT_Write(FONT_FRAN, (*RecPlayStatus) == AUDIO_ST_PLAY ? STATUS_COLOR_PLAY : STATUS_COLOR_DISABLE, BACK_TIME_COLOR, 3, 17, str);

        sprintf(str, "%3d%%", (*RecVol));
        FONT_Write(FONT_FRAN, (*RecPlayStatus) == AUDIO_ST_REC ? STATUS_COLOR_REC : STATUS_COLOR_DISABLE, BACK_TIME_COLOR, 43, 17, str);

        if (sndFileSizes[(*selectedSlotIndex) -1] == 0)
        {
            FONT_Write(FONT_FRAN, LCD_WHITE, BACK_TIME_COLOR, 100, 3, "Empty");
        }
        else
        {
            GetTimeStrByFileIndex((*selectedSlotIndex), str);
            FONT_Write(FONT_FRAN, LCD_WHITE, BACK_TIME_COLOR, 100, 3, str);

            sprintf(str, "%dKb", sndFileSizes[(*selectedSlotIndex) -1] / 1000);
            FONT_Write(FONT_FRAN, STATUS_COLOR_DISABLE, BACK_TIME_COLOR, 100, 17, str);
        }

        //PTT CONTROL
        FONT_Write(FONT_FRAN, (*pttControl) && ((*RecPlayStatus) == AUDIO_ST_PLAY) ? LCD_RED : STATUS_COLOR_DISABLE, BACK_TIME_COLOR, 170, 3, "PTT");

        uint32_t rptDrawColor = STATUS_COLOR_DISABLE;

        if ((*RecPlayStatus) == AUDIO_ST_PLAY)
            rptDrawColor = STATUS_COLOR_PLAY;
        else if ((*RecPlayStatus) == AUDIO_ST_READY)
            rptDrawColor = LCD_RED;
        //RPT CONTROL
        FONT_Write(FONT_FRAN, rptDrawColor, BACK_TIME_COLOR, 220, 3, "RPT");

        sprintf(str, "%3.1fs", (float)(*rptDelayTime) / 5);
        FONT_Write(FONT_FRAN, rptDrawColor, BACK_TIME_COLOR, 220, 17, str);
    }

    //if (lastUpdateDisplayTime != totalTimeMSec)
    //{
    lastUpdateDisplayTime = totalTimeMSec;
    GetTimeStrFromFilePos(lastUpdateDisplayTime, str);
    FONT_Write(FONT_BDIGITS, LCD_WHITE, BACK_TIME_COLOR, 4, 37, str);
    //}
}

#define BACK_SLIDE_COLOR LCD_RGB(12, 42, 58)

void DrawRepeatDelay(void)
{
    char str[50];
    //Draw Rectangle
    LCD_FillRect(LCD_MakePoint(1, 190),LCD_MakePoint(272, 230), BACK_SLIDE_COLOR);

    LCD_HLine(LCD_MakePoint(1, 190), 272, LCD_BLACK);
    LCD_VLine(LCD_MakePoint(1, 190),  40, LCD_BLACK);
    LCD_HLine(LCD_MakePoint(1, 230), 272, LCD_WHITE);
    LCD_VLine(LCD_MakePoint(272, 190),  40, LCD_WHITE);

    //Is Repeat
    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 3, 190, "Repeat");

    LCD_FillRect(LCD_MakePoint(3, 208),
                 LCD_MakePoint(40, 224), (*isRepeatPlay) ? LCD_RED : LCD_RGB(50, 50, 50));

    //(*rptDelayTime)
    sprintf(str, "Repeat Interval : %3.1f sec", (float)(*rptDelayTime) / 5);
    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 50, 190, str);
    LCD_FillRect(LCD_MakePoint(50, 208), LCD_MakePoint(50 + (*rptDelayTime), 224), SLIDEBAR_COLOR); //LCD_BLACK);
}

void DrawVolume(void)
{
    char str[50];
    //Draw Rectangle
    LCD_FillRect(LCD_MakePoint(1, 145),LCD_MakePoint(272, 185), BACK_SLIDE_COLOR);

    LCD_HLine(LCD_MakePoint(1, 145), 272, LCD_BLACK);
    LCD_VLine(LCD_MakePoint(1, 145),  40, LCD_BLACK);
    LCD_HLine(LCD_MakePoint(1, 230), 272, LCD_WHITE);
    LCD_VLine(LCD_MakePoint(272, 145),  40, LCD_WHITE);

    //Is Repeat
    sprintf(str, "Record Vol: %d%%", (*RecVol));
    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 3, 145, str);
    LCD_FillRect(LCD_MakePoint(3, 163), LCD_MakePoint(3 + (*RecVol), 179), SLIDEBAR_COLOR); //LCD_BLACK);

    sprintf(str, "Play Vol: %d%%", (*PlayVol));
    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 140, 145, str);
    LCD_FillRect(LCD_MakePoint(140, 163), LCD_MakePoint(140 + (*PlayVol), 179), SLIDEBAR_COLOR); //LCD_BLACK);
}

int CheckTouchOnOption(void)
{
    if (pt.x > 272 || pt.y > 230 || pt.y < 90)
    {
        return 0;
    }

    //while(TOUCH_IsPressed());

    if (pt.y > 190 && pt.y < 270 )  //Repeat Option
    {
        if (pt.x < 30)
        {
            TRACK_Beep(1);
            while(TOUCH_IsPressed());

            (*isRepeatPlay) =  ! (*isRepeatPlay);
        }
        else if (pt.x > 40)
        {
            //change Repeat Delay Time
            if (pt.x < 50)
                (*rptDelayTime) = 0;
            else if (pt.x > 250)
                (*rptDelayTime) = 200;
            else
                (*rptDelayTime) = pt.x - 50;

            HAL_Delay(100);
        }

        DrawRepeatDelay();
    }
    else if (pt.y > 150 && pt.y < 179 )  //Volume Control
    {
        if (pt.x < 3)           //Limit Record Volume
        {
            (*RecVol) = 0;
        }
        else if (pt.x > 240)    //Limit Play Voloume
        {
            (*PlayVol) = 100;
        }
        else if (pt.x < 110)    //Rec Volume Control
        {
            (*RecVol) = pt.x - 3;
            if ((*RecVol) > 100)
                (*RecVol) = 100;
        }
        else if (pt.x > 130)
        {
            if (pt.x < 140)
                (*PlayVol) = 0;
            else
                (*PlayVol) = pt.x - 140;
        }

        HAL_Delay(100);
        DrawVolume();
    }

    return 1;
    //DrawWSInformation();
}

int DrawRateAndCheckStop(int progressPercent, uint32_t progressColor, int playingTime)
{
    DrawStatusInfo(1, playingTime);
    LCD_FillRect(LCD_MakePoint(5, 85), LCD_MakePoint(5 + progressPercent * 2.5, 89), progressColor);

    //progressPercent
    if (TOUCH_Poll(&pt))
    {
        if ((pt.y > recMenus[MENU_PLAY][BUTTON_TOP]) && (pt.x > recMenus[MENU_PLAY][BUTTON_LEFT]))
        {
            TRACK_Beep(1);
            //while(TOUCH_IsPressed());
            (*playUserStop) = 1;
            return 1;
        }
    }

    return 0;
}


uint32_t GetFileSize(int fileIndex)
{
    FRESULT res;
    TCHAR path[64];

    sprintf(path, "%s/x%d.pcm", g_recplay_dir, fileIndex);
    //f_mkdir(g_recplay_dir);

    FILINFO finfo;
    res = f_stat(path, &finfo);

    if (FR_OK != res)
    {
        return 0;
    }
    else
    {
        return finfo.fsize;
    }
}

void UpdateFileSizes(int fileIndex)
{
    if (fileIndex == 0)
    {
        for (int i = 1; i <= 14; i++)
        {
            sndFileSizes[i -1] = GetFileSize(i);
        }
    }
    else
    {
        sndFileSizes[fileIndex -1] = GetFileSize(fileIndex);
    }
}


void StartRecord(void)
{
    FRESULT res;
    FIL fp;
    TCHAR path[64];
    int16_t REC_BUFF[512];

    //INIT DRIVER
    if (BSP_AUDIO_IN_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, 100, I2S_AUDIOFREQ_16K) != AUDIO_OK)
    {
        //CRASH("BSP_AUDIO_IN_Init failed");
        return;
    }

    //AUDIO_BUFFER_INOUT

    if ((*selectedSlotIndex) < 1 || (*selectedSlotIndex) > 14)
        return;

    sprintf(path, "%s/x%d.pcm", g_recplay_dir, (*selectedSlotIndex));
    f_mkdir(g_recplay_dir);

    res = f_open(&fp, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_OK != res)
    {
        return;
        //CRASHF("Failed to open file %s for write: error %d", path, res);
    }

    UINT bw;

    //Start Record
    BSP_AUDIO_IN_Record(AUDIO_BUFFER_INOUT, 1024);
    BSP_AUDIO_IN_SetVolume((*RecVol));
    int playTimeCnt = 0;

    int recordPosition = 0;
    while (1)
    {
        while(Audio_Status != AUDIO_TRANSFER_HALF)
        {
            HAL_Delay(1);
            //__NOP();
        }

        Audio_Status = AUDIO_TRANSFER_NONE;
        memcpy(REC_BUFF, AUDIO_BUFFER_INOUT, 512 * 2);
        res = f_write(&fp, REC_BUFF, sizeof(REC_BUFF), &bw);

        if (FR_OK != res || bw != sizeof(REC_BUFF))
            break;

        while(Audio_Status != AUDIO_TRANSFER_COMPLETE)
        {
            HAL_Delay(1);
            //__NOP();
        }

        Audio_Status = AUDIO_TRANSFER_NONE;
        memcpy(REC_BUFF, &AUDIO_BUFFER_INOUT[512], 512 * 2);
        res = f_write(&fp, REC_BUFF, sizeof(REC_BUFF), &bw);

        if (FR_OK != res || bw != sizeof(REC_BUFF))
            break;

        int recPercent = recordPosition++ * 100 / 8400 / 2;

        if (DrawRateAndCheckStop(recPercent, LCD_RED, ++playTimeCnt) || recPercent >= 100)
        {
            //Add below lines to Init section
            //BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
            BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
            break;
        }
    }   //end of while

    f_close(&fp);

    //Update File Size
    UpdateFileSizes((*selectedSlotIndex));

    recMenuDraw();           //Draw Menu
    DrawStatusInfo(0, 0);
    //DrawRepeatDelay();
    //DrawVolume();
    //File Save
}

void StartPlay(void)
{
    FRESULT res;
    FIL fp;
    TCHAR path[64];
    int16_t PLAY_BUFF[512];

    if ((*selectedSlotIndex) < 1 || (*selectedSlotIndex) > 14)
        return;

    sprintf(path, "%s/x%d.pcm", g_recplay_dir, (*selectedSlotIndex));
    f_mkdir(g_recplay_dir);

    FILINFO finfo;
    res = f_stat(path, &finfo);
    int pcmFileSize = finfo.fsize;

    int readSendCount = pcmFileSize / (512 * 2);

    res = f_open(&fp, path, FA_READ | FA_OPEN_EXISTING);
    if (FR_OK != res)
    {
        return;
    }

    //Start Record
    //BSP_AUDIO_IN_Record(AUDIO_BUFFER_INOUT, AUDIO_REC_BUFF_SIZE);
    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 100, I2S_AUDIOFREQ_16K);

    memset(AUDIO_BUFFER_INOUT, 0, 512 * 2 * 2);
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

    int playCount = 0;

    UINT br =  0;
    Audio_Play_Status = AUDIO_TRANSFER_NONE;	//Receive Start
    int playTimeCnt = 0;

    //Start Play
    //FileRead
    res = f_read(&fp, PLAY_BUFF, sizeof(PLAY_BUFF), &br);
    readSendCount;

    if (br != sizeof(PLAY_BUFF))
    {
        f_close(&fp);
        return;
    }

    //BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_INOUT, AUDIO_BUFFER_INOUT * 2);    //
    //0 ~ 512 Set
    memcpy(&AUDIO_BUFFER_INOUT[0], PLAY_BUFF, 512 * 2);
    BSP_AUDIO_OUT_SetVolume((*PlayVol));
    BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_INOUT, 512 * 2 * 2);    //
    //Play : 0 ~ 512

    int nowPostionCount = 1;

    while (1)
    {
        //FileRead
        res = f_read(&fp, PLAY_BUFF, sizeof(PLAY_BUFF), &br);

        if (br != sizeof(PLAY_BUFF))
            break;

        //BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_INOUT, AUDIO_BUFFER_INOUT * 2);    //
        //Set : 512 ~ 1024
        memcpy(&AUDIO_BUFFER_INOUT[512], PLAY_BUFF, 512 * 2);

        //complete : play 0~512
        while(Audio_Play_Status != AUDIO_TRANSFER_HALF)
        {
            //__NOP();
            HAL_Delay(1);
        }

        Audio_Play_Status = AUDIO_TRANSFER_NONE;


        //FileRead
        res = f_read(&fp, PLAY_BUFF, sizeof(PLAY_BUFF), &br);

        if (br != sizeof(PLAY_BUFF))
        {
            break;
        }

        //BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_INOUT, AUDIO_BUFFER_INOUT * 2);    //
        //set 0 ~ 512
        memcpy(&AUDIO_BUFFER_INOUT[0], PLAY_BUFF, 512 * 2);

        //complete : 512 ~ 1024
        while(Audio_Play_Status != AUDIO_TRANSFER_COMPLETE)
        {
            //__NOP();
            HAL_Delay(1);
        }

        Audio_Play_Status = AUDIO_TRANSFER_NONE;

        //int progressPercent = 50;
        nowPostionCount++;

        if (DrawRateAndCheckStop(nowPostionCount * 100  / readSendCount * 2, LCD_BLUE, ++playTimeCnt) )
        {
            //Add below lines to Init section
            //BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
            break;
        }
        else if (nowPostionCount >= readSendCount)
        {
            break;
        }

    }   //end of while

    f_close(&fp);
    BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
    //File Save
}


void RecPlayAudio_Proc(void)
{
    int checkCount = 0;     //Check Measure Interval

    SetColours();
    BSP_LCD_SelectLayer(0);
    LCD_FillAll(BACK_COLOR);
    BSP_LCD_SelectLayer(1);
    LCD_FillAll(BACK_COLOR);
    LCD_ShowActiveLayerOnly();

    REC_LoadInformation();
    while(TOUCH_IsPressed());


    UpdateFileSizes(0);
    recMenuDraw();           //Draw Menu
    DrawStatusInfo(0, 0);
    DrawRepeatDelay();
    DrawVolume();

    #define VERSION_FONT_COLOR LCD_RGB(59, 87, 100)

    FONT_Write(FONT_FRAN, VERSION_FONT_COLOR, 0, 40, 98, "Voice Keyer & Recorder for SSB,FM,AM");
    FONT_Write(FONT_FRAN, VERSION_FONT_COLOR, 0, 100, 115, "Version 0.5 + By KD8CEC");

    for(;;)
    {
        if (TOUCH_Poll(&pt))
        {
            //Check Text (Callsign, TX dBm, Message, Loc)
            if (CheckTouchOnOption())
            {
                continue;
            }

            int touchIndex = GetTouchIndex(pt, recMenus, recMenus_Length);

            //Play Beep
            if (touchIndex != -1)
            {
                TRACK_Beep(1);

                while(TOUCH_IsPressed());

                if (touchIndex == MENU_EXIT)    //EXIT
                {
                    break;
                }
                else if (touchIndex == MENU_SAVECONFIG)
                {
                    REC_StoredInformatoin();
                }
                else if (touchIndex == MENU_SETPTT)
                {
                    (*pttControl) = ! (*pttControl);
                }
                else if (touchIndex == MENU_PLAY)
                {
                    (*playUserStop) = 0;

                    (*RecPlayStatus) = AUDIO_ST_PLAY;
                    recMenuDraw();

                    while((*playUserStop) == 0)
                    {
                        if ((*pttControl))
                            SET_PTT(1);

                        (*RecPlayStatus) = AUDIO_ST_PLAY;
                        DrawStatusInfo(0, 0);
                        StartPlay();

                        if ((*pttControl))
                            SET_PTT(0);

                        (*RecPlayStatus) = AUDIO_ST_READY;
                        DrawStatusInfo(0, 0);

                        if ((! (*isRepeatPlay)) || (*playUserStop))
                            break;

                        //Delay RepeatTime
                        for (int i = 0; i < (*rptDelayTime); i++)
                        {
                            HAL_Delay(200);
                            if (DrawRateAndCheckStop((i + 1) * 100 / (*rptDelayTime), STATUS_COLOR_DISABLE, 0))
                            {
                                break;
                            }
                        }

                    }

                    (*RecPlayStatus) = AUDIO_ST_NONE;
                    DrawStatusInfo(0, 0);
                    recMenuDraw();

                    while(TOUCH_IsPressed());
                }
                else if (touchIndex == MENU_RECORD)
                {
                    (*RecPlayStatus) = AUDIO_ST_REC;
                    recMenuDraw();
                    DrawStatusInfo(0, 0);
                    StartRecord();
                    (*RecPlayStatus) = AUDIO_ST_NONE;
                    DrawStatusInfo(0, 0);
                    recMenuDraw();

                    while(TOUCH_IsPressed());
                }
                else if (touchIndex >= MENU_SLOT1 && touchIndex <= MENU_SLOT14)  //TOUCH SLOT NUMBER
                {
                    (*selectedSlotIndex) = touchIndex - MENU_SLOT1 + 1;
                    DrawStatusInfo(0, 0);
                }

                DrawStatusInfo(0, 0);
                recMenuDraw();
            }
        }

        Sleep(2);

    }   //end of for

    SET_PTT(0);
    GEN_SetMeasurementFreq(0);
    DSP_Init();
    return;

}
