/*
  LCR Meater for EU1KY's AA

  by KD8CEC
  kd8cec@gmail.com
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
#include "smith.h"
#include "measurement.h"
#include "panfreq.h"
#include "bitmaps/bitmaps.h"


#define BACK_COLOR LCD_RGB(10, 52, 74)
#define SELECT_FREQ_COLOR LCD_RGB(255, 127, 39)

extern void Sleep(uint32_t ms);
//==============================================================================

static uint8_t isMatch = 0;
static uint32_t meas_maxstep = 500000;

void OSL_CorrectLC(int freqIdx, float *phdif);
extern float GetLCCal(int freqIdx);
static float MeasMagDif;


extern int32_t OSL_IsLCCorrLoaded(void);

extern void LC_OSL_ScanShort(void);
extern void LC_OSL_ScanLoad(void);
extern void LC_OSL_ScanOpen(void);
extern void LC_OSL_Save(void);
extern int32_t LC_OSL_LoadFromFile(void);
extern const char* LC_OSL_GetSelectedName(void);

extern uint8_t lc_cal_status;

uint8_t LC_Mode = 0;    //0 :L, 1 :C

/*
#define LC_STEP1    100000
#define LC_STEP2    500000
#define LC_STEP3   1000000
#define LC_STEP4   5000000
#define LC_STEP5  10000000
#define LC_STEP6  15000000
#define LC_STEP7  30000000
*/

#define LC_STEP1    100000
#define LC_STEP2   5000000
#define LC_STEP3  10000000
#define LC_STEP4  15000000
#define LC_STEP5  20000000
#define LC_STEP6  25000000
#define LC_STEP7  30000000

#define LC_STEP   7
static const uint32_t LC_MEASURE_FREQS[LC_STEP] = {LC_STEP1, LC_STEP2, LC_STEP3, LC_STEP4, LC_STEP5, LC_STEP6, LC_STEP7};

//int GetLCMeasureFreq1(uint32_t targetIndex)
int GetLCStepFreq(uint32_t targetIndex)
{
    return LC_MEASURE_FREQS[targetIndex];
}

//This code from measurement.c
//DrawMeasureLC(rx, findedIndex, MeasureFreq[findedIndex]);
static void DrawMeasureLC(DSP_RX rx, int findedIndex, uint32_t findedFreq, int isFinded, float qualityValue, int isFirstDraw)
{
    #define PANEL_LEFT1   4
    #define PANEL_LEFT2 130
    char str[40] = "";

    float r= fabsf(crealf(rx));
    float im= cimagf(rx);
    float rp,xp,Cppf,Lpuh;
    if(r >0.05 ) rp=r+im*(im/r); else rp=10000.0;
    if(im*im>0.0025) xp=im+r*(r/im); else im=10000.0;
    /*
    if(parallel==1){
        r=rp;
        im=xp;
    }
    */

    if(r > 0.05)
        rp = r + im * (im / r);
    else
        rp=10000.0;

    if(im * im > 0.0025)
        xp = im + r * (r/im);
    else
        im=10000.0;

    //Clear Screen
    //LCD_FillRect(LCD_MakePoint(1, 62),LCD_MakePoint(260, 151),BACK_COLOR);
    LCD_FillRect(LCD_MakePoint(1, 89),LCD_MakePoint(270, 235), BACK_COLOR); //LCD_BLACK);

    //Display Function
    if (LC_Mode == 0)
        FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, BACK_COLOR, PANEL_LEFT1, 100, "MODE : L-Inductance");
    else
        FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, BACK_COLOR, PANEL_LEFT1, 100, "MODE : C-Capacitance");

    //Display Frequency
    sprintf(str, "Frequency: %.1f MHz", GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX)) / (float)1000000 );
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT2, 100, str);

    MeasMagDif = DSP_MeasuredDiffdB();
    sprintf(str, "Magnitude diff %.2f dB ", MeasMagDif);
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT1, 120, str);

    sprintf(str, "im: %.4f", im);
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT1, 140, str);

    sprintf(str, "Test Freq: %u Hz", findedFreq);
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT2, 140, str);
    //findedIndex, uint32_t findedFreq

    sprintf(str, "V-MiliVolt: %.5f", DSP_MeasuredMagVmv());
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT1, 160, str);

    sprintf(str, "I-MiliVolt: %.5f", DSP_MeasuredMagImv());
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT2, 160, str);


    if(r >= 5000.0f)
        sprintf(str, "Rs > 5k ");
    else if (r >= 999.5f)
        sprintf(str, "Rs:%.1f k ", r / 1000.0f);
    else if (r >= 99.5f)
        sprintf(str, "Rs: %.1f ", r);
    else
        sprintf(str, "Rs: %.2f ", r);

    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT1, 180, str);

    if (fabsf(im) > 999.5f)
        sprintf(str, "Xs:%.1f k", im / 1000.0f);
    else if (fabsf(im) >= 199.5f)
        sprintf(str, "Xs: %.1f", im);
    else
        sprintf(str, "Xs: %.2f", im);

    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, PANEL_LEFT2, 180, str);

//    FONT_ClearHalfLine(FONT_FRANBIG, BACK_COLOR, 62);

    float Luh = 1e6 * fabsf(im) / (2.0 * 3.1415926 * GEN_GetLastFreq());
    float Cpf = 1e12 / (2.0 * 3.1415926 * GEN_GetLastFreq() * fabs(im));

    sprintf(str, "L: %.7f", Luh);
    FONT_Write(FONT_FRAN, LC_Mode == 0 ? SELECT_FREQ_COLOR : TextColor, BACK_COLOR, PANEL_LEFT1, 200, str);

    sprintf(str, "C: %.7f", Cpf);
    FONT_Write(FONT_FRAN, LC_Mode == 1 ? SELECT_FREQ_COLOR : TextColor, BACK_COLOR, PANEL_LEFT2, 200, str);

    if (isFinded)
    {
        if (qualityValue > 990)
        {
            sprintf(str, "E:%3.3f", 999.999);
        }
        else
        {
            sprintf(str, "E:%3.3f", qualityValue);
        }
    }
    else
    {
        sprintf(str, "E: n/a");
    }
    LCD_FillRect(LCD_MakePoint(PANEL_LEFT2 + 120, 189),LCD_MakePoint(PANEL_LEFT2 + 245, 225),BACK_COLOR);
    //LCD_FillRect(LCD_MakePoint(1, 89),LCD_MakePoint(270, 235), BACK_COLOR); //LCD_BLACK);

    FONT_Write(FONT_FRANBIG, LCD_WHITE, BACK_COLOR, PANEL_LEFT2 + 120, 190, str);


    uint8_t isOutofRange = 0;
    if (LC_Mode == 0)   //L
    {
        if(Luh < 0 || im < 1.0)
        {
            Luh = 0;
            isOutofRange = 1;
        }


        if (qualityValue > 40)
        {
            sprintf(str, "[E] %.3f uH", Luh);
        }
        else
        {
            sprintf(str, "%.3f uH", Luh);
        }

        if (im > 5000.0f)
        {
            isOutofRange = 1;
        }
    }
    else    //C
    {
        if(Cpf < 0)
        {
            Cpf = 0;
        }

        if (qualityValue > 40)
        {
            sprintf(str, "[E] %.3f pF", Cpf);
        }
        else
        {
            sprintf(str, "%.3f pF", Cpf);
        }

        if (im > -1.0 || im < -5000.0f)
        {
            isOutofRange = 1;
        }

    }

    //BY KD8CEC
    //DISPLAY Result of Measure
    LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);

    if (isOutofRange == 1 || (! isFinded) || (qualityValue > 100))
    {
        FONT_Write(FONT_FRANBIG, LCD_RED, BACK_COLOR, 4, 40, "-- Out of Range --");
    }
    else
    {
        FONT_Write_RightAlign(FONT_BDIGITS, (qualityValue > 40) ? LCD_RED : LCD_WHITE, BACK_COLOR, 4, 40, 479, str);
    }

    LCD_HLine(LCD_MakePoint(0, 35), 479, LCD_BLACK);
    LCD_VLine(LCD_MakePoint(0, 35),  55, LCD_BLACK);
    LCD_HLine(LCD_MakePoint(0, 90), 479, LCD_WHITE);
    LCD_VLine(LCD_MakePoint(479, 35),  55, LCD_WHITE);
}


//Display Calibration Information
static void DisplayCalInfo()
{
    #define CALINFO_LEFT 271
    char str[40] = "";

    //Clear Screen
    LCD_FillRect(LCD_MakePoint(270, 89),LCD_MakePoint(378, 235), BACK_COLOR);

    if (OSL_IsErrCorrLoaded())
    {
        FONT_Write(FONT_FRAN, Color1, BACK_COLOR, CALINFO_LEFT, 100, "HW cal: OK");// WK
    }
    else
    {
        FONT_Write(FONT_FRAN, LCD_RED, BACK_COLOR, CALINFO_LEFT, 100, "HW cal: NO");// WK
    }

    //Display Name
    sprintf(str, "LC-Probe: %s", LC_OSL_GetSelectedName());
    FONT_Write(FONT_FRAN, SELECT_FREQ_COLOR, BACK_COLOR, CALINFO_LEFT, 120, str);

    //LC_OSL_GetSelectedName(), lc_cal_status
    if (OSL_IsLCCorrLoaded())
    {
        FONT_Print(FONT_FRAN, Color1, BACK_COLOR, CALINFO_LEFT, 140, "Probe cal: OK");
    }
    else
    {
        FONT_Print(FONT_FRAN, LCD_RED, BACK_COLOR, CALINFO_LEFT, 140, "Probe cal: NO");
    }
}

//static uint32_t fx = 14000000ul; //Scan range start frequency, in Hz
//static uint32_t fxkHz;//Scan range start frequency, in kHz
static BANDSPAN pBs1;

static void MEASUREMENT_Screenshot(void)
{
    char* fname = 0;
    fname = SCREENSHOT_SelectFileName();

     if(strlen(fname)==0) return;

    SCREENSHOT_DeleteOldest();
    Date_Time_Stamp();
    if (CFG_GetParam(CFG_PARAM_SCREENSHOT_FORMAT))
        SCREENSHOT_SavePNG(fname);
    else
        SCREENSHOT_Save(fname);
}


//Draw ImageButton
//BOTTOM
#define MENU_EXIT   0
#define MENU_LC     1
#define MENU_SNAP   2
#define MENU_PDOWN  3   //Probe Select Down
#define MENU_PUP    4   //Probe Select Up

//RIGHT
#define MENU_SHORT  5
#define MENU_LOAD   6
#define MENU_OPEN   7
#define MENU_APPLY  8
//#define MENU_SCREEN 9

//FREQ SELECT MENU
#define MENU_FREQ1   9
#define MENU_FREQ2  10
#define MENU_FREQ3  11
#define MENU_FREQ4  12
#define MENU_FREQ5  13
#define MENU_FREQ6  14

#define lcMenus_Length   15

const int lcMenus[lcMenus_Length][4] = {
#define RIGHT_MENU_TOP  90

//BOTTOM MENU
{0,   237,     0 + 80,    280},    //Exit
{80,  237,    80 + 100,   280},    //L / C
{180, 237,   180 + 100,   280},    //Screen Shot
{280, 237,   280 + 100,   280},    //Prior Probe
{380, 237,   380 + 100,   280},    //Next Probe

//RIGHT MENU
{380,   0 + RIGHT_MENU_TOP, 479,   0 + 35 + RIGHT_MENU_TOP},    //Set Short
{380,  37 + RIGHT_MENU_TOP, 479,  35 + 35 + RIGHT_MENU_TOP},    //Set Load
{380,  75 + RIGHT_MENU_TOP, 479,  70 + 35 + RIGHT_MENU_TOP},    //Set Open
{380, 111 + RIGHT_MENU_TOP, 479, 105 + 35 + RIGHT_MENU_TOP},    //Apply & Save

//FREQ MENU
{  0, 0,  82, 37},    //FREQ1
{ 82, 0, 164, 37},    //FREQ2
{164, 0, 244, 37},    //FREQ3
{244, 0, 316, 37},    //FREQ4
{316, 0, 398, 37},    //FREQ5
{398, 0, 479, 37},    //FREQ6

/*  //right align
{312, 244, 395, 273}, //FREQ
{397, 244, 480, 273}, //CAPTURE
*/
//{2, 244, 100, 260}, //ZOOM IN
};

static void lcMenuDraw(void){
    uint32_t LCSaveColor = TextColor;

    //Bottom Menu
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_EXIT  ][BUTTON_LEFT], lcMenus[MENU_EXIT  ][BUTTON_TOP]), imgbtn_home2, imgbtn_home2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_LC  ][BUTTON_LEFT], lcMenus[MENU_LC  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_SNAP  ][BUTTON_LEFT], lcMenus[MENU_SNAP  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_PDOWN  ][BUTTON_LEFT], lcMenus[MENU_PDOWN  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_PUP    ][BUTTON_LEFT], lcMenus[MENU_PUP    ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);

    //Draw Text on Empty Button
    FONT_Write(FONT_FRAN, TextColor, 0, lcMenus[MENU_LC][BUTTON_LEFT] + 10, lcMenus[MENU_LC][BUTTON_TOP] + 10, "Mode L/C");
    FONT_Write(FONT_FRAN, TextColor, 0, lcMenus[MENU_SNAP][BUTTON_LEFT] + 10, lcMenus[MENU_SNAP][BUTTON_TOP] + 10, "Screen Snapshot");
    FONT_Write(FONT_FRAN, TextColor, 0, lcMenus[MENU_PDOWN][BUTTON_LEFT] + 10, lcMenus[MENU_PDOWN][BUTTON_TOP] + 10, "Probe Prior");
    FONT_Write(FONT_FRAN, TextColor, 0, lcMenus[MENU_PUP][BUTTON_LEFT] + 10, lcMenus[MENU_PUP][BUTTON_TOP] + 10, "Probe Next");

    //Right Menu
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_SHORT ][BUTTON_LEFT], lcMenus[MENU_SHORT ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_LOAD  ][BUTTON_LEFT], lcMenus[MENU_LOAD  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_OPEN  ][BUTTON_LEFT], lcMenus[MENU_OPEN  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(lcMenus[MENU_APPLY ][BUTTON_LEFT], lcMenus[MENU_APPLY ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    //LCD_DrawBitmap(LCD_MakePoint(lcMenus[5][BUTTON_LEFT], lcMenus[5][BUTTON_TOP]), imgbtn_capture, imgbtn_capture_size);

#define SAVED_COLOR LCD_BLUE
#define WORK_COLOR  LCD_YELLOW
#define WORKED_COLOR  LCD_RED

    if (lc_cal_status == 0x80)  //Loaded Calibration
    {
        LCSaveColor = SAVED_COLOR;
    }

    FONT_Write(FONT_FRAN,    (lc_cal_status & 0x01 ? LCD_RED : lc_cal_status == 0x80 ?  SAVED_COLOR: WORK_COLOR), 0, lcMenus[MENU_SHORT][BUTTON_LEFT] + 10, lcMenus[MENU_SHORT][BUTTON_TOP] + 10, "SET SHORT");
    FONT_Write(FONT_FRAN, (lc_cal_status & 0x02 ? LCD_RED : lc_cal_status == 0x80 ? SAVED_COLOR : WORK_COLOR), 0, lcMenus[MENU_LOAD][BUTTON_LEFT] + 10, lcMenus[MENU_LOAD][BUTTON_TOP] + 10, "SET LOAD");
    FONT_Write(FONT_FRAN, (lc_cal_status & 0x04 ? LCD_RED : lc_cal_status == 0x80 ? SAVED_COLOR : WORK_COLOR), 0, lcMenus[MENU_OPEN][BUTTON_LEFT] + 10, lcMenus[MENU_OPEN][BUTTON_TOP] + 10, "SET OPEN");
    FONT_Write(FONT_FRAN, LCSaveColor, 0, lcMenus[MENU_APPLY][BUTTON_LEFT] + 10, lcMenus[MENU_APPLY][BUTTON_TOP] + 10, "Apply & Save");

    //FREQUENCY MENU
    //{100000, 500000, 1000000, 5000000, 10000000, 50000000, 100000000, 200000000};
    FONT_Write(FONT_FRANBIG, CFG_GetParam(CFG_PARAM_LC_INDEX) == 0 ? SELECT_FREQ_COLOR : Color1, BACK_COLOR,   5, 2, "0.1M");
    FONT_Write(FONT_FRANBIG, CFG_GetParam(CFG_PARAM_LC_INDEX) == 1 ? SELECT_FREQ_COLOR : Color1, BACK_COLOR,  87, 2, "5M");
    FONT_Write(FONT_FRANBIG, CFG_GetParam(CFG_PARAM_LC_INDEX) == 2 ? SELECT_FREQ_COLOR : Color1, BACK_COLOR, 169, 2, "10M");
    FONT_Write(FONT_FRANBIG, CFG_GetParam(CFG_PARAM_LC_INDEX) == 3 ? SELECT_FREQ_COLOR : Color1, BACK_COLOR, 249, 2, "15M");
    FONT_Write(FONT_FRANBIG, CFG_GetParam(CFG_PARAM_LC_INDEX) == 4 ? SELECT_FREQ_COLOR : Color1, BACK_COLOR, 320, 2, "20M");
    FONT_Write(FONT_FRANBIG, CFG_GetParam(CFG_PARAM_LC_INDEX) == 5 ? SELECT_FREQ_COLOR : Color1, BACK_COLOR, 402, 2, "25M");
}



extern void DSP_MeasureLC(uint32_t freqHz, int applyErrCorr, int applyOSL, int nMeasurements);
extern int32_t OSL_ChangeCurrentOSL(int oslIndex);
extern void TRACK_Beep(int duration);
extern int32_t LC_OSL_GetSelected(void);

uint32_t MeasureStartFreq = 0;
uint32_t MeasureEndFreq = 0;

float *MeasureIM; //Max 10Mhz per 1 Step
uint32_t *MeasureFreq;

#define MEASURE_INTERVAL 5 // * 100 ms
void Measure_LCR_Proc(void)
{
    //float MeasureIM[100] = {0}; //Max 10Mhz per 1 Step
    //uint32_t MeasureFreq[100] = {0};
    int i, l, j=0;

    int firstDraw = 1;

    DSP_RX rx;
    LCDPoint pt;

    int checkCount = 0;     //Check Measure Interval
    int NowMesaureIndex = 0;
    uint32_t NowMeasureFreq = MeasureStartFreq;

    MeasureIM = (float *)malloc(sizeof(float) * 55); //Max 10Mhz per 1 Step
    MeasureFreq = (uint32_t *)malloc(sizeof(uint32_t) * 55);

    //Check Range
    if (CFG_GetParam(CFG_PARAM_LC_INDEX) < 0 || CFG_GetParam(CFG_PARAM_LC_INDEX) > 6)
    {
        CFG_SetParam(CFG_PARAM_LC_INDEX, 5);    //Default Value
        MeasureStartFreq = GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX));
        MeasureEndFreq = GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX) + 1);

        CFG_Flush();
    }

    if (LC_OSL_GetSelected() == -1)
    {
        CFG_SetParam(CFG_PARAM_LC_OSL_SELECTED, 0);    //Default Value, Cal File Name : 'A'
        CFG_Flush();
    }

    LC_OSL_LoadFromFile();
    OSL_CorrectZ(BAND_FMIN, 0+0*I); //To force lazy loading OSL file if it has not been loaded yet


    isMatch = 0;
    SetColours();
    BSP_LCD_SelectLayer(0);
    LCD_FillAll(BACK_COLOR);
    BSP_LCD_SelectLayer(1);
    LCD_FillAll(BACK_COLOR);
    LCD_ShowActiveLayerOnly();


    //Set Default Start, End Frequency
    MeasureStartFreq = GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX));
    MeasureEndFreq = GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX) + 1);

    //Load saved middle frequency value from BKUP registers 2, 3
    //to MeasurementFreq
    while(TOUCH_IsPressed());

    LCD_FillRect(LCD_MakePoint(0,0), LCD_MakePoint(479,271), BACK_COLOR); // Graph rectangle
    FONT_Write(FONT_FRANBIG, TextColor, BACK_COLOR, 120, 60, "L/C Measure Mode");
    Sleep(1000);

    LCD_FillRect(LCD_MakePoint(120,60), LCD_MakePoint(479,140), BACK_COLOR); // Graph rectangle

    FONT_Write(FONT_FRANBIG, CurvColor, BACK_COLOR, 6, 236, " Exit");
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, 92, 250, "Set frequency");
    FONT_Write(FONT_FRAN, TextColor, BACK_COLOR, 182, 250, "Save snapshot ");

    lcMenuDraw();   //Draw Menu
    DisplayCalInfo();

    //Init Start Frequency
    NowMeasureFreq = MeasureStartFreq;

//MEASUREMENT_REDRAW:
    for(;;)
    {
        if (TOUCH_Poll(&pt))
        {
            int touchIndex = GetTouchIndex(pt, lcMenus, lcMenus_Length);
            int fChanged = 0;

            //Play Beep
            if (touchIndex != -1)
            {
                TRACK_Beep(1);
                while(TOUCH_IsPressed());
            }

            if (touchIndex == MENU_EXIT)    //EXIT
            {
                break;
            }
            else if (touchIndex == MENU_LC)
            {
                LC_Mode = LC_Mode == 0 ? 1 : 0;
            }
            else if (touchIndex == MENU_SNAP)
            {
                MEASUREMENT_Screenshot();
            }
            else if (touchIndex == MENU_PDOWN || touchIndex == MENU_PUP)
            {
                int nowCalIndex = LC_OSL_GetSelected();

                if (touchIndex == MENU_PDOWN)
                {
                    if (nowCalIndex > 0)
                    {
                        nowCalIndex--;
                    }
                }
                else
                {
                    if (nowCalIndex < 25)
                    {
                        nowCalIndex++;
                    }
                }

                CFG_SetParam(CFG_PARAM_LC_OSL_SELECTED, nowCalIndex);    //Default Value, Cal File Name : 'A'
                CFG_Flush();
                LC_OSL_LoadFromFile();
            }
            else if (touchIndex == MENU_SHORT)
            {
                LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);
                FONT_Write(FONT_FRANBIG, LCD_RED, BACK_COLOR, 4, 40, "Please wait. Read Short");
                LC_OSL_ScanShort();
                LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);
            }
            else if (touchIndex == MENU_LOAD)
            {
                LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);
                FONT_Write(FONT_FRANBIG, LCD_RED, BACK_COLOR, 4, 40, "Please wait. Read Load");
                LC_OSL_ScanLoad();
                LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);
            }
            else if (touchIndex == MENU_OPEN)
            {
                LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);
                FONT_Write(FONT_FRANBIG, LCD_RED, BACK_COLOR, 4, 40, "Please wait. Read Open");
                LC_OSL_ScanOpen();
                LCD_FillRect(LCD_MakePoint(4, 40),LCD_MakePoint(479, 88),BACK_COLOR);
            }
            else if (touchIndex == MENU_APPLY)
            {
                LC_OSL_Save();
            }
            else if (touchIndex == MENU_FREQ1)
            {
                CFG_SetParam(CFG_PARAM_LC_INDEX, 0);
                fChanged = 1;
            }
            else if (touchIndex == MENU_FREQ2)
            {
                CFG_SetParam(CFG_PARAM_LC_INDEX, 1);
                fChanged = 1;
            }
            else if (touchIndex == MENU_FREQ3)
            {
                CFG_SetParam(CFG_PARAM_LC_INDEX, 2);
                fChanged = 1;
            }
            else if (touchIndex == MENU_FREQ4)
            {
                CFG_SetParam(CFG_PARAM_LC_INDEX, 3);
                fChanged = 1;
            }
            else if (touchIndex == MENU_FREQ5)
            {
                CFG_SetParam(CFG_PARAM_LC_INDEX, 4);
                fChanged = 1;
            }
            else if (touchIndex == MENU_FREQ6)
            {
                CFG_SetParam(CFG_PARAM_LC_INDEX, 5);
                fChanged = 1;
            }

            //if Select Measured Index, Change to Frequency
            if (fChanged)
            {
                MeasureStartFreq = GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX));
                MeasureEndFreq = GetLCStepFreq(CFG_GetParam(CFG_PARAM_LC_INDEX) + 1);
                NowMesaureIndex = 0;
                NowMeasureFreq = MeasureStartFreq;


                fChanged = 0;
                //MeasurementFreq = GetLCMeasureFreq(0);
                CFG_Flush();
            }

            DisplayCalInfo();
            lcMenuDraw();
        }



#define LC_OSL_SCAN_STEP    (100000  )      //100Khz
        //Realtime Measuring
        //int MesaureIndex = 0;
        GEN_SetMeasurementFreq(NowMeasureFreq);
        DSP_MeasureLC(NowMeasureFreq, 1, 1, 1);
        rx = DSP_MeasuredZ();

        //Init Screen at first boot
        if (firstDraw)
        {
            firstDraw = 0;
            DrawMeasureLC(rx, 0, NowMeasureFreq, 0, 0, 0);
        }

        //Calculate im
        float r= fabsf(crealf(rx));
        float im= cimagf(rx);
        float rp,xp,Cppf,Lpuh;
        if(r >0.05 ) rp=r+im*(im/r); else rp=10000.0;
        if(im*im>0.0025) xp=im+r*(r/im); else im=10000.0;

        if(r > 0.05)
            rp = r + im * (im / r);
        else
            rp=10000.0;

        if(im * im > 0.0025)
            xp = im + r * (r/im);
        else
            im=10000.0;

        MeasureIM[NowMesaureIndex] = im;
        MeasureFreq[NowMesaureIndex] = NowMeasureFreq;

        NowMesaureIndex++;
        NowMeasureFreq += LC_OSL_SCAN_STEP;

        float qualityValue = 0.0;
        float qualitySum = 0.0;

        if (NowMeasureFreq >= MeasureEndFreq )
        {
            //Find Frequency
            int isFinded = 0;
            int findedIndex = 0;    //default Value is 0
            int validCount = 0;
            int validStartIndex = -1;
            int validEndIndex = 0;

#define VALID_FIELD_MIN_NUMBER 10   //for noise
            if (LC_Mode == 0)
            {
                //Case L
                //Found valid value and Index
                for (i = NowMesaureIndex - 1; i >= 0; i--)
                {
                    im = MeasureIM[i];

                    //not valid or last check row
                    if((im <= 1.0) || (im > 5000.0f) || i == 0)
                    {
                        if (validCount > VALID_FIELD_MIN_NUMBER)    //Valid
                        {
                            //Get AVG Value
                            float qualityAVG = qualitySum / validCount;

                            for (int j = i + 1; j <= validStartIndex; j++)
                            {
                                qualityValue += powf((1e6 * fabsf(im) / (2.0 * 3.1415926 * MeasureFreq[j])) - qualityAVG, 2);
                            }

                            qualityValue = sqrtf(qualityValue / (validCount -1));
                            qualityValue = qualityValue / qualityAVG * 100;

                            findedIndex = validStartIndex - 2;  //2 :Buffer
                            isFinded = 1;

                            break;
                        }

                        validCount = 0;
                        qualitySum = 0;
                        validStartIndex = -1;
                    }
                    else
                    {
                        if (validCount == 0)
                        {
                            validStartIndex = i;
                        }

                        float Luh = 1e6 * fabsf(im) / (2.0 * 3.1415926 * MeasureFreq[i]);
                        validCount++;
                        qualitySum += Luh;

                    }

                }

            }
            else
            {
                //Case C
                //Found valid value and Index
                //Case L
                //Found valid value and Index

                for (i = 0; i < NowMesaureIndex; i++)
                {
                    im = MeasureIM[i];

                    //not valid or last check row
                    //if ((im < -1.0) && (im > -5000.0f))
                    if((im >= -1.0) || (im < -5000.0f) || i == NowMesaureIndex -1)
                    {
                        if (validCount > VALID_FIELD_MIN_NUMBER)    //Valid
                        {
                            //Get AVG Value
                            float qualityAVG = qualitySum / validCount;

                            for (int j = validStartIndex; j < i; j++)
                            {
                                qualityValue += powf((1e12 / (2.0 * 3.1415926 * MeasureFreq[j] * fabs(im))) - qualityAVG, 2);
                            }
                            qualityValue = sqrtf(qualityValue / (validCount -1));
                            qualityValue = qualityValue / qualityAVG * 100;

                            isFinded = 1;
                            findedIndex = validStartIndex + 2;  //2 :Buffer

                            break;
                        }

                        validCount = 0;
                        qualitySum = 0;
                        validStartIndex = -1;
                    }
                    else
                    {
                        if (validCount == 0)
                        {
                            validStartIndex = i;
                        }

                        //float Luh = 1e6 * fabsf(im) / (2.0 * 3.1415926 * MeasureFreq[i]);
                        float Cpf = 1e12 / (2.0 * 3.1415926 * MeasureFreq[i] * fabs(im));

                        validCount++;
                        qualitySum += Cpf;

                    }
                }   //end of for
            }

            //Init
            NowMesaureIndex = 0;
            NowMeasureFreq = MeasureStartFreq;;

            //Re Measure
            GEN_SetMeasurementFreq(MeasureFreq[findedIndex]);
            DSP_MeasureLC(MeasureFreq[findedIndex], 1, 1, 1);
            rx = DSP_MeasuredZ();

            //Draw
            DrawMeasureLC(rx, findedIndex, MeasureFreq[findedIndex], isFinded, qualityValue, 0);
        }

        //Sleep(50);
        Sleep(2);
    }   //end of for

    //Release Memory
    free(MeasureIM);
    free(MeasureFreq);

    //Sleep(100);
    GEN_SetMeasurementFreq(0);
    return;
}
