/*
  S21-Gain for EU1KY's AA

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
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <string.h>

#include "LCD.h"
#include "touch.h"
#include "font.h"
#include "config.h"
#include "ff.h"
#include "crash.h"
#include "dsp.h"
#include "gen.h"
#include "oslfile.h"
#include "stm32746g_discovery_lcd.h"
#include "screenshot.h"
#include "panvswr2.h"
#include "panfreq.h"
#include "smith.h"
#include "textbox.h"
#include "generator.h"
#include "FreqCounter.h"
#include "bitmaps/bitmaps.h"

#define X0 51
#define Y0 18
#define WWIDTH  400
#define WHEIGHT 190
#define WY(offset) ((WHEIGHT + Y0) - (offset))
#define WGRIDCOLOR LCD_COLOR_DARKGRAY
#define RED1 LCD_RGB(245,0,0)
#define RED2 LCD_RGB(235,0,0)


static const char *modstr = "CEC v." AAVERSION_CECV " ";

//To have the same range to reduce memory usage
extern const char* BSSTR[];
extern const char* BSSTR_HALF[];
extern const uint32_t BSVALUES[];

#define BSSTR_TRACK       BSSTR
#define BSSTR_TRACK_HALF  BSSTR_HALF
#define BSVALUES_TRACK    BSVALUES

static char autoMeasureSpeed = 0;
static uint32_t f1 = 14000000;      //Scan range start frequency, in Hz
static BANDSPAN span = BS400;
static float fcur;                  // frequency at cursor position in kHz
//static char buf[64];
static LCDPoint pt;
float *valuesmI;
float *valuesdBI;

//static float displayRate = 2.9;   //Display Zoom
//static float displayOffset = 190; //Display Top Offset
#define displayRate (float)2.9         //Display Zoom
#define displayOffset 190
//static char DisplayType = 0;      //values or valuesmI
static int isMeasured = 0;
static uint32_t cursorPos = WWIDTH / 2;

static uint32_t cursorChangeCount = 0;
static uint32_t autofast = 0;
extern uint8_t NotSleepMode;

static void Track_DrawRX();
static int trackbeep;

//=====================================================================
//Menu
//---------------------------------------------------------------------
#define S21_BOTTOM_MENU_TOP 243
#define trackMenu_Length 6
static const int trackMenus[trackMenu_Length][4] = {{0, S21_BOTTOM_MENU_TOP, 79, 271},    //HOME BUTTON
    {81, S21_BOTTOM_MENU_TOP, 164, 271}, //SCAN
    {165, S21_BOTTOM_MENU_TOP, 216, 271}, //ZOOM OUT
    {217, S21_BOTTOM_MENU_TOP, 268, 271}, //ZOOM IN
    {269, S21_BOTTOM_MENU_TOP, 352, 271}, //FREQ
    {353, S21_BOTTOM_MENU_TOP, 435, 271}, //CAPTURE
};

void TRACK_Beep(int duration)
{
    if (BeepOn1==0)
        return;

    if(trackbeep==0)
    {
        trackbeep=1;
        AUDIO1=1;
        UB_TIMER2_Init_FRQ(880);
        UB_TIMER2_Start();
        Sleep(100);
        AUDIO1=0;
       // UB_TIMER2_Stop();
    }

    if(duration==1)
        trackbeep=0;
}

static void DrawAutoText(void)
{
    static const char* atxt = "  Auto  ";
    if (0 == autofast)
        FONT_Print(FONT_FRAN, TextColor, BackGrColor, 437, trackMenus[5][BUTTON_TOP] + 5,  atxt);
    else
        FONT_Print(FONT_FRAN, TextColor, LCD_MakeRGB(0, 128, 0), 437, trackMenus[5][BUTTON_TOP] + 5,  atxt);
}


//imgbtn_home
static void TRACK_DrawFootText(void){
    LCD_DrawBitmap(LCD_MakePoint(trackMenus[0][BUTTON_LEFT], trackMenus[0][BUTTON_TOP]), imgbtn_home1, imgbtn_home1_size);
    LCD_DrawBitmap(LCD_MakePoint(trackMenus[1][BUTTON_LEFT], trackMenus[1][BUTTON_TOP]), imgbtn_scan, imgbtn_scan_size);
    LCD_DrawBitmap(LCD_MakePoint(trackMenus[2][BUTTON_LEFT], trackMenus[2][BUTTON_TOP]), imgbtn_zoomout, imgbtn_zoomout_size);
    LCD_DrawBitmap(LCD_MakePoint(trackMenus[3][BUTTON_LEFT], trackMenus[3][BUTTON_TOP]), imgbtn_zoomin, imgbtn_zoomin_size);
    LCD_DrawBitmap(LCD_MakePoint(trackMenus[4][BUTTON_LEFT], trackMenus[4][BUTTON_TOP]), imgbtn_freq, imgbtn_freq_size);
    LCD_DrawBitmap(LCD_MakePoint(trackMenus[5][BUTTON_LEFT], trackMenus[5][BUTTON_TOP]), imgbtn_capture, imgbtn_capture_size);

    DrawAutoText();
}


static void WK_InvertPixel(LCDPoint p){
LCDColor    c;
    c=LCD_ReadPixel(p);
    switch (c){
    case LCD_COLOR_YELLOW:
        {
            LCD_SetPixel(p,LCD_COLOR_RED);
            return;
        }
    case LCD_COLOR_WHITE:
        {
            LCD_SetPixel(p,RED1);
            return;
        }
    case LCD_COLOR_DARKGRAY:
        {
            LCD_SetPixel(p,RED2);
            return;
        }
    case LCD_COLOR_RED:
        {
            LCD_SetPixel(p,LCD_COLOR_YELLOW);
            return;
        }
    case RED1:
        {
            LCD_SetPixel(p,LCD_COLOR_WHITE);
            return;
        }
    case RED2:
        {
            LCD_SetPixel(p,LCD_COLOR_DARKGRAY);
            return;
        }
    default:LCD_InvertPixel(p);
    }
}

static void DrawCursor()
{
    int8_t i;
    LCDPoint p;
    if (!isMeasured)
        return;

    //Draw cursor line as inverted image
    p = LCD_MakePoint(X0 + cursorPos, Y0);

    if(ColourSelection==1){// Daylightcolours
        while (p.y < Y0 + WHEIGHT){
           if((p.y % 20)<10)
                WK_InvertPixel(p);
           else LCD_InvertPixel(p);
           p.y++;

        }
    }
    else{
        while (p.y < Y0 + WHEIGHT){
            if((p.y % 20)<10)
                LCD_InvertPixel(p);
            p.y++;

        }
    }

    if(FatLines){
        p.x--;
        while (p.y >= Y0)
        {
            LCD_InvertPixel(p);
            p.y--;
        }
        p.x+=2;
        while (p.y < Y0 + WHEIGHT)
        {
            LCD_InvertPixel(p);
            p.y++;
        }
        p.x--;
    }

    LCD_FillRect((LCDPoint){X0 + cursorPos-3,Y0+WHEIGHT+1},(LCDPoint){X0 + cursorPos+3,Y0+WHEIGHT+3},BackGrColor);
    LCD_FillRect((LCDPoint){X0 + cursorPos-2,Y0+WHEIGHT+1},(LCDPoint){X0 + cursorPos+2,Y0+WHEIGHT+3},TextColor);

    Sleep(5);
}

static void MoveCursor(int moveDirection)
{
    if (!isMeasured)
        return;

    if (moveDirection > 0)
    {
        if (cursorPos >= WWIDTH)
            return;
    }
    else
    {
        if (cursorPos <= 0)
            return;
    }

    DrawCursor();
    cursorPos += moveDirection;
    DrawCursor();
    //DrawCursorText();


    float rx = valuesmI[cursorPos];
    float dispDB = valuesdBI[cursorPos];

    uint32_t fstart;
    if (CFG_GetParam(CFG_PARAM_PAN_CENTER_F) == 0)
        fstart = f1;
    else
        fstart = f1 - 500*BSVALUES_TRACK[span];

    fcur = ((float)(fstart/1000. + (float)cursorPos * BSVALUES_TRACK[span] / WWIDTH));///1000.;
    if (fcur * 1000.f > (float)(CFG_GetParam(CFG_PARAM_BAND_FMAX) + 1))
        fcur = 0.f;

    LCD_FillRect(LCD_MakePoint(40, 227),LCD_MakePoint(479 , 239),BackGrColor);
    FONT_Print(FONT_FRAN, TextColor, BackGrColor, 60, Y0 + WHEIGHT + 16, "F:%10.5fMhz    dB:%8.3f    Measured: %5.3fmV",
               fcur / 1000,
               dispDB,
               rx);
    LCD_HLine(LCD_MakePoint(40,226), 400, CurvColor);
    LCD_HLine(LCD_MakePoint(40,239), 400, CurvColor);

    if (cursorChangeCount++ < 10)
        Sleep(100); //Slow down at first steps
    Sleep(5);
}

static void Track_DrawGrid(int justGraphDraw)  //
{
    char buf[30];
    int i;
    int verticalPos = 12;
    uint32_t fstart;
    uint32_t pos = 130;

    #define VerticalStep 29.3
    #define VerticalX 10

    if (0 == CFG_GetParam(CFG_PARAM_PAN_CENTER_F))
    {
        fstart = f1;
        sprintf(buf, " graph: %.3f MHz +%s", (float)f1/1000000, BSSTR_TRACK[span]);
    }

    else {
        fstart = f1 - 500*BSVALUES_TRACK[span];
        sprintf(buf, " graph: %.3f MHz +/- %s", (float)f1/1000000, BSSTR_TRACK_HALF[span]);
    }


    if (justGraphDraw)
    {
        LCD_FillRect(LCD_MakePoint(40, 15), LCD_MakePoint(459 , 225), BackGrColor);
    }
    else
    {
        LCD_FillAll(BackGrColor);

        //Vertical Numbers
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, " 0dB");
        verticalPos += VerticalStep;
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-10");
        verticalPos += VerticalStep;
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-20");
        verticalPos += VerticalStep;
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-30");
        verticalPos += VerticalStep;
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-40");
        verticalPos += VerticalStep;
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-50");
        verticalPos += VerticalStep;
        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-60");
        //verticalPos += VerticalStep;
        //FONT_Write(FONT_FRAN, CurvColor, BackGrColor, VerticalX, verticalPos, "-70");

        FONT_Write(FONT_FRAN, LCD_BLACK, LCD_PURPLE, 50, 0, modstr);
        FONT_Write(FONT_FRANBIG, TextColor, 0, 2, 107, "<");
        FONT_Write(FONT_FRANBIG, TextColor, 0, 460, 107, ">");

        FONT_Write(FONT_FRAN, CurvColor, BackGrColor, 200, 0, "VNA |S21|Gain");
        FONT_Write(FONT_FRAN, TextColor, BackGrColor, 320, 0, buf);//LCD_BLUE
    }

    //Draw F grid and labels
    int lmod = 5;
    int linediv = 10; //Draw vertical line every linediv pixels

    for (i = 0; i <= WWIDTH/linediv; i++)
    {
        int x = X0 + i * linediv;
        if ((i % lmod) == 0 || i == WWIDTH/linediv)
        {
            //char fr[10];
            float flabel = ((float)(fstart/1000. + i * BSVALUES_TRACK[span] / (WWIDTH/linediv)))/1000.f;
            if (flabel * 1000000.f > (float)(CFG_GetParam(CFG_PARAM_BAND_FMAX)+1))
                continue;
            if(flabel>999.99)
                sprintf(buf, "%.1f", ((float)(fstart/1000. + i * BSVALUES_TRACK[span] / (WWIDTH/linediv)))/1000.f);
            else if(flabel>99.99)
                sprintf(buf, "%.2f", ((float)(fstart/1000. + i * BSVALUES_TRACK[span] / (WWIDTH/linediv)))/1000.f);
            else
                sprintf(buf, "%.3f", ((float)(fstart/1000. + i * BSVALUES_TRACK[span] / (WWIDTH/linediv)))/1000.f);// WK
            int w = FONT_GetStrPixelWidth(FONT_SDIGITS, buf);
           // FONT_Write(FONT_SDIGITS, LCD_WHITE, LCD_BLACK, x - w / 2, Y0 + WHEIGHT + 5, f);// WK
            FONT_Write(FONT_FRAN, TextColor, BackGrColor, x -8 - w / 2, Y0 + WHEIGHT + 2, buf);
            LCD_VLine(LCD_MakePoint(x, Y0), WHEIGHT, WGRIDCOLOR);
            LCD_VLine(LCD_MakePoint(x+1, Y0), WHEIGHT, WGRIDCOLOR);// WK
        }
        else
        {
            LCD_VLine(LCD_MakePoint(x, Y0), WHEIGHT, WGRIDCOLOR);
        }
    }

    //LCD_FillRect((LCDPoint){X0 ,Y0+WHEIGHT+1},(LCDPoint){X0 + WWIDTH+2,Y0+WHEIGHT+3},BackGrColor);
}

static uint32_t Fs, Fp;// in Hz
static float Cp, Rs;

//extern void DSP_MeasureTrack(uint32_t freqHz, int applyErrCorr, int applyOSL, int nMeasurements);
//extern float DSP_MeasuredTrackValue();

static void Scan21(int selector)
{
    float inputMI;
    uint32_t i, nScanCount;
    uint32_t fstart, freq1, deltaF;

    f1=CFG_GetParam(CFG_PARAM_S21_F1);
    if (CFG_GetParam(CFG_PARAM_PAN_CENTER_F) == 0)
        fstart = f1;
    else
        fstart = f1 - 500*BSVALUES_TRACK[span]; // 2;

    freq1 = fstart - 150000;
    if (freq1 < 100000)
        freq1 = 100000;

    DSP_MeasureTrack(freq1, 1, 1, 1); //Fake initial run to let the circuit stabilize
    inputMI = DSP_MeasuredTrackValue();
    Sleep(20);

    DSP_MeasureTrack(freq1, 1, 1, 1);
    inputMI = DSP_MeasuredTrackValue();

    float xp=1.0f;

    Cp = -1 / ( 6.2832 * freq1 * xp);
    if (selector == 1)
        return;

    deltaF=(BSVALUES_TRACK[span] * 1000) / WWIDTH;
    nScanCount = CFG_GetParam(CFG_PARAM_PAN_NSCANS);

    for(i = 0; i <= WWIDTH; i++)
    {
        freq1 = fstart + i * deltaF;
        if (freq1 == 0) //To overcome special case in DSP_Measure, where 0 is valid value
            freq1 = 1;
        DSP_MeasureTrack(freq1, 1, 1, nScanCount);
        inputMI = DSP_MeasuredTrackValue();

        valuesmI[i] = inputMI;
        valuesdBI[i] = freq1;   //OSL_TXTodB(freq1, inputMI);   //

        LCD_SetPixel(LCD_MakePoint(X0 + i, 135), LCD_BLUE);// progress line
        LCD_SetPixel(LCD_MakePoint(X0 + i, 136), LCD_BLUE);
    }

    for(i = 0; i <= WWIDTH; i++)
    {
        valuesdBI[i] = OSL_TXTodB(valuesdBI[i], valuesmI[i]);
    }

    FONT_Write(FONT_FRAN, LCD_RED, LCD_BLACK, 420, 0, "     ");
    GEN_SetMeasurementFreq(0);
    isMeasured = 1;
}
//=========================================================================
//                        NOT REMOVE BELOW LINES
//=========================================================================
////For Version 0.5
////Check memory interference and open
//int offsetIndex = 0;
//static void Scan21Fast_v05()
//{
//    float inputMI;
//    uint32_t i, nScanCount;
//    uint32_t fstart, freq1, deltaF;
//
//    f1=CFG_GetParam(CFG_PARAM_S21_F1);
//    if (CFG_GetParam(CFG_PARAM_PAN_CENTER_F) == 0)
//        fstart = f1;
//    else
//        fstart = f1 - 500*BSVALUES_TRACK[span]; // 2;
//
//    freq1 = fstart - 150000;
//    if (freq1 < 100000)
//        freq1 = 100000;
//
//    DSP_MeasureTrack(freq1, 1, 1, 1); //Fake initial run to let the circuit stabilize
//    inputMI = DSP_MeasuredTrackValue();
//    //Sleep(20);
//
//    DSP_MeasureTrack(freq1, 1, 1, 1);
//    inputMI = DSP_MeasuredTrackValue();
//
//    deltaF=(BSVALUES_TRACK[span] * 1000) / WWIDTH;
//    nScanCount = CFG_GetParam(CFG_PARAM_PAN_NSCANS);
//
///*
//#define MEASURE_INTERVAL_FAST 32
//#define USEHIST_INTERVAL_FAST 16
//#define MEASURE_MAX_INDEX 1
//*/
//#define MEASURE_INTERVAL_FAST 32
//#define USEHIST_INTERVAL_FAST 8
//#define MEASURE_MAX_INDEX 3
//
//    const int FAST_SCAN_OFFSET[] = {0, 16, 8, 24};
//
//    int fastOffset = FAST_SCAN_OFFSET[offsetIndex++];
//
//    if (offsetIndex > MEASURE_MAX_INDEX)
//        offsetIndex = 0;
//
//    for(i = 0; i <= WWIDTH; i++)
//    {
//        freq1 = fstart + (i + fastOffset) * deltaF;
//        if (freq1 == 0) //To overcome special case in DSP_Measure, where 0 is valid value
//            freq1 = 1;
//
//        int drawX = i + fastOffset;
//
//
//        if (i % MEASURE_INTERVAL_FAST != 0)
//        {
//            continue;
//        }
//#ifdef _DEBUG_UART
////    DBG_Printf("PROCESS: OFST:%d, IDX:%d, drawX:%d, Freq:%u", fastOffset, i, drawX, freq1);
//#endif
//
//        //DSP_MeasureTrack(freq1, 1, 1, nScanCount);
//        DSP_MeasureTrack(freq1, 1, 1, 1);
//        valuesdBI[drawX] = OSL_TXTodB(freq1, DSP_MeasuredTrackValue());
//
//            /*
//        //Break by Touch
//        if (autofast)
//        {
//            if (TOUCH_Poll(&pt))
//            {
//                if (pt.y > 230 && pt.x > 442)
//                {
//                    autofast = 0;
//                    TRACK_Beep(0);
//                    TRACK_DrawFootText();
//                    return;
//                }
//            }
//        }
//            */
//    }
//
//    //FONT_Write(FONT_FRAN, LCD_RED, LCD_BLACK, 420, 0, "     ");
//    GEN_SetMeasurementFreq(0);
//    isMeasured = 1;
//
///*
//    //fastOffset
//    //First Interpolate
//    for(i = fastOffset; i < WWIDTH - MEASURE_INTERVAL_FAST; i += USEHIST_INTERVAL_FAST)
//    {
//        uint32_t fr = i % MEASURE_INTERVAL_FAST;
//
//        if (fastOffset == fr)
//            continue;
//
//        int fi0, fi2;
//
//        //17 : 16, 48
//
//        fi0 = i / MEASURE_INTERVAL_FAST;
//        fi2 = fi0 + 1;
//
//        fi0 *= MEASURE_INTERVAL_FAST;
//        fi2 *= MEASURE_INTERVAL_FAST;
//
//        fi0 += fastOffset;
//        fi2 += fastOffset;
//
//        if (fi2 > WWIDTH)
//            continue;
//
//#ifdef _DEBUG_UART
//    //DBG_Printf("PROCESS: OFST:%d, IDX:%d, f0:%d, f2:%u", fastOffset, i, fi0, fi2);
//#endif
//
//        float Yi = valuesdBI[fi0];
//        float Xi = fi0;
//        float Yi1 = valuesdBI[fi2];
//        float Xi1 = fi2;
//        float Xb = i;
//
//        float Yb = Yi + (Yi1 - Yi) * (Xb - Xi) / (Xi1 - Xi);
//        valuesdBI[i] = Yb * 0.3 + valuesdBI[i] * 0.7;
//    }
//*/
//
//
//    //Interpolate intermediate values
//    for(i = 1; i <= WWIDTH; i++)
//    {
//        uint32_t fr = i % USEHIST_INTERVAL_FAST;
//
//        if (0 == fr)
//            continue;
//
//        int fi0, fi1, fi2;
//
//        fi0 = i / USEHIST_INTERVAL_FAST;
//        fi2 = fi0 + 1;
//
//        fi0 *= USEHIST_INTERVAL_FAST;
//        fi2 *= USEHIST_INTERVAL_FAST;
//
//
//        float Yi = valuesdBI[fi0];
//        float Xi = fi0;
//        float Yi1 = valuesdBI[fi2];
//        float Xi1 = fi2;
//        float Xb = i;
//
//        float Yb = Yi + (Yi1 - Yi) * (Xb - Xi) / (Xi1 - Xi);
//        valuesdBI[i] = Yb;
//    }
//
///*
//    //Full Interpolate by Latest Measurement data
//    for(i = fastOffset; i <= WWIDTH; i++)
//    {
//        uint32_t fr = i % MEASURE_INTERVAL_FAST;
//
//        if (fastOffset == fr)
//            continue;
//
//        int fi0, fi2;
//        fi0 = i / MEASURE_INTERVAL_FAST;
//        fi2 = fi0 + 1;
//
//        fi0 *= MEASURE_INTERVAL_FAST;
//        fi2 *= MEASURE_INTERVAL_FAST;
//
//        fi0 += fastOffset;
//        fi2 += fastOffset;
//
//
//        //if (fi2 > WWIDTH)
//        //    continue;
//
//#ifdef _DEBUG_UART
//    //DBG_Printf("PROCESS: OFST:%d, IDX:%d, f0:%d, f2:%u", fastOffset, i, fi0, fi2);
//#endif
//
//        float Yi = valuesdBI[fi0];
//        float Xi = fi0;
//        float Yi1 = valuesdBI[fi2];
//        float Xi1 = fi2;
//        float Xb = i;
//
//        float Yb = Yi + (Yi1 - Yi) * (Xb - Xi) / (Xi1 - Xi);
//        //valuesdBI[i] = Yb * 0.3 + valuesdBI[i] * 0.7;
//
//        if (fabs(valuesdBI[i] - Yb) > 20)
//        {
//            valuesdBI[i] = Yb ;
//        }
//        else
//        {
//            valuesdBI[i] = Yb * 0.3 + valuesdBI[i] * 0.7;
//        }
//        //valuesdBI[i] = Yb;
//    }
//*/
//
//}


//Version 0.35
static void Scan21Fast()
{
//#define FAST_DIVIDE_FREQ 12
#define FAST_DIVIDE_FREQ autoMeasureSpeed

    //float inputMI;
    uint32_t i, nScanCount;
    uint32_t fstart, freq1, deltaF;

    f1=CFG_GetParam(CFG_PARAM_S21_F1);
    if (CFG_GetParam(CFG_PARAM_PAN_CENTER_F) == 0)
        fstart = f1;
    else
        fstart = f1 - 500*BSVALUES_TRACK[span]; // 2;

    freq1 = fstart - 150000;
    if (freq1 < 100000)
        freq1 = 100000;

    DSP_MeasureTrack(freq1, 1, 1, 1); //Fake initial run to let the circuit stabilize
    //inputMI = DSP_MeasuredTrackValue();
    //Sleep(20);

    DSP_MeasureTrack(freq1, 1, 1, 1);
    //inputMI = DSP_MeasuredTrackValue();

    deltaF=(BSVALUES_TRACK[span] * 1000) / WWIDTH;
    nScanCount = CFG_GetParam(CFG_PARAM_PAN_NSCANS);


    for(i = 0; i <= WWIDTH; i++)
    {
        freq1 = fstart + i * deltaF;
        //if (freq1 == 0) //To overcome special case in DSP_Measure, where 0 is valid value
        //    freq1 = 1;

        //valuesdBI[i] = freq1;   //OSL_TXTodB(freq1, inputMI);   //

        if (i % FAST_DIVIDE_FREQ != 0)
        {
            continue;
        }

        DSP_MeasureTrack(freq1, 1, 1, nScanCount);
        //inputMI = DSP_MeasuredTrackValue();

        //OSL_CorrectTX(freq1, &inputMI);
        valuesmI[i] = DSP_MeasuredTrackValue();
        valuesdBI[i] = OSL_TXTodB(freq1, valuesmI[i]);


        //Break by Touch
        if (autofast)
        {
            if (TOUCH_Poll(&pt))
            {
                if (pt.y > 230 && pt.x > 450)
                {
                    autofast = 0;
                    TRACK_Beep(0);
                    TRACK_DrawFootText();
                    //break;
                }
            }
        }
    }

    //FONT_Write(FONT_FRAN, LCD_RED, LCD_BLACK, 420, 0, "     ");
    GEN_SetMeasurementFreq(0);
    isMeasured = 1;

    //Interpolate intermediate values
    for(i = 1; i <= WWIDTH; i++)
    {
        //uint32_t fr = i % FAST_DIVIDE_FREQ;

        if (0 == i % FAST_DIVIDE_FREQ)
            continue;

        int fi0, fi1, fi2;

        fi0 = i / FAST_DIVIDE_FREQ;
        fi2 = fi0 + 1;

        fi0 *= FAST_DIVIDE_FREQ;
        fi2 *= FAST_DIVIDE_FREQ;
/*

        float Yi = valuesdBI[fi0];
        float Xi = fi0;
        float Yi1 = valuesdBI[fi2];
        float Xi1 = fi2;
        float Xb = i;

        float Yb = Yi + (Yi1 - Yi) * (Xb - Xi) / (Xi1 - Xi);
        valuesdBI[i] = Yb;
*/

        valuesdBI[i] = valuesdBI[fi0] + (valuesdBI[fi2] - valuesdBI[fi0]) * (i - fi0) / (fi2 - fi0);
        valuesmI[i] = valuesmI[fi0] + (valuesmI[fi2] - valuesmI[fi0]) * (i - fi0) / (fi2 - fi0);
    }
}

static void Track_DrawRX(int SelQu, int SelEqu)// SelQu=1, if quartz measurement  SelEqu=1, if equal scales
{
#define LimitR 1999.f
    int i, imax;
    int x, RXX0;

    if (!isMeasured)
        return;

    RXX0 = X0;

    //Now draw R graph
    int lastoffset = 0;

    //Draw  labels
    int yofs = 0;
    int yofs_sm = 0;

    //Now draw X graph
    lastoffset = 0;
    for(i = 0; i <= WWIDTH; i++)
    {
        x = RXX0 + i;

        yofs = valuesdBI[i] * displayRate + displayOffset;

        if (yofs > 193)
        {
            yofs = 193;
        }
        else if (yofs < 0)
        {
            yofs = 0;
        }

        if(i == 0)
        {
            LCD_SetPixel(LCD_MakePoint(x, WY(yofs)), LCD_WHITE);//LCD_RGB(SM_INTENSITY, 0, 0
        }
        else
        {
            LCD_Line(LCD_MakePoint(x - 1, WY(lastoffset)), LCD_MakePoint(x, WY(yofs)), LCD_WHITE);
            if(FatLines){
                LCD_Line(LCD_MakePoint(x - 1, WY(lastoffset)+1), LCD_MakePoint(x, WY(yofs)+1), LCD_WHITE);// WK
            }
        }

        lastoffset = yofs;
    }
}

int nowLayerIndex = 0;
static void RedrawWindow(int justGraphDraw)
{
    Track_DrawGrid(justGraphDraw);
    Track_DrawRX(0, 0);// 1
    DrawCursor();


    if (! justGraphDraw)
    {
        TRACK_DrawFootText();
    }

    DrawAutoText();
}


 #define XX0 190
 #define YY0 42

int TRACK_TouchTest(){

     if (TOUCH_Poll(&pt)){
        if((pt.y <80)&&(pt.x >380)){
            // Upper right corner --> EXIT
            TRACK_Beep(1);
            while(TOUCH_IsPressed());
            Sleep(100);
            return 99;
        }
        if(pt.x<(XX0-8)){// select the pressed field:
            TRACK_Beep(1);
            if(pt.y<YY0+48) return 0;
            if(pt.y<YY0+96) return 1;
            if(pt.y<YY0+144) return 2;
            if(pt.y<YY0+192) return 3;
            return 4;
        }
     }
     return -1;
 }

static void save_snapshot(void)
{
    static const TCHAR *sndir = "/aa/snapshot";
    char path[64];
    char wbuf[256];
    char* fname = 0;
    uint32_t i = 0;
    FRESULT fr = FR_OK;

    if (!isMeasured)
        return;

    Date_Time_Stamp();

    fname = SCREENSHOT_SelectFileName();

    if(strlen(fname)==0) return;

    SCREENSHOT_DeleteOldest();
    if (CFG_GetParam(CFG_PARAM_SCREENSHOT_FORMAT))
        SCREENSHOT_SavePNG(fname);
    else
        SCREENSHOT_Save(fname);

    //DrawSavedText();
    //static const char* txt = "  Snapshot saved  ";
    FONT_Write(FONT_FRAN, LCD_WHITE, LCD_RGB(0, 60, 0), 165,  Y0 + WHEIGHT + 16 + 16, "  Snapshot saved  ");
    TRACK_DrawFootText();
    DrawAutoText();

    return;
CRASH_WR:
    CRASHF("Failed to write to file %s", path);
}

//=====================================================================
//TRACK PROC
//---------------------------------------------------------------------
void Track_Proc(void)
{
    int redrawRequired = 0;
    uint32_t activeLayer;
    uint32_t FreqkHz;

    //allocated memory
    valuesmI = (float *)malloc(sizeof(float) * (WWIDTH + 20));
    valuesdBI = (float *)malloc(sizeof(float) * (WWIDTH + 20));

    SetColours();
    LCD_FillAll(BackGrColor);
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 170, 100, "VNA |S21|Gain");
    Sleep(1000);
    while(TOUCH_IsPressed());

    //Load Calibration
    if (! OSL_IsTXCorrLoaded())
    {
        OSL_LoadTXCorr();
#ifdef _DEBUG_UART
    DBG_Str("Load VNA Calibration File \r\n");
#endif
    }


    //LoadBkups();
    //Load saved frequency and span values from config file
    uint32_t fbkup = CFG_GetParam(CFG_PARAM_S21_F1);

    //2019.03.26
    //if (fbkup != 0 && fbkup >= BAND_FMIN && fbkup <= CFG_GetParam(CFG_PARAM_BAND_FMAX) && (fbkup % 100) == 0)
    if (fbkup != 0 && fbkup >= BAND_FMIN && fbkup <= 100000000 && (fbkup % 100) == 0)
    {
        f1 = fbkup;
    }
    else
    {
        f1 = 14000000;
        CFG_SetParam(CFG_PARAM_S21_F1, f1);
        CFG_SetParam(CFG_PARAM_S21_SPAN, BS400);
        CFG_Flush();
    }

    int spbkup = CFG_GetParam(CFG_PARAM_S21_SPAN);
    span = (BANDSPAN)spbkup;
    autoMeasureSpeed = CFG_GetParam(CFG_PARAM_S21_AUTOSPEED);


    FreqkHz = f1 / 1000;

    Track_DrawGrid(0);
    TRACK_DrawFootText();
    DrawAutoText();

    //FONT_Write(FONT_FRAN, LCD_PURPLE, LCD_BLACK, 160,  20, "(Tap here to set F and Span)");

    for(;;)
    {
        //

        //Auto Scan Call
        if (autofast)
        {
            Scan21Fast();
            RedrawWindow(1);
            redrawRequired = 0;
        }
        else
        {
            Sleep(5);
        }

        if (TOUCH_Poll(&pt))
        {
            int touchIndex = GetTouchIndex(pt, trackMenus, trackMenu_Length);

            //Play Beep
            if (touchIndex != -1)
            {
                TRACK_Beep(1);
                while(TOUCH_IsPressed());
            }


            if (touchIndex == 0)    //EXIT
            {
                autofast = 0;
                break;// Exit
            }
            else if (touchIndex == 1)    //SCAN
            {
                if (autofast)
                {
                    autofast = 0;
                }

                FONT_Write(FONT_FRANBIG, LCD_RED, LCD_BLACK, 180, 100, "  Scanning...  ");
                Scan21(0);
                redrawRequired = 1;
            }
            else if (touchIndex == 2)    //ZOOM OUT
            {
                    //zoomMinus();
                    if(fcur!=0)
                        f1=fcur*1000.;
                    if(span>0)
                        span--;
                    if(0 == autofast)
                        Scan21(0);
                    redrawRequired = 1;
            }
            else if (touchIndex == 3)    //ZOOM IN
            {
                    //zoomPlus();
                    if(fcur!=0)
                        f1=fcur*1000.;
                    if(span<BS100M)
                        span++;
                    if(0 == autofast)
                        Scan21(0);
                    redrawRequired = 1;
            }
            else if (touchIndex == 4 || (pt.y < 80)&&(pt.x>60))    //Input Freq
            {

                uint32_t fxkHzs;//Scan range start frequency, in kHz
                uint32_t fxs = CFG_GetParam(CFG_PARAM_S21_F1);
                fxkHzs = fxs/1000;
                //span=BS400;
                if (PanFreqWindow(&fxkHzs, (BANDSPAN*)&span))
                {
                    f1 = fxkHzs * 1000;

                    CFG_SetParam(CFG_PARAM_S21_F1, f1);

                    isMeasured = 0;

                    //BSP_LCD_SelectLayer(activeLayer);
                    RedrawWindow(0);

                    CFG_SetParam(CFG_PARAM_S21_SPAN, span);
                    CFG_Flush();
                }


            }
            else if (touchIndex == 5)    //Capture
            {
                save_snapshot();

                //Redraw
                RedrawWindow(0);
                TRACK_DrawFootText();
                DrawAutoText();
            }
            else if (pt.y > 90 && pt.y <= 170)
            {
                //< Button
                if (pt.x < 50)
                {
                    TRACK_Beep(0);
                    MoveCursor(-1);
                    continue;
                }
                else if (pt.x > 430)
                {
                    // > Button
                    TRACK_Beep(0);
                    MoveCursor(1);
                    continue;
                }
            }
            else if (pt.y > 250)
            {
                if (pt.x > 435)
                {
                    if (autofast == 0)
                    {
                        autofast = 1;
                    }
                    else
                    {
                        autofast = 0;
                    }

                    TRACK_Beep(0);
                    redrawRequired = 1;
                    TRACK_DrawFootText();
                }

            }   //end of if

            if(redrawRequired)
            {
                //BSP_LCD_SelectLayer(!activeLayer);
                //RedrawWindow(0);
                //BSP_LCD_SelectLayer(activeLayer);
                RedrawWindow(0);
                redrawRequired = 0;

                if (autofast)
                {
                    char tmpBuff[16];
                    sprintf(tmpBuff, "AUTO SPEED : [%d] ", autoMeasureSpeed);
                    FONT_Write(FONT_FRAN, LCD_BLACK, LCD_RED, 50, 0, tmpBuff);
                }
            }

            while(TOUCH_IsPressed())
            {
                Sleep(250);
            }

            //LCD_ShowActiveLayerOnly();

        }
        else
        {
            cursorChangeCount = 0;
            trackbeep=0;
        }   //end of touch check

        //NotSleepMode = autofast;
        //LCD_ShowActiveLayerOnly();
    }   //end of while

    NotSleepMode = 0;
    //Release Memory
    free(valuesmI);
    free(valuesdBI);
    isMeasured = 0;
}
