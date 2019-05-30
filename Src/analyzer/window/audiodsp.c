/*
  KD8CEC's AUDIO DSP for HAM
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
#include "bitmaps/bitmaps.h"
#include <stdint.h>
#include "arm_math.h"

#include "audioirq.h"

#define BACK_COLOR LCD_RGB(10, 52, 74)
#define SELECT_FREQ_COLOR LCD_RGB(255, 127, 39)
#define SELECT_PROTOCOL_COLOR LCD_RGB(255, 30, 30)

#define GRAPH_TOP               0
#define GRAPH_BOTTOM            70
#define METER_HEIGHT            15
#define GRAPH_WATERFALL_TOP     GRAPH_BOTTOM + METER_HEIGHT
#define GRAPH_WATERFALL_HEIGHT  100
#define GRAPH_WATERFALL_BOTTOM  GRAPH_WATERFALL_TOP + GRAPH_WATERFALL_HEIGHT

//Default Filters
#include "flt_lpf.h"
#include "flt_hpf.h"
#include "flt_bpf_50.h"
#include "flt_bpf_100.h"
#include "flt_bpf_150.h"
#include "flt_stop_50.h"

#include "flt_windows.h"

extern void Sleep(uint32_t ms);
extern void TRACK_Beep(int duration);

//=============================================================================
//VARIABLE DEFINE FOR FFT, FIR, IIR
//By KD8CEC
//-----------------------------------------------------------------------------
#define AUDIO_BUFF_SIZE ((uint32_t)512)                 //HALF 256, COMPLETE 256
#define FFT_BIN_FREQ (float)7.8125                      //FFT SPECTRUM (FFT BIN)
//#define AUDIO_BUFFER_RAM AUDIO_BUFFER_OUT

//Share SD-RAM for Audio IN / OUT
//Received Buffer (MEMS MIC on board or MIC JACK)
static int16_t __attribute__((section (".user_sdram"))) AUDIO_BUFFER_IN_FLT[AUDIO_BUFF_SIZE] = { 0 };
//int16_t *AUDIO_BUFFER_IN_FLT = &AUDIO_BUFFER_RAM[0];

//Output Buffer (Phone Jack)
static int16_t __attribute__((section (".user_sdram"))) AUDIO_BUFFER_OUT_FLT[AUDIO_BUFF_SIZE] = { 0 };
//int16_t *AUDIO_BUFFER_OUT_FLT = &AUDIO_BUFFER_RAM[AUDIO_BUFF_SIZE];

//for iir
static float __attribute__((section (".user_sdram"))) float_buffer_in[AUDIO_BUFF_SIZE * 2] = { 0 };
static float __attribute__((section (".user_sdram"))) float_buffer_out[AUDIO_BUFF_SIZE * 2] = { 0 };
//float *float_buffer_in  = (float *)&AUDIO_BUFFER_RAM[AUDIO_BUFF_SIZE * 2];
//float *float_buffer_out = (float *)&AUDIO_BUFFER_RAM[AUDIO_BUFF_SIZE * 6];


#define iirStageCount 6
#define iirTapsCount  5 * iirStageCount	//2 order : 5, B0, b1, B2, A1, B2
//float iirStateBuff[2 * iirStageCount];
//float iirStateBuff_HPF[2 * iirStageCount];
static float __attribute__((section (".user_sdram"))) iirStateBuff[2 * iirStageCount];
static float __attribute__((section (".user_sdram"))) iirStateBuff_HPF[2 * iirStageCount];


#define iirLength		AUDIO_BUFF_SIZE / 2     //


//BASIC FILTER
//float iirCoeffBuff[30] = {
static float __attribute__((section (".user_sdram"))) iirCoeffBuff[30];
//float iirCoeffBuff_HPF[30] = {
static float __attribute__((section (".user_sdram"))) iirCoeffBuff_HPF[30];


//Spectrum Buffer
#define FFT_SPECTRUM_STEP 16
#define FFT_AUDIO_SIZE    128   //512 => 256 => 128 (LEFT CHANNEL )
#define FFT_SPECTRUM_SIZE FFT_AUDIO_SIZE * FFT_SPECTRUM_STEP
static float __attribute__((section (".user_sdram"))) SpectrumBuff[FFT_SPECTRUM_SIZE * 2] = { 0 };
static float __attribute__((section (".user_sdram"))) testOutput[FFT_SPECTRUM_SIZE] = { 0 };
//static float __attribute__((section (".user_sdram"))) windowData[FFT_AUDIO_SIZE] = { 0 };

//EXIT, SAVECONFIG, BPF, LPF, HPF, SSB
void SetCenterFrequency(arm_biquad_cascade_df2T_instance_f32 instMain, arm_biquad_cascade_df2T_instance_f32 instSecond, uint32_t newCenterFreq, uint32_t isChangedFilter);

#define FILTER_MINIMUM_FREQ 300
#define FILTER_MAXIMUM_FREQ 3000


//Draw ImageButton
//BOTTOM
#define MENU_EXIT         0
#define MENU_SAVECONFIG   1
#define MENU_BPF          2
#define MENU_USERFILTER   3      //Probe Select Down
#define MENU_BYPASS       4      //Probe Select Up

//RIGHT
#define MENU_AFFREQDOWN   5     //-Hz Center Freq
#define MENU_AFFREQUP     6     //+Hz Center Freq

//FREQ SELECT MENU
#define MENU_TOP1         9
#define MENU_TOP2        10
#define MENU_TOP3        11
#define MENU_TOP4        12

#define dspMenus_Length   7

const int dspMenus[dspMenus_Length][4] = {
#define FREQ_MENU_TOP  95
#define TOP_MENU_TOP    0
#define FREQ_INFO_TOP  80

//BOTTOM MENU (5)
{0,   237,     0 + 80,    280},    //Exit
{80,  237,    80 + 100,   280},    //Menu1
{180, 237,   180 + 100,   280},    //Menu2
{280, 237,   280 + 100,   280},    //Menu3
{380, 237,   380 + 100,   280},    //Menu4

//AF MENU
{280, 195,   280 + 100,   236},    //AFMENU0
{380, 195,   380 + 100,   236},    //AFMENU1

//FREQ MENU (4)
//{ 80, FREQ_MENU_TOP, 180, FREQ_MENU_TOP + 37},    //FREQ2 //Before Band
//{180, FREQ_MENU_TOP, 280, FREQ_MENU_TOP + 37},    //FREQ3 //Next Band

//TOP MENU PROTOCOL (5)
//{ 80, TOP_MENU_TOP, 180, TOP_MENU_TOP + 37},    //FREQ2 //WSPR
//{180, TOP_MENU_TOP, 280, TOP_MENU_TOP + 37},    //FREQ3 //FT8
//{280, TOP_MENU_TOP, 380, TOP_MENU_TOP + 37},    //FREQ4 //JT65
//{380, TOP_MENU_TOP, 480, TOP_MENU_TOP + 37},    //FREQ5 //JT9

};

//uint8_t isOnAir = 1;


#define DSP_FILTER_NONE     0
#define DSP_FILTER_BPF      1
#define DSP_FILTER_USER     2

//for FILTER
#define DSP_FILTER_BYPASS   0
#define DSP_FILTER_BPF_50   1
#define DSP_FILTER_BPF_100  2
#define DSP_FILTER_BPF_150  3
#define DSP_FILTER_LPF      4
#define DSP_FILTER_HPF      5
#define DSP_FILTER_SSB      10   //DUAL LPF, HPF

//Filter Type
/*
uint8_t DSP_OUT_VOL       = 100;                //0
uint8_t isTmpMute         = 0;                  //1
uint8_t nowSelectedFilter = DSP_FILTER_NONE;    //2
uint8_t nowBPFFilterIndex = 0;  //              //3
uint8_t filterType        = DSP_FILTER_BYPASS;  //4
int filterApplyFreq1 	= 1000;                 //5~8
int filterApplyFreq2    = 200;  //HPF           //9~12
int userFilterHalfWidth = 100;                  //13~16
*/

static char g_dsp_data[20]  = {0};
uint8_t *DSP_OUT_VOL        = &g_dsp_data[0];                //0
uint8_t *isTmpMute          = &g_dsp_data[1];                  //1
uint8_t *nowSelectedFilter  = &g_dsp_data[2];    //2
uint8_t *nowBPFFilterIndex  = &g_dsp_data[3];  //              //3
uint8_t *filterType         = &g_dsp_data[4];  //4
int *filterApplyFreq1 	    = (int *)&g_dsp_data[5];                 //5~8
int *filterApplyFreq2       = (int *)&g_dsp_data[9];  //HPF           //9~12
int *userFilterHalfWidth    = (int *)&g_dsp_data[13];                  //13~16

#include "ff.h"
#include "crash.h"
static const char *g_dsp_fpath = "/aa/audiodsp.bin";

void DSP_StoredInformatoin(void)
{
    FRESULT res;
    FIL fo = { 0 };
    res = f_open(&fo, g_dsp_fpath, FA_OPEN_ALWAYS | FA_WRITE);
    if (FR_OK == res)
    {
        UINT bw;
        res = f_write(&fo, g_dsp_data, sizeof(g_dsp_data), &bw);
        res = f_close(&fo);
    }
}


void DSP_LoadInformation(void)
{
    FRESULT res;
    FIL fo = { 0 };

    FILINFO finfo;
    res = f_stat(g_dsp_fpath, &finfo);
    if (FR_NOT_ENABLED == res || FR_NOT_READY == res)
        CRASH("No SD card");

    if (FR_OK == res)
    {
        res = f_open(&fo, g_dsp_fpath, FA_READ);

        UINT br;
        f_read(&fo, g_dsp_data, sizeof(g_dsp_data), &br);
        f_close(&fo);
    }

    //Default Setting
    if ((*DSP_OUT_VOL < 10) || (*DSP_OUT_VOL > 256) || (*filterApplyFreq1) < 200 || (*filterApplyFreq2 < 200))  //Not Load Configuration
    {
        *DSP_OUT_VOL         = 100;                //0
        *nowSelectedFilter   = DSP_FILTER_NONE;    //2
        *nowBPFFilterIndex   = 0;  //              //3
        *filterType          = DSP_FILTER_BYPASS;  //4
        *filterApplyFreq1 	= 1000;                 //5~8
        *filterApplyFreq2    = 500;  //HPF           //9~12
        *userFilterHalfWidth = 500;                  //13~16
    }

    *isTmpMute         = 0;                  //1


}

//==============================================================================
//DISPLAY Protocol Information
//------------------------------------------------------------------------------
#define INFO_TOP   130
#define INFO_LINE2 183

//#define TITLE_COLOR LCD_RGB(0, 63, 119)
#define TITLE_COLOR LCD_RGB(2, 26, 39)

void dspMenuDraw(void)
{
    uint32_t LCSaveColor = TextColor;

    //Bottom Menu
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_EXIT  ][BUTTON_LEFT], dspMenus[MENU_EXIT  ][BUTTON_TOP]), imgbtn_home2, imgbtn_home2_size);
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_SAVECONFIG  ][BUTTON_LEFT], dspMenus[MENU_SAVECONFIG  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_BPF  ][BUTTON_LEFT], dspMenus[MENU_BPF  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_USERFILTER  ][BUTTON_LEFT], dspMenus[MENU_USERFILTER  ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_BYPASS    ][BUTTON_LEFT], dspMenus[MENU_BYPASS    ][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);

    //Draw Text on Empty Button
    FONT_Write(FONT_FRAN, TextColor, 0, dspMenus[MENU_SAVECONFIG][BUTTON_LEFT] + 10, dspMenus[MENU_SAVECONFIG][BUTTON_TOP] + 10, "Save Config");

    char bpfBuff[30];
    sprintf(bpfBuff, "Band Pass");

    if (*nowBPFFilterIndex == 0)
    {
        sprintf(bpfBuff, "BPF(50Hz)");
    }
    else if (*nowBPFFilterIndex == 1)
    {
        sprintf(bpfBuff, "BPF(100Hz)");
    }
    else if (*nowBPFFilterIndex == 2)
    {
        sprintf(bpfBuff, "BPF(150Hz)");
    }


    FONT_Write(FONT_FRAN, *nowSelectedFilter == DSP_FILTER_BPF ? SELECT_FREQ_COLOR : TextColor, 0, dspMenus[MENU_BPF][BUTTON_LEFT] + 20, dspMenus[MENU_BPF][BUTTON_TOP] + 10, bpfBuff);
    LCD_FillRect(LCD_MakePoint(dspMenus[MENU_BPF][BUTTON_LEFT] + 5, dspMenus[MENU_BPF][BUTTON_TOP] + 8),
                 LCD_MakePoint(dspMenus[MENU_BPF][BUTTON_LEFT] + 15, dspMenus[MENU_BPF][BUTTON_TOP] + 27), *nowSelectedFilter == DSP_FILTER_BPF ? LCD_BLUE : LCD_RGB(50, 50, 50));

    FONT_Write(FONT_FRAN, *nowSelectedFilter == DSP_FILTER_USER ? SELECT_FREQ_COLOR : TextColor, 0, dspMenus[MENU_USERFILTER][BUTTON_LEFT] + 20, dspMenus[MENU_USERFILTER][BUTTON_TOP] + 10, "User Filter");

    LCD_FillRect(LCD_MakePoint(dspMenus[MENU_USERFILTER][BUTTON_LEFT] + 5, dspMenus[MENU_USERFILTER][BUTTON_TOP] + 8),
                 LCD_MakePoint(dspMenus[MENU_USERFILTER][BUTTON_LEFT] + 15, dspMenus[MENU_USERFILTER][BUTTON_TOP] + 27), *nowSelectedFilter == DSP_FILTER_USER ? LCD_BLUE : LCD_RGB(50, 50, 50));

    FONT_Write(FONT_FRAN, *nowSelectedFilter == DSP_FILTER_NONE ? SELECT_FREQ_COLOR : TextColor, 0, dspMenus[MENU_BYPASS][BUTTON_LEFT] + 20, dspMenus[MENU_BYPASS][BUTTON_TOP] + 10, "Pass through");
    LCD_FillRect(LCD_MakePoint(dspMenus[MENU_BYPASS][BUTTON_LEFT] + 5, dspMenus[MENU_BYPASS][BUTTON_TOP] + 8),
                 LCD_MakePoint(dspMenus[MENU_BYPASS][BUTTON_LEFT] + 15, dspMenus[MENU_BYPASS][BUTTON_TOP] + 27), *nowSelectedFilter == DSP_FILTER_NONE ? LCD_BLUE : LCD_RGB(50, 50, 50));


    //Freq Menu
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_AFFREQDOWN][BUTTON_LEFT], dspMenus[MENU_AFFREQDOWN][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    LCD_DrawBitmap(LCD_MakePoint(dspMenus[MENU_AFFREQUP][BUTTON_LEFT], dspMenus[MENU_AFFREQUP][BUTTON_TOP]), imgbtn_empty2, imgbtn_empty2_size);
    FONT_Write(FONT_FRAN, TextColor, 0, dspMenus[MENU_AFFREQDOWN][BUTTON_LEFT] + 10, dspMenus[MENU_AFFREQDOWN][BUTTON_TOP] + 10, "Fine Tune -1Hz");
    FONT_Write(FONT_FRAN, TextColor, 0, dspMenus[MENU_AFFREQUP][BUTTON_LEFT] + 10, dspMenus[MENU_AFFREQUP][BUTTON_TOP] + 10, "Fine Tune +1Hz");
}


void ApplyBiquad(int fltType, arm_biquad_cascade_df2T_instance_f32 instMain, arm_biquad_cascade_df2T_instance_f32 instSecond, q15_t *srcBuff, q15_t *destBuff)
{
    //SpectrumBuff[buffIndex + i * 2] = inBuff[i * 2] * (WINDOW_FUNC((double)(i-(double)FFT_AUDIO_SIZE/2) / (double)((double)FFT_AUDIO_SIZE/2)));    //1 Channel of streo => Good!!!

    //Just Main Filterring
    for (int i = 0; i < iirLength / 2; i++)
    {
        //APPLY WINDOW FUNCTION
        //srcBuff[i * 2 + 1] *= (WINDOW_FUNC((double)(i-(double)FFT_AUDIO_SIZE/2) / (double)((double)FFT_AUDIO_SIZE/2)));    //1 Channel of streo => Good!!!

        srcBuff[i * 2 + 1] = 0;
    }

	if (fltType == DSP_FILTER_BYPASS)
	{
		//bypass
		memcpy(destBuff, srcBuff, AUDIO_BUFF_SIZE);	//512
	}
	else if (fltType == DSP_FILTER_SSB)
	{
		arm_q15_to_float (srcBuff, (float32_t *)float_buffer_in, iirLength);
		//Apply Main Filter
		arm_biquad_cascade_df2T_f32(&instMain, (float32_t *)float_buffer_in, (float32_t *)float_buffer_out, iirLength);

		//Apply Second Filter
		arm_biquad_cascade_df2T_f32(&instSecond, (float32_t *)float_buffer_out, (float32_t *)float_buffer_in, iirLength);
		arm_float_to_q15 ((float32_t *)float_buffer_in, destBuff, iirLength);
	}
	else
	{
		//Just Main Filterring
		//Apply Main Filter
		arm_q15_to_float (srcBuff, (float32_t *)float_buffer_in, iirLength);
		arm_biquad_cascade_df2T_f32(&instMain, (float32_t *)float_buffer_in, (float32_t *)float_buffer_out, iirLength);
		arm_float_to_q15 ((float32_t *)float_buffer_out, destBuff, iirLength);
	}
}

//FILTER INFORMATION
int FLT_COUNT				        = FLT_LPF_COUNT;
const int *FLT_INDEXS 	            = FLT_LPF_INDEX;
//const float (*APPLY_FILTER)[30]     = FLT_LPF;
//uint32_t FLT_MINFREQ 		        = FLT_LPF_MINFREQ;
//uint32_t FLT_MAXFREQ 		        = FLT_LPF_MAXFREQ;
//const float (*APPLY_FILTER_HPF)[30]	= FLT_HPF;

int filterStartIndex 	= (1000 - 50) / FFT_BIN_FREQ;
int filterEndIndex 	    = (1000 + 50) / FFT_BIN_FREQ;
int filterStartIndexMin = (1000 - 50) / FFT_BIN_FREQ;
int filterEndIndexMax 	= (1000 + 50) / FFT_BIN_FREQ;

int findCenterFreq(int targetFreq)
{
	if (targetFreq <= FLT_INDEXS[0])
	{
		return 0;
	}
	else if (targetFreq >= FLT_INDEXS[FLT_COUNT -1])
	{
		return FLT_COUNT -1;
	}

	for (int i = FLT_COUNT - 1; i >= 0; i--)
	{
		if (FLT_INDEXS[i] <= targetFreq)
		{
			return i;
		}
	}
	return 0;
}


float FloatInterpolation(float y1, float y2, float y3, 				//values for frequencies x1, x2, x3
                         float x1, float x2, float x3,      //frequencies of respective y values
                         float x) //Frequency between x2 and x3 where we want to interpolate result
{
	float a = ((y3 - y2) / (x3 - x2) - (y2 - y1) / (x2 - x1)) / (x3 - x1);
	float b = ((y3 - y2) / (x3 - x2) * (x2 - x1) + (y2 - y1) / (x2 - x1) * (x3 - x2)) / (x3 - x1);
	float res = a * powf(x - x2, 2.0) + b * (x - x2) + y2;

	return res;
}

void ApplyCoeffs(uint32_t targetFreq)
{
	int x1, x2 , x3;
	float y1, y2, y3;

  int baseIndex = findCenterFreq(targetFreq);

	if (baseIndex == 0)
	{
		baseIndex = 1;
	}
	else if (baseIndex >= FLT_COUNT -1)
	{
		baseIndex = FLT_COUNT -2;
	}

	x1 = FLT_INDEXS[baseIndex -1];
	x2 = FLT_INDEXS[baseIndex];
	x3 = FLT_INDEXS[baseIndex +1];

	for (int i = 0; i < 30; i++)
	{
        if (*filterType == DSP_FILTER_BPF_50)
        {
            y1 = FLT_BPF_50[baseIndex -1][i];
            y2 = FLT_BPF_50[baseIndex][i];
            y3 = FLT_BPF_50[baseIndex +1][i];
        }
        else if (*filterType == DSP_FILTER_BPF_100)
        {
            y1 = FLT_BPF_100[baseIndex -1][i];
            y2 = FLT_BPF_100[baseIndex][i];
            y3 = FLT_BPF_100[baseIndex +1][i];
        }
        else if (*filterType == DSP_FILTER_BPF_150)
        {
            y1 = FLT_BPF_150[baseIndex -1][i];
            y2 = FLT_BPF_150[baseIndex][i];
            y3 = FLT_BPF_150[baseIndex +1][i];
        }
        else if (*filterType == DSP_FILTER_LPF || *filterType == DSP_FILTER_SSB)
        {
            y1 = FLT_LPF[baseIndex -1][i];
            y2 = FLT_LPF[baseIndex][i];
            y3 = FLT_LPF[baseIndex +1][i];
        }
        else if (*filterType == DSP_FILTER_HPF)
        {
            y1 = FLT_HPF[baseIndex -1][i];
            y2 = FLT_HPF[baseIndex][i];
            y3 = FLT_HPF[baseIndex +1][i];
        }

		iirCoeffBuff[i] = FloatInterpolation(y1, y2, y3, x1, x2, x3,targetFreq);
	}

    char buff[64];
    char buffInfo[64];

    //sprintf(buff, "CENTER FREQ : %d ", *filterType, filterApplyFreq1, filterApplyFreq2);
    if (*filterType == DSP_FILTER_BYPASS)
    {
        sprintf(buff, "PASS THROUGH MODE");
        sprintf(buffInfo, "Filter : Pass through");
    }
    else if (*filterType == DSP_FILTER_SSB)
    {
        sprintf(buff, "FREQ : %d ~ %d ", *filterApplyFreq2, *filterApplyFreq1);
        sprintf(buffInfo, "Filter : BPF %dHz", *nowBPFFilterIndex == 0 ? 50 : (*nowBPFFilterIndex == 1 ? 100 : 200));
    }
    else
    {
        sprintf(buff, "CENTER FREQ : %d ", *filterApplyFreq1);
        sprintf(buffInfo, "Filter : User Filter ");
    }

    //LCD_FillRect(LCD_MakePoint(0, 190), LCD_MakePoint(275, 234), LCD_RGB(45, 45, 45));
    LCD_FillRect(LCD_MakePoint(0, 190), LCD_MakePoint(275, 234), LCD_RGB(8, 34, 47));

    LCD_HLine(LCD_MakePoint(0, 190), 479, LCD_BLACK);
    LCD_VLine(LCD_MakePoint(0, 190),  235, LCD_BLACK);
    LCD_HLine(LCD_MakePoint(0, 234), 479, LCD_WHITE);
    LCD_VLine(LCD_MakePoint(479, 234),  234, LCD_WHITE);

    FONT_Write(FONT_FRAN, TextColor, 0, 10, 191, buffInfo);
    FONT_Write(FONT_FRANBIG, TextColor, 0, 10, 202, buff);
}

void ApplyCoeffs_HPF(uint32_t targetFreq)
{
	int x1, x2 , x3;

  int baseIndex = findCenterFreq(targetFreq);

	if (baseIndex == 0)
	{
		baseIndex = 1;
	}
	else if (baseIndex >= FLT_COUNT -1)
	{
		baseIndex = FLT_COUNT -2;
	}

	x1 = FLT_INDEXS[baseIndex -1];
	x2 = FLT_INDEXS[baseIndex];
	x3 = FLT_INDEXS[baseIndex +1];
	for (int i = 0; i < 30; i++)
	{
		iirCoeffBuff_HPF[i] = FloatInterpolation(FLT_HPF[baseIndex -1][i], FLT_HPF[baseIndex][i], FLT_HPF[baseIndex +1][i],
			x1, x2, x3,targetFreq);
	}
}

void SetFFTFilter(uint8_t newFilter, arm_biquad_cascade_df2T_instance_f32 instMain, arm_biquad_cascade_df2T_instance_f32 instSecond)
{
    if (newFilter == DSP_FILTER_BPF_50)
    {
        FLT_COUNT				= FLT_BPF_50_COUNT;
        FLT_INDEXS 			    = FLT_BPF_50_INDEX;
        //FLT_MINFREQ 		    = FLT_BPF_50_MINFREQ;
        //FLT_MAXFREQ 		    = FLT_BPF_50_MAXFREQ;

    }
    else if (newFilter == DSP_FILTER_BPF_100)
    {
        FLT_COUNT				= FLT_BPF_100_COUNT;
        FLT_INDEXS 			    = FLT_BPF_100_INDEX;
        //FLT_MINFREQ 		    = FLT_BPF_100_MINFREQ;
        //FLT_MAXFREQ 		    = FLT_BPF_100_MAXFREQ;
    }
    else if (newFilter == DSP_FILTER_BPF_150)
    {
        FLT_COUNT				= FLT_BPF_150_COUNT;
        FLT_INDEXS 			    = FLT_BPF_150_INDEX;
        //FLT_MINFREQ 		    = FLT_BPF_150_MINFREQ;
        //FLT_MAXFREQ 		    = FLT_BPF_150_MAXFREQ;
    }
    else if (newFilter == DSP_FILTER_LPF)
    {
        FLT_COUNT				= FLT_LPF_COUNT;
        FLT_INDEXS 			    = FLT_LPF_INDEX;
        //FLT_MINFREQ 		    = FLT_LPF_MINFREQ;
        //FLT_MAXFREQ 		    = FLT_LPF_MAXFREQ;
    }
    else if (newFilter == DSP_FILTER_HPF)
    {
        FLT_COUNT				= FLT_HPF_COUNT;
        FLT_INDEXS 			    = FLT_HPF_INDEX;
        //FLT_MINFREQ 		    = FLT_HPF_MINFREQ;
        //FLT_MAXFREQ 		    = FLT_HPF_MAXFREQ;
    }
    else if (newFilter == DSP_FILTER_SSB)
    {
        FLT_COUNT				= FLT_LPF_COUNT;
        FLT_INDEXS 			    = FLT_LPF_INDEX;
        //FLT_MINFREQ 		    = FLT_LPF_MINFREQ;
        //FLT_MAXFREQ 		    = FLT_LPF_MAXFREQ;

        //APPLY_FILTER_HPF    	= FLT_HPF;
    }

    *filterType = newFilter;

    ApplyCoeffs(*filterApplyFreq1);
    if (newFilter == DSP_FILTER_SSB)
    {
        ApplyCoeffs_HPF(*filterApplyFreq1);
    }

    SetCenterFrequency(instMain, instSecond, *filterApplyFreq1, 1);
    dspMenuDraw();
}


extern void BSP_LCD_HLineShift(uint16_t startYY, uint16_t endYY, uint16_t startXX,  uint16_t endXX);
extern void BSP_LCD_DrawColorLine(uint16_t posYY, uint16_t startXX,  uint16_t endXX, uint32_t *lineColorInfoBuff);

uint32_t GetWaterfallColor(uint32_t fftLevel)
{
    if (fftLevel < 256)
    {
        return LCD_RGB(0, fftLevel, 255);
    }
    else if (fftLevel < 512)
    {
        return LCD_RGB(0, 255, 255 - fftLevel);
    }
    else if (fftLevel < 768)
    {
        return LCD_RGB(fftLevel, 255, 0);
    }
    else if (fftLevel < 1024)
    {
        return LCD_RGB(255, 255 - fftLevel, 0);
    }
    else
    {
        return LCD_RED;
    }
}



#include "arm_const_structs.h"

int spectrumStep = 0;
//uint32_t lineColorInfoBuff[480];
uint32_t lineColorInfoBuff[480];

extern void BSP_LCD_Draw2ColorVLine(uint16_t Xpos, uint16_t posYY0, uint32_t RGB_Code0, uint16_t posYY1, uint32_t RGB_Code1, uint16_t posYY2 );

void DrawSpectrum(int16_t *inBuff)
{
    //512 / 256
    if (spectrumStep < FFT_SPECTRUM_STEP)
    {
        int buffIndex = spectrumStep * FFT_AUDIO_SIZE * 2;
        for (int i = 0; i < FFT_AUDIO_SIZE; i++)    //HALF -> COMPLETE
        {
            SpectrumBuff[buffIndex + i * 2] = inBuff[i * 2];// * windowData[i];   //1 Channel of streo => Good!!!
            SpectrumBuff[buffIndex + i * 2 + 1] = 0;
        }
    }

    spectrumStep++;

    if (spectrumStep == FFT_SPECTRUM_STEP + 1)
    {
#define LCD_WATER_RGB(r, g, b) ((LCDColor) ( 0xFF000000ul | \
                             (( ((uint32_t)(r)) & 0xFF) << 16) |  \
                             (( ((uint32_t)(g)) & 0xFF) << 8) |   \
                             (  ((uint32_t)(b)) & 0xFF)           \
                         ))

        arm_cfft_f32(&arm_cfft_sR_f32_len2048, SpectrumBuff, 0, 1);
        arm_cmplx_mag_f32(SpectrumBuff, testOutput, FFT_SPECTRUM_SIZE);

        for (int i = 0; i < 480; i++)
        {
            uint32_t colorLevel = testOutput[i] / 200;
            lineColorInfoBuff[i] = GetWaterfallColor(colorLevel);

            int freqLength = testOutput[i] / 7000;

            if (freqLength > GRAPH_BOTTOM)
                freqLength = GRAPH_BOTTOM;

            testOutput[i] = freqLength;
        }
    }
    else if (spectrumStep >= FFT_SPECTRUM_STEP + 2)
    {
        //Display Graph
        int dispIndex = spectrumStep - (FFT_SPECTRUM_STEP + 2);
        #define DISP_LINE_BY_STEP   48 //120
        #define MAX_DISP_INDEX      10  //4

        if (dispIndex < MAX_DISP_INDEX)
        {
            for (int i = 0; i < DISP_LINE_BY_STEP; i++)
            {
                int pixelIndex = dispIndex * DISP_LINE_BY_STEP + i;

                uint32_t FFT_BACK_COLOR     = LCD_BLACK;
                uint32_t FFT_GRAPH_COLOR    = LCD_WHITE;

                if (*filterType != DSP_FILTER_BYPASS)
                {
                    //Check Selected range
                    if (filterStartIndex <= pixelIndex && filterEndIndex >= pixelIndex)
                    {
                        //FFT_BACK_COLOR     = LCD_RGB(59, 59, 59);
                        //FFT_BACK_COLOR     = LCD_RGB(80, 90, 233);
                        FFT_BACK_COLOR     = LCD_RGB(59, 59, 143);
                        //FFT_GRAPH_COLOR    = LCD_RGB(255, 127, 39);
                        FFT_GRAPH_COLOR    = LCD_RGB(230, 70, 70);
                    }
                    else if (filterStartIndexMin <= pixelIndex && filterEndIndexMax >= pixelIndex)
                    {
                        FFT_BACK_COLOR     = LCD_RGB(30, 30, 30);
                        FFT_GRAPH_COLOR    = LCD_RGB(255, 127, 39);
                    }
                }   //end of if

                BSP_LCD_Draw2ColorVLine(pixelIndex, 0, FFT_BACK_COLOR, GRAPH_BOTTOM - testOutput[pixelIndex], FFT_GRAPH_COLOR, GRAPH_BOTTOM);
            }

            BSP_LCD_HLineShift(GRAPH_WATERFALL_TOP, GRAPH_WATERFALL_BOTTOM, dispIndex * DISP_LINE_BY_STEP, dispIndex * DISP_LINE_BY_STEP + DISP_LINE_BY_STEP -1);
        }
        else
        {
            //BSP_LCD_DrawColorLine
            //LCD_RGB(100, 100, 200);

            //BSP_LCD_HLineShift(GRAPH_BOTTOM + 1, 150, 0, 479);
            BSP_LCD_DrawColorLine(GRAPH_WATERFALL_TOP, 0,  479, lineColorInfoBuff);

            spectrumStep = 0;
        }

    }
}


void DrawFFTSpectrumLayout()
{
    #define METER_H_LINE_COLOR LCD_RGB(255, 127, 39)
    char aBuff[32];
    LCD_FillRect(LCD_MakePoint(0, GRAPH_BOTTOM + 1), LCD_MakePoint(479, GRAPH_BOTTOM + METER_HEIGHT -1), LCD_BLACK);
    LCD_Line(LCD_MakePoint(0, GRAPH_BOTTOM + 1), LCD_MakePoint(479, GRAPH_BOTTOM + 1), METER_H_LINE_COLOR);
    //LCD_Line(LCD_MakePoint(0, GRAPH_BOTTOM + METER_HEIGHT -1), LCD_MakePoint(479, GRAPH_BOTTOM + METER_HEIGHT -1), METER_H_LINE_COLOR);
    //LCD_FillRect(LCD_MakePoint(0, GRAPH_BOTTOM + 1), LCD_MakePoint(479, GRAPH_BOTTOM + 10), LCD_BLACK);
    for (int i = 0; i < 480; i += 8)
    {
        LCD_Line(LCD_MakePoint(i, GRAPH_BOTTOM + 1), LCD_MakePoint(i, GRAPH_BOTTOM + 2), METER_H_LINE_COLOR);

        if ((i % 128) == 0)
        {
            sprintf(aBuff, "%dKhz", i / 128);
            FONT_Write(FONT_FRAN, LCD_WHITE, 0, i + 2, GRAPH_BOTTOM + METER_HEIGHT - 14, aBuff);
            LCD_Line(LCD_MakePoint(i, GRAPH_BOTTOM + 1), LCD_MakePoint(i, GRAPH_BOTTOM + 10), METER_H_LINE_COLOR);
        }
        else if ((i % 64) == 0)
        {
            sprintf(aBuff, "%d", i / 64 * 512);
            FONT_Write(FONT_SDIGITS, LCD_WHITE, 0, i + 2, GRAPH_BOTTOM + METER_HEIGHT - 10, aBuff);
            LCD_Line(LCD_MakePoint(i, GRAPH_BOTTOM + 1), LCD_MakePoint(i, GRAPH_BOTTOM + 5), METER_H_LINE_COLOR);
        }
    }

    //LCD_MakePoint(0, GRAPH_WATERFALL_TOP)
    LCD_FillRect(LCD_MakePoint(0, GRAPH_WATERFALL_TOP), LCD_MakePoint(479, GRAPH_WATERFALL_BOTTOM), GetWaterfallColor(0));
    FONT_Write(FONT_FRANBIG, LCD_WHITE, 0, 50, GRAPH_WATERFALL_TOP + 3, "+DSP Version 0.5 / Audio DSP");
    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 100, GRAPH_WATERFALL_TOP + 35, "for Amateur radio CW/SSB (300Hz ~ 3000Hz)");
    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 290, GRAPH_WATERFALL_TOP + 50, "by KD8CEC");
}

void SetCenterFrequency(arm_biquad_cascade_df2T_instance_f32 instMain, arm_biquad_cascade_df2T_instance_f32 instSecond, uint32_t newCenterFreq, uint32_t isChangedFilter)
{
    if ((newCenterFreq != *filterApplyFreq1) || isChangedFilter)
    {
        *filterApplyFreq1 = newCenterFreq;
        uint32_t newCenterFreqStart = newCenterFreq;
        uint32_t newCenterFreqEnd = newCenterFreq;
        int secondInterval = 50;

        if (*filterType == DSP_FILTER_BPF_50)
        {
            newCenterFreqStart = (*filterApplyFreq1 - 25);
            newCenterFreqEnd = (*filterApplyFreq1 + 25);
            secondInterval = 15;
        }
        else if (*filterType == DSP_FILTER_BPF_100)
        {
            newCenterFreqStart = (*filterApplyFreq1 - 50);
            newCenterFreqEnd = (*filterApplyFreq1 + 50);
            secondInterval = 15;
        }
        else if (*filterType == DSP_FILTER_BPF_150)
        {
            newCenterFreqStart = (*filterApplyFreq1 - 80);
            newCenterFreqEnd = (*filterApplyFreq1 + 80);
            secondInterval = 20;
        }
        else if (*filterType == DSP_FILTER_LPF || *filterType == DSP_FILTER_SSB)
        {
            newCenterFreqStart = 0;
            newCenterFreqEnd = (*filterApplyFreq1);
            secondInterval = 35;

            if (*filterType == DSP_FILTER_SSB)
            {
                newCenterFreqStart = (*filterApplyFreq2);
            }
        }
        else if (*filterType == DSP_FILTER_HPF)
        {
            secondInterval = 35;
            newCenterFreqStart = (*filterApplyFreq1);
            newCenterFreqEnd = 8000;
        }

        filterStartIndex = (float)newCenterFreqStart / FFT_BIN_FREQ;
        filterEndIndex = (float)newCenterFreqEnd     / FFT_BIN_FREQ;

        filterStartIndexMin = filterStartIndex - secondInterval;
        filterEndIndexMax = filterEndIndex + secondInterval;

        *isTmpMute = 5;
        BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
        ApplyCoeffs(*filterApplyFreq1);
        arm_biquad_cascade_df2T_init_f32(&instMain, iirStageCount, iirCoeffBuff, iirStateBuff);

        if (*filterType == DSP_FILTER_SSB)
        {
            ApplyCoeffs_HPF(*filterApplyFreq2);
            arm_biquad_cascade_df2T_init_f32(&instSecond, iirStageCount, iirCoeffBuff_HPF, iirStateBuff_HPF);
        }
    }
}

void SetNewFreqByTouch(int newFreq, int isCenter, arm_biquad_cascade_df2T_instance_f32 instMain, arm_biquad_cascade_df2T_instance_f32 instSecond)
{
    if (*filterType == DSP_FILTER_SSB)
    {
        int basicFreq = (*filterApplyFreq1 - *filterApplyFreq2) / 2 + *filterApplyFreq2;

        if (isCenter)
        {
            //diff value
            int diffFreq = newFreq - basicFreq;
            //HPF
            *filterApplyFreq2 += diffFreq;
            *filterApplyFreq1 += diffFreq;

            if (*filterApplyFreq2 < FILTER_MINIMUM_FREQ)
            {
                *filterApplyFreq2 = FILTER_MINIMUM_FREQ;
            }
            if (*filterApplyFreq1 < FILTER_MINIMUM_FREQ + 100)
            {
                *filterApplyFreq1 = FILTER_MINIMUM_FREQ + 100;
            }

            if (*filterApplyFreq1 > FILTER_MAXIMUM_FREQ)
            {
                *filterApplyFreq1 = FILTER_MAXIMUM_FREQ;
            }

            SetCenterFrequency(instMain, instSecond, *filterApplyFreq1, 1);
        }
        else if (newFreq < basicFreq)
        {
            //HPF
            if (newFreq >= FILTER_MINIMUM_FREQ && *filterApplyFreq2 != newFreq)
            {
                *filterApplyFreq2 = newFreq;

                SetCenterFrequency(instMain, instSecond, *filterApplyFreq1, 1);
            }
        }
        else
        {
            //LPF
            if (newFreq <= FILTER_MAXIMUM_FREQ && *filterApplyFreq1 != newFreq)
            {
                //LPF
                if (newFreq < FILTER_MINIMUM_FREQ + 100)
                {
                    newFreq = FILTER_MINIMUM_FREQ + 100;
                }
                SetCenterFrequency(instMain, instSecond, newFreq, 0);
            }

        }

        *userFilterHalfWidth = (*filterApplyFreq1 - *filterApplyFreq2) / 2;
    }
    else
    {
        SetCenterFrequency(instMain, instSecond, newFreq, 0);
    }
}

void AudioDSP_Proc(void)
{
    int checkCount = 0;     //Check Measure Interval
    LCDPoint pt;

    SetColours();
    BSP_LCD_SelectLayer(0);
    LCD_FillAll(BACK_COLOR);
    BSP_LCD_SelectLayer(1);
    LCD_FillAll(BACK_COLOR);
    LCD_ShowActiveLayerOnly();

    //Load Configuration
    DSP_LoadInformation();


    DrawFFTSpectrumLayout();
    dspMenuDraw();

    while(TOUCH_IsPressed());

    if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, OUTPUT_DEVICE_HEADPHONE, 100, I2S_AUDIOFREQ_16K) == AUDIO_OK)

    {
        #ifdef _DEBUG_UART
        DBG_Printf("Complete Audio Init...");
        #endif
    }
    else
    {
        #ifdef _DEBUG_UART
        DBG_Printf("Failed Init...");
        #endif
    }

    memset((uint16_t*)AUDIO_BUFFER_IN_FLT, 0, AUDIO_BUFF_SIZE * 2);		//Half  (512) -> Complete (512)
    memset((uint16_t*)AUDIO_BUFFER_OUT_FLT, 0, AUDIO_BUFF_SIZE * 2);		//Half  (512) -> Complete (512)
    Audio_Status = AUDIO_TRANSFER_NONE;

    BSP_AUDIO_IN_Record((uint16_t*)AUDIO_BUFFER_IN_FLT, AUDIO_BUFF_SIZE);

    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_OUT_FLT, AUDIO_BUFF_SIZE * 2);

    BSP_AUDIO_IN_SetVolume(100);
    BSP_AUDIO_OUT_SetVolume(*DSP_OUT_VOL);
    BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);

	arm_biquad_cascade_df2T_instance_f32 instMain;
	arm_biquad_cascade_df2T_instance_f32 instSecond;

	//uint8_t *filterType = DSP_FILTER_BYPASS;
    SetFFTFilter(*filterType, instMain, instSecond);
	arm_biquad_cascade_df2T_init_f32(&instMain, iirStageCount, iirCoeffBuff, iirStateBuff);

    ApplyCoeffs_HPF(*filterApplyFreq2);
    arm_biquad_cascade_df2T_init_f32(&instSecond, iirStageCount, iirCoeffBuff_HPF, iirStateBuff_HPF);

    int skipCount = 0;  //for while button process, processing DSP

    for(;;)
    {
		while(Audio_Status != AUDIO_TRANSFER_HALF)
		{
            HAL_Delay(1);
			__NOP();
		}
		Audio_Status = AUDIO_TRANSFER_NONE;	//Receive Start
		ApplyBiquad(*filterType, instMain, instSecond, AUDIO_BUFFER_IN_FLT, AUDIO_BUFFER_OUT_FLT);
        DrawSpectrum(AUDIO_BUFFER_IN_FLT);

        if (*isTmpMute > 0)
        {
            if (--(*isTmpMute) <= 0)
            {
                //BSP_AUDIO_IN_SetVolume(*DSP_OUT_VOL);
                //BSP_AUDIO_OUT_SetVolume(*DSP_OUT_VOL);
                BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);
            }
        }

		while(Audio_Status != AUDIO_TRANSFER_COMPLETE)
		{
			HAL_Delay(1);
			__NOP();
		}

		Audio_Status = AUDIO_TRANSFER_NONE;

        DrawSpectrum(&AUDIO_BUFFER_IN_FLT[256]);
		ApplyBiquad(*filterType, instMain, instSecond, &AUDIO_BUFFER_IN_FLT[256], &AUDIO_BUFFER_OUT_FLT[256]);

        //==========================================================================================
        //CHECK INPUT USER BUTTONS
        //------------------------------------------------------------------------------------------
        if (TOUCH_Poll(&pt))
        {
            int touchIndex = GetTouchIndex(pt, dspMenus, dspMenus_Length);

            //Play Beep
            if (touchIndex != -1)
            {
                //TRACK_Beep(1);
                //Chbeck AF Frequency Up And Down / for Continues change
                if ((touchIndex == MENU_AFFREQDOWN || touchIndex == MENU_AFFREQUP) && (*nowSelectedFilter != DSP_FILTER_NONE))  //+- 10Hz
                {
                    if (skipCount < 1)
                    {
                        int newFreq = *filterApplyFreq1;

                        if (*filterType == DSP_FILTER_SSB)
                        {
                            //newFreq = (*filterApplyFreq1 - *filterApplyFreq2) / 2 + *filterApplyFreq2 + (touchIndex == MENU_AFFREQDOWN ? -1 : 1);
                            newFreq = (*filterApplyFreq1 - *filterApplyFreq2) / 2 + *filterApplyFreq2;
                        }

                        newFreq += (touchIndex == MENU_AFFREQDOWN ? -1 : 1);

                        if (newFreq < FILTER_MINIMUM_FREQ)
                        {
                            newFreq = FILTER_MINIMUM_FREQ;
                        }
                        else if (newFreq > FILTER_MAXIMUM_FREQ)
                        {
                            newFreq = FILTER_MAXIMUM_FREQ;
                        }

                        SetNewFreqByTouch(newFreq, 1, instMain, instSecond);

                        skipCount = 5;
                    }
                    else
                    {
                        skipCount--;
                    }

                    //Sleep(70);
                    continue;
                }
                else
                {
                    *isTmpMute = 1;
                    BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);

                    TRACK_Beep(1);
                    while(TOUCH_IsPressed());
                }

                //while(TOUCH_IsPressed());

                if (touchIndex == MENU_EXIT)    //EXIT
                {
                    break;
                }
                else if (touchIndex == MENU_SAVECONFIG)
                {
                    DSP_StoredInformatoin();
                }
                else if (touchIndex == MENU_BPF)
                {
                    //Check before BPF
                    if (*nowSelectedFilter == DSP_FILTER_BPF)
                    {
                        (*nowBPFFilterIndex)++;

                        if (*nowBPFFilterIndex > 2)
                        {
                            *nowBPFFilterIndex = 0;
                        }
                    }
                    else if (*nowSelectedFilter == DSP_FILTER_USER)
                    {
                        *filterApplyFreq1 = *filterApplyFreq1 - *userFilterHalfWidth;
                    }


                    if (*nowBPFFilterIndex == 0)
                        *filterType = DSP_FILTER_BPF_50;
                    else if (*nowBPFFilterIndex == 1)
                        *filterType = DSP_FILTER_BPF_100;
                    else if (*nowBPFFilterIndex == 2)
                        *filterType = DSP_FILTER_BPF_150;

                    *nowSelectedFilter = DSP_FILTER_BPF;
                    //*filterType = DSP_FILTER_BPF_50;
                }
                else if (touchIndex == MENU_USERFILTER)
                {
                    if (*nowSelectedFilter != DSP_FILTER_USER)
                    {
                        //Adjust
                        if (*nowSelectedFilter == DSP_FILTER_BPF)
                        {
                            *filterApplyFreq2      =    *filterApplyFreq1 - *userFilterHalfWidth; //HPF
                            *filterApplyFreq1       =    *filterApplyFreq1 + *userFilterHalfWidth; //LPF
                        }

                        if (*filterApplyFreq2 < FILTER_MINIMUM_FREQ)
                        {
                            *filterApplyFreq2 = FILTER_MINIMUM_FREQ;  //HPF
                        }

                        if (*filterApplyFreq1 > FILTER_MAXIMUM_FREQ)
                        {
                            *filterApplyFreq1  =  FILTER_MAXIMUM_FREQ;  //HPF
                        }

                        *nowSelectedFilter = DSP_FILTER_USER;
                        *filterType = DSP_FILTER_SSB;
                    }
                }
                else if (touchIndex == MENU_BYPASS)
                {
                    *nowSelectedFilter = DSP_FILTER_NONE;
                    *filterType = DSP_FILTER_BYPASS;
                }

                SetFFTFilter(*filterType, instMain, instSecond);
            }
            else if (pt.y < 150)
            {
                int nowPos = pt.x;
                int newFreq = nowPos * FFT_BIN_FREQ;

                if (newFreq < FILTER_MINIMUM_FREQ)
                {
                    newFreq = FILTER_MINIMUM_FREQ;
                }
                else if (newFreq > FILTER_MAXIMUM_FREQ)
                {
                    newFreq = FILTER_MAXIMUM_FREQ;
                }

                if (*filterType == DSP_FILTER_SSB)
                {
                    if (newFreq != (*filterApplyFreq1 - *filterApplyFreq2) / 2 + *filterApplyFreq2)
                        SetNewFreqByTouch(newFreq, pt.y > GRAPH_BOTTOM, instMain, instSecond);
                }
                else
                {
                    if (newFreq != *filterApplyFreq1)
                        SetNewFreqByTouch(newFreq, pt.y > GRAPH_BOTTOM, instMain, instSecond);
                }

            }
        }   //end of if (check for touch screen)

    }   //end of for

    //SET_PTT(0);
    //GEN_SetMeasurementFreq(0);
    DSP_Init();
    return;

}
