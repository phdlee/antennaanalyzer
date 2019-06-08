/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   Modified by KD8CEC
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */


#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "font.h"
#include "touch.h"
#include "mainwnd.h"

#include "textbox.h"
#include "config.h"
#include "fftwnd.h"
#include "generator.h"
#include "measurement.h"
#include "oslcal.h"
#include "panvswr2.h"
#include "panfreq.h"
#include "main.h"
#include "usbd_storage.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "crash.h"
#include "dsp.h"
#include "gen.h"
#include "aauart.h"
#include "build_timestamp.h"
#include "tdr.h"
#include "screenshot.h"
#include "spectr.h"
#include "stm32_ub_adc3_single.h"
#include "DS3231.h"

//KD8CEC
#include "guicontrol.h"
#include "aaprotocol.h"

//#include "trackspectr.h"
extern void Track_Proc(void);
extern void TRACK_Beep(int duration);
extern void Sleep(uint32_t);
extern void WeakSignal_Proc(void);
static void Exit1(void);
void DisplayVoltage();

static uint32_t date, time;
uint32_t RTCpresent;

void Voltage(void);
void InitVoltage(void);

// MainWnd
static TEXTBOX_CTX_t main_ctx;
static TEXTBOX_t hbTimestamp;

// MenuTools,  MenuSettings
static TEXTBOX_CTX_t menu1_ctx;
static void MenuSettings(void);

// MenuCalibration
static TEXTBOX_CTX_t menu2_ctx;
static void MenuCalibration(void);

static TEXTBOX_CTX_t menu3_ctx;

// MenuRTC
static TEXTBOX_CTX_t menuRTC_ctx;
// Oscillator test
static TEXTBOX_CTX_t osci_ctx;

// Voltage
static int  VoltCase, Volt_max_Display, Volt_min_Display, Volt_max_Factor;
static uint32_t color1, color2;

int BattVoltage, Batt;
float VoltFloat, percent;
int cntr,CountMax;

static void Colours(void);

#define M_BGCOLOR LCD_RGB(0,0,64)    //Menu item background color
#define M_FGCOLOR LCD_RGB(255,255,0) //Menu item foreground color

#define COL1 10  //Column 1 x coordinate
#define COL2 230 //Column 2 x coordinate

static USBD_HandleTypeDef USBD_Device;
extern char SDPath[4];
extern FATFS SDFatFs;
extern uint8_t second;
static FILINFO fno;
static int voltage;
static bool rqExit1;

static void USBD_Proc()
{
    while(TOUCH_IsPressed());

    LCD_FillAll(LCD_BLACK);
    FONT_Write(FONT_FRANBIG, LCD_YELLOW, LCD_BLACK, 10, 0, "USB storage access via USB HS port");
    FONT_Write(FONT_FRANBIG, LCD_YELLOW, LCD_BLACK, 80, 200, "Exit (Reset device)");

    FATFS_UnLinkDriver(SDPath);
    BSP_SD_DeInit();
    Sleep(100);

    USBD_Init(&USBD_Device, &MSC_Desc, 0);
    USBD_RegisterClass(&USBD_Device, USBD_MSC_CLASS);
    USBD_MSC_RegisterStorage(&USBD_Device, &USBD_DISK_fops);
    USBD_Start(&USBD_Device);

    //USBD works in interrupts only, no need to leave CPU running in main.
    for(;;)
    {
        Sleep(50); //To enter low power if necessary
        LCDPoint coord;
        if (TOUCH_Poll(&coord))
        {
            while(TOUCH_IsPressed());
            if (coord.x > 80 && coord.x < 320 && coord.y > 200 && coord.y < 240)
            {
                USBD_Stop(&USBD_Device);
                USBD_DeInit(&USBD_Device);
                BSP_LCD_DisplayOff();
                BSP_SD_DeInit();
                Sleep(100);
                NVIC_SystemReset(); //Never returns
            }
        }
    }
}


//static TCHAR    fileNames[13][13]; remarked by KD8CEC
//static uint16_t Pointer;
static uint16_t  Line;
static uint16_t  Pmax;
static uint8_t EndFlag;

static uint16_t rqExit, rqExitR, newLoad, FileNo;
volatile int Page;

#define TBX0 190
#define Fieldw1 70
#define TBX1 300
#define Fieldw2 60
#define FieldH 36
#define TBY 90

void(Delete(void)){

    SCREENSHOT_DeleteFile(Line%12);

     newLoad=1;
}

void(ShowP(void)){

    SCREENSHOT_ShowPicture(Line%12);
    newLoad=1;
}

void(Next(void)){
uint16_t idx=Line%12;
    LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),BackGrColor);

    if(idx>=11)  {
        FileNo+=12;
        Page+=1;
        Line=0;
        newLoad=1;
        return;
    }
    if(idx>=Pmax-1) {
        EndFlag=0;
        Page=1;
        Line=0;
        FileNo=0;
        newLoad=1;
        return;
    }
    Line++;
    idx=Line%12;
    LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),CurvColor);
}

void(NextPage(void)){
uint16_t idx=Line%12;
    LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),BackGrColor);
    newLoad=1;
    if(EndFlag==1 ){
        EndFlag=0;
        Page=1;
        FileNo=0;
    }
    else     {
        FileNo+=12;
        Page+=1;
    }
    Line=0;
}

void(Prev(void)){
uint16_t idx=Line%12;

    LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),BackGrColor);// delete the "cursor"
    if((idx==0)&&(Page==1)){
        do{
            Pmax=SCREENSHOT_SelectFileNames(12*(Page-1));
            Page++;
            FileNo+=12;
        }
        while (11<Pmax);// up from first element --> go to end
        Page--;
        EndFlag=1;
        newLoad=1;
    }
    else if(idx>0) {
        Line--;
        idx=Line%12;
        LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),CurvColor);
    }
    else{
        Page--;
        FileNo-=12;
        newLoad=1;
    }
}

void Exit(void){
    rqExitR=1;
    InitVoltage();
}

static const TEXTBOX_t tb_reload[] = {
    (TEXTBOX_t){ .x0 = TBX0+200, .y0 = 10, .text = "Up", .font = FONT_FRANBIG, .width = Fieldw1, .height = FieldH, .center = 1,
                 .border = 1, .fgcolor = LCD_WHITE, .bgcolor = LCD_BLACK, .cb = (void(*)(void))Prev, .cbparam = 1, .next = (void*)&tb_reload[1] },
    (TEXTBOX_t){ .x0 = TBX0+200, .y0 = 10+FieldH+1, .text = "Down", .font = FONT_FRANBIG, .width = Fieldw1, .height = FieldH, .center = 1,
                 .border = 1, .fgcolor = LCD_WHITE, .bgcolor = LCD_BLACK, .cb = (void(*)(void))Next, .cbparam = 1,  .next = (void*)&tb_reload[2] },
    (TEXTBOX_t){ .x0 = TBX0+200, .y0 = 234, .text = "Delete", .font = FONT_FRANBIG, .width = Fieldw1, .height = FieldH, .center = 1,
                 .border = 1, .fgcolor = LCD_WHITE, .bgcolor = LCD_RED, .cb = (void(*)(void))Delete, .cbparam = 1,  .next = (void*)&tb_reload[3] },
    (TEXTBOX_t){ .x0 = TBX0+200, .y0 = TBY+18, .text = "Show", .font = FONT_FRANBIG, .width = Fieldw1, .height = FieldH, .center = 1,
                 .border = 1, .fgcolor = LCD_WHITE, .bgcolor = LCD_GREEN, .cb = (void(*)(void))ShowP, .cbparam = 1,  .next = (void*)&tb_reload[4] },
    (TEXTBOX_t){ .x0 = 2, .y0 = 234, .text = " Exit ", .font = FONT_FRANBIG, .width = 100, .height = FieldH, .center = 1,
                 .border = 1, .fgcolor = LCD_WHITE, .bgcolor = LCD_BLACK, .cb = (void(*)(void))Exit, .cbparam = 1, .next =(void*)&tb_reload[5] },
    (TEXTBOX_t){ .x0 = 80, .y0 = 234, .text = " Next Page", .font = FONT_FRANBIG, .width = 150, .height = FieldH, .center = 1,
                 .border = 1, .fgcolor = LCD_WHITE, .bgcolor = LCD_BLACK, .cb = (void(*)(void))NextPage, .cbparam = 1,},

};

void Reload_Proc(void){// WK ***************************************************************************************
    TEXTBOX_CTX_t fctx;
    LCDPoint pt;
    char str[16];
    uint32_t idx;

    SetColours();
    rqExitR=0;
    FileNo=0;
    Page=1;
    Line=0;
    LCD_FillAll(BackGrColor);
    TEXTBOX_InitContext(&fctx);
    TEXTBOX_Append(&fctx, (TEXTBOX_t*)tb_reload); //Append the very first element of the list in the flash.
                                                      //It is enough since all the .next fields are filled at compile time.
    TEXTBOX_DrawContext(&fctx);
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 0, 0, "Manage Snapshots");// WK
    newLoad=0;
    EndFlag=0;

    if(SCREENSHOT_SelectFileNames(0)<11) EndFlag=1;
    LCD_Rectangle(LCD_MakePoint(0,31), LCD_MakePoint(280,16+32),CurvColor);
    sprintf(str,"Page: %d",Page);
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 250, 235, str);
    do{
        Pmax=SCREENSHOT_SelectFileNames(12*(Page-1));
        Page++;
        FileNo+=12;
    }
    while (11<Pmax);// --> go to end
    Page--;
    EndFlag=1;
    newLoad=1;
    Line+=Pmax-1;
    idx=Line%12;
    LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),CurvColor);

    for(;;)
    {
        if (TEXTBOX_HitTest(&fctx))
        {
            Sleep(50);
        }
        if (rqExitR)
        {
            LCD_FillAll(BackGrColor);
            break;
        }
        Sleep(10);
        if(newLoad==1)
        {
            newLoad=0;

            LCD_FillRect(LCD_MakePoint(0,31), LCD_MakePoint(290,234),BackGrColor);

            TEXTBOX_InitContext(&fctx);
            TEXTBOX_Append(&fctx, (TEXTBOX_t*)tb_reload); //Append the very first element of the list in the flash.
            TEXTBOX_DrawContext(&fctx);
            FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 0, 0, "Manage Snapshots");// WK
            Pmax=SCREENSHOT_SelectFileNames(12*(Page-1));
            if(11>Pmax) EndFlag=1;
            LCD_Rectangle(LCD_MakePoint(0,Line*16+31), LCD_MakePoint(280,(Line%12)*16+16+32),CurvColor);
            sprintf(str,"Page: %d",Page);
            FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 250, 235, str);
        }

        //Check Touch FileName
        if (TOUCH_Poll(&pt))
        {
            if (pt.x < 300 && pt.y > 32 && pt.y < ((Pmax) * 16 + 32 ) )
            {
                TRACK_Beep(1);
                //Select
                while(TOUCH_IsPressed());

                idx=Line%12;
                LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),BackGrColor);
                int newIdx = (pt.y - 32) / 16;
                idx = newIdx;
                //Line = Page * 12 + idx;
                Line = idx;
                LCD_Rectangle(LCD_MakePoint(0,idx*16+31), LCD_MakePoint(280,idx*16+16+32),CurvColor);
            }

        }

    }
}


static uint8_t second1;

uint8_t TestDate(void){
int dd,mm,yyyy,hh,mi;
char txt[10], txt1[5];

    if((date<=19800101)||(date>20803112))return 7;
    sprintf(txt, "%08u", date);// date
    dd=atoi(&txt[6]);
    if((dd==0)||(dd>31)) return 1;// day false
    strncpy(txt1,&txt[4],2);// test month 0..12
    txt1[2]=0;
    mm=atoi(txt1);
    if((mm==0)||(mm>12))  return 2;// month false
    strncpy(txt1,&txt[4],4);// test month 0..12
    txt1[4]=0;
    yyyy=atoi(txt1);
    if(mm==2){
        if((yyyy%4==0)&&(yyyy%100!=0)){// leap-year ?
            if(dd>29) return 3;
        }
        else if(dd>28) return 4;
    }
    switch(mm){
        case 4:
        case 6:
        case 9:
        case 11:
            if(dd>30) return 5;
        default:
            if(dd>31) return 6;
    }
    return 0;
}

void SetInternTime(uint32_t timx){
    secondsCounter=second1+(timx%100)*60+(timx/100)*3600;
}
void NextDay(void){
    if(NoDate==1) return;
    date+=1;
    if(TestDate()==0) return;
    date = date-(date%100)+101;// month +1; day =1
    if(TestDate()==0) return;
    date = date-(date%10000)+10101;// year +1; moth = day = 1
}

uint32_t GetInternTime(uint8_t *secondsx){
uint32_t minutes;
    if(secondsCounter>86400){// 24 * 3600
        NextDay();
        CFG_SetParam(CFG_PARAM_Date,date);
        CFG_Flush();
        secondsCounter-=86400;
    }
    minutes=secondsCounter/60;
    *secondsx=(uint8_t)(secondsCounter%60);
    return (minutes%60+(minutes/60)*100);
}


//KD8CEC : RESIZE FONT AND CHANGE POSITION
static void DateTime(void){
uint32_t mon;
short AMPM1;
char text1[20];

    if(NoDate==1) return;
    if(RTCpresent){
        getTime(&time, &second1,&AMPM1,0);
        getDate(&date);

    }
    else{

        time=GetInternTime(&second1);
        date=CFG_GetParam(CFG_PARAM_Date);
    }
    mon=date%10000;
    sprintf(text1, "%04d %02d %02d  ", date/10000,mon / 100, mon % 100);
    FONT_Write(FONT_FRAN, CurvColor, LCD_RGB(64, 64, 64), 220, 257, text1);

    sprintf(text1, "%02d:%02d:%02d ", time/100, time % 100, second1);

    FONT_Write(FONT_FRAN, CurvColor, LCD_RGB(64, 64, 64), 305, 257, text1);
}


void Measure_LCR_Proc(void);
void AudioDSP_Proc(void);
void RecPlayAudio_Proc(void);

#ifdef _DEBUG_UART
extern void PrintOSLSize(void);
#endif

// ================================================================================================
// Main window procedure (never returns)
void MainWnd(void)
{
//uint8_t i;
int counter11;

    //By KD8CEC
    #ifdef _DEBUG_UART
    DBGUART_Init();
    DBG_Str("Debug Start for Antenna Analyzer!!! \r\n");
    #endif

    SetColours();
    BSP_LCD_SelectLayer(0);
    SetColours();

    //modified by KD8CEC
    //LCD_FillAll(LCD_BLACK);
    SetColours();

    //KD8CEC
    BSP_LCD_SelectLayer(1);
    LCD_ShowActiveLayerOnly();
    LCD_DrawBitmap(LCD_MakePoint(0, 0), mainimg_bmp, mainimg_bmp_size);


    while (TOUCH_IsPressed());

    //Initialize textbox context
    TEXTBOX_InitContext(&main_ctx);

    //modified by KD8CEC
    hbTimestamp = (TEXTBOX_t) {.x0 = 27, .y0 = 257, .text = "" AAVERSION "," BUILD_TIMESTAMP, .font = FONT_FRAN,
                            .fgcolor = LCD_GRAY, .bgcolor = LCD_RGB(64, 64, 64) };
    TEXTBOX_Append(&main_ctx, &hbTimestamp);

    //Draw context
    TEXTBOX_DrawContext(&main_ctx);

    PROTOCOL_Reset();
    InitVoltage();

    FONT_Write(FONT_FRAN, LCD_WHITE, 0, 420, 9, AAVERSION_CECV);

    counter11=98;

    for(;;)
    {

        Voltage();
        if (++counter11>100){
            DateTime();
            counter11=0;
        }
        Sleep(10); //for autosleep to work

        //KD8CEC
        LCDPoint pt;
        if (TOUCH_Poll(&pt))
        {
            int touchIndex = GetTouchIndex(pt, buttonsArea, BUTTON_COUNT);

            if (touchIndex > -1)
            {
                TRACK_Beep(1);
                while(TOUCH_IsPressed());

                switch (touchIndex)
                {

                //LINE 1
                case BTN_SINGLE :
                    Single_Frequency_Proc();
                    break;
                case BTN_SWEEP  :
                    PANVSWR2_Proc();
                    break;
                case BTN_MULTISWR :
                    MultiSWR_Proc();
                    break;
                case BTN_TUNESWR :
                    Tune_SWR_Proc();
                    break;
                case BTN_TDR :
                    TDR_Proc();
                    break;

                //LINE 2
                case BTN_FIND :
                    SPECTR_FindFreq();
                    break;
                case BTN_QUARTZ :
                    Quartz_proc();
                    break;
                case BTN_RFGEN :
                    GENERATOR_Window_Proc();
                    break;
                case BTN_SNAPSHOT :
                    Reload_Proc();
                    break;
                case BTN_USB :
                    USBD_Proc();
                    break;

                case BTN_LC :
                    Measure_LCR_Proc();
                    break;

                //LINE 3
                case BTN_CONFIG :
                    MenuSettings();
                    break;
                case BTN_DSP :
                    FFTWND_Proc();
                    break;

                case BTN_TRACKER :
                    Track_Proc();
                    break;

                case BTN_WSPR :
                    WeakSignal_Proc();
                    break;
                }

                while(TOUCH_IsPressed());

                Sleep(50);
                //Redraw main window
                BSP_LCD_SelectLayer(1);
                LCD_DrawBitmap(LCD_MakePoint(0, 0), mainimg_bmp, mainimg_bmp_size);
                TEXTBOX_DrawContext(&main_ctx);
                LCD_ShowActiveLayerOnly();
                PROTOCOL_Reset();

                //Write Version Number
                FONT_Write(FONT_FRAN, LCD_WHITE, 0, 420, 9, AAVERSION_CECV);

                Voltage();  //reduce delay time for display voltage information
                DisplayVoltage();
            }   //end of if (check touch)

        }

        PROTOCOL_Handler();
    }
}
#define XXa 2

void SetColours(void){
 if(ColourSelection==1){// Daylight
        BackGrColor=LCD_WHITE;
        CurvColor=LCD_COLOR_DARKGREEN;// dark blue
        TextColor=LCD_BLACK;
        Color1=LCD_COLOR_DARKGREEN;//LCD_COLOR_DARKBLUE;
        Color2=LCD_COLOR_DARKGRAY;
        Color3=LCD_YELLOW;// -ham bands area at daylight
        Color4=LCD_MakeRGB(128,255,128);//Light green

    }
    else{// Inhouse
        BackGrColor=LCD_BLACK;
        CurvColor=LCD_COLOR_LIGHTGREEN;
        TextColor=LCD_WHITE;
        Color1=LCD_COLOR_LIGHTBLUE;
        Color2=LCD_COLOR_GRAY;
        Color3=LCD_MakeRGB(0, 0, 64);// very dark blue -ham bands area
        Color4=LCD_MakeRGB(0, 64, 0);// very dark green
    }
    CFG_SetParam(CFG_PARAM_Daylight,ColourSelection);
    CFG_Flush();
}

static void Daylight(void){
    ColourSelection=1;
    SetColours();// includes "CFG_Flush"
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 240, 10,   "  Daylight Colours   ");
}

static void Inhouse(void){
    ColourSelection=0;
    SetColours();
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 240, 10,   "  Inhouse Colours   ");
}
static void Fatlines(void){
    FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 20, 10,   "  Fat Lines  ");
    FatLines=true;
    CFG_SetParam(CFG_PARAM_Fatlines,1);
    CFG_Flush();
}


static void Thinlines(void){
    FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 20, 10,   "  Thin Lines ");
    FatLines=false;
    CFG_SetParam(CFG_PARAM_Fatlines,0);
    CFG_Flush();
}

static void BeepOn(void){
    FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 240, 50,   "  Beep On  ");
    BeepOn1=1;
    CFG_SetParam(CFG_PARAM_BeepOn,1);
    CFG_Flush();
}

static void BeepOff(void){
    FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 240, 50,   "  Beep Off ");
    BeepOn1=0;
    CFG_SetParam(CFG_PARAM_BeepOn,0);
    CFG_Flush();
}

static void Exit1(void){
    rqExit1=true;
    InitVoltage();
}

static const TEXTBOX_t tb_col[] = {
    (TEXTBOX_t){ .x0 = XXa, .y0 = 50, .text = "Daylight", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = (void(*)(void))Daylight, .cbparam = 1, .next = (void*)&tb_col[1] },
    (TEXTBOX_t){ .x0 = XXa, .y0 = 100, .text = "Inhouse", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = (void(*)(void))Inhouse, .cbparam = 1, .next = (void*)&tb_col[2] },
    (TEXTBOX_t){ .x0 = XXa, .y0 = 150, .text = "Fat Lines", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = (void(*)(void))Fatlines, .cbparam = 1, .next = (void*)&tb_col[3] },
    (TEXTBOX_t){ .x0 = XXa, .y0 = 200, .text = "Thin Lines", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = (void(*)(void))Thinlines, .cbparam = 1, .next = (void*)&tb_col[4] },
    (TEXTBOX_t){ .x0 = XXa+240, .y0 = 100, .text = "Beep On", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = (void(*)(void))BeepOn, .cbparam = 1, .next = (void*)&tb_col[5] },
    (TEXTBOX_t){ .x0 = XXa+240, .y0 = 150, .text = "Beep Off", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = (void(*)(void))BeepOff, .cbparam = 1, .next = (void*)&tb_col[6] },
    (TEXTBOX_t){ .x0 = 240, .y0 = 200, .text = "  Exit   ", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = LCD_RED, .cb = (void(*)(void))Exit1, .cbparam = 1,  },
};


static void Colours(void){

    while(TOUCH_IsPressed());
    SetColours();
    rqExit1=false;
    LCD_FillAll(LCD_COLOR_DARKGRAY);
    TEXTBOX_CTX_t fctx;
    if(ColourSelection==0)
        FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 240, 10,   "  Inhouse Colours   ");
    else
        FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 240, 10,   "  Daylight Colours   ");
    if(FatLines==true)
        FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 20, 10,   "  Fat Lines  ");
    else
        FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 20, 10,   "  Thin Lines ");
    if(BeepOn1==1)
        FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 240, 50,   "  Beep On  ");
    else
        FONT_Write(FONT_FRANBIG, LCD_BLACK, LCD_YELLOW, 240, 50,   "  Beep Off ");


    TEXTBOX_InitContext(&fctx);
    TEXTBOX_Append(&fctx, (TEXTBOX_t*)tb_col); //Append the very first element of the list in the flash.
                                                      //It is enough since all the .next fields are filled at compile time.
    TEXTBOX_DrawContext(&fctx);


    for(;;)
    {
        Sleep(0); //for autosleep to work
        if (TEXTBOX_HitTest(&fctx))
        {
            Sleep(50);
            if (rqExit1)
            {
                rqExit1=false;
                while (TOUCH_IsPressed());
                return;
            }
            Sleep(50);
        }
        Sleep(0);
    }
}


#define COL3 380

void InitVoltage(void){

    Volt_max_Display=CFG_GetParam(CFG_PARAM_Volt_max_Display);
    Volt_min_Display=CFG_GetParam(CFG_PARAM_Volt_min_Display);
    Volt_max_Factor=CFG_GetParam(CFG_PARAM_Volt_max_Factor);
    if(Volt_max_Factor==0)Volt_max_Factor=614.f;// default factor
    BattVoltage=0;
    CountMax=100;
    cntr=0;
    SetColours();
}

//KD8CEC : for Visible Voltage indicator from hidden by other screen
void DisplayVoltage()
{
    char text1[10];
    LCDColor vbg;

    //Modified by KD8CEC for users request
    //sprintf(text1,"%5.2fV %3f", VoltFloat, percent);
    //strcat(text1,"%");
    //sprintf(text1,"%5.2fV %3.0f%% ", VoltFloat, percent);
    //2019.03.26 removed % mark
    sprintf(text1,"%5.2fV %3.0f ", VoltFloat, percent);

    //vbg=BackGrColor;
    vbg = LCD_RGB(64, 64, 64);  //

    if(percent<=25){
        vbg=LCD_RED;
    }
    else if(percent<=50){
        vbg = LCD_RGB(127, 127, 0);//LCD_YELLOW
    }

    FONT_Write(FONT_FRAN, TextColor, vbg, 399, 257, text1);

}

//KD8CEC : RESIZE FONT AND CHANGE POSITION
void Voltage(void)
{
    char text1[10];
    int VoltRaw;
    if(Volt_max_Display == 0)
        return;

    cntr++;
    VoltRaw = UB_ADC3_SINGLE_Read_MW(ADC_PF9);

    if(VoltRaw <= 0){
        sprintf(text1,"Error: %d ", VoltRaw);
        FONT_Write(FONT_FRAN, TextColor, LCD_RED, 340, 230, text1);
        return;
    }
    BattVoltage+=VoltRaw;
    if(cntr>=CountMax){
        Batt=BattVoltage/CountMax;// value for calibration
        VoltFloat=(float)Batt/Volt_max_Factor;//614.f  563.0f
        cntr=0;
        CountMax=3200,
        BattVoltage=0;

        if(VoltFloat>(float)Volt_max_Display/1000.0f) {
                percent=100;
        }
        else
            //percent=(int)(VoltFloat*100.f-300.f);
            percent=(int)100.f*((1000.f*VoltFloat-(float)Volt_min_Display)/(Volt_max_Display-Volt_min_Display));

        DisplayVoltage();
    }
}


void minus10(void){
    while(TOUCH_IsPressed());
    if(VoltCase==1)
    Volt_max_Display-=10;
    else Volt_min_Display-=10;
}
void minus100(void){
    while(TOUCH_IsPressed());
    if(VoltCase==1)
    Volt_max_Display-=100;
    else Volt_min_Display-=100;
}
void minus1000(void){
    while(TOUCH_IsPressed());
    if(VoltCase==1)
    Volt_max_Display-=1000;
    else Volt_min_Display-=1000;
}

void plus10(void){
    while(TOUCH_IsPressed());
    if(VoltCase==1)
        Volt_max_Display+=10;
    else Volt_min_Display+=10;
}
void plus100(void){
    while(TOUCH_IsPressed());
    if(VoltCase==1)
        Volt_max_Display+=100;
    else Volt_min_Display+=100;
}
void plus1000(void){
    while(TOUCH_IsPressed());
    if(VoltCase==1)
        Volt_max_Display+=1000;
    else Volt_min_Display+=1000;
}

void VoltCase1(void){
    while(TOUCH_IsPressed());
    VoltCase=1;
    LCD_Rectangle(LCD_MakePoint(COL1,160),LCD_MakePoint(COL1+200,160+34),LCD_RED);
    LCD_Rectangle(LCD_MakePoint(COL2,160),LCD_MakePoint(COL2+200,160+34),LCD_YELLOW);
}
void VoltCase2(void){
    while(TOUCH_IsPressed());
    VoltCase=2;
    LCD_Rectangle(LCD_MakePoint(COL1,160),LCD_MakePoint(COL1+200,160+34),LCD_YELLOW);
    LCD_Rectangle(LCD_MakePoint(COL2,160),LCD_MakePoint(COL2+200,160+34),LCD_RED);
}

static int rqExit3;

static void Exit3(void){
    rqExit3=1;
    InitVoltage();
}

void ShowVoltage(void){
char str1[30];

    sprintf(str1,"MaxV: %.3f ", (float)Volt_max_Display/1000.f);
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 120, 10, str1);

    sprintf(str1,"MinV: %.3f ", (float)Volt_min_Display/1000.f);
    FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 120, 45, str1);//60
    sprintf(str1,"MaxFact: %d ", Volt_max_Factor);
    FONT_Write(FONT_FRAN, TextColor, BackGrColor, 120, 80, str1);

    if  (Volt_max_Display==0) LCD_Rectangle(LCD_MakePoint(COL2,210),LCD_MakePoint(COL2+240,210+34),LCD_RED);
    else   LCD_Rectangle(LCD_MakePoint(COL2,210),LCD_MakePoint(COL2+240,210+34),LCD_YELLOW);
}

void VoltOff(void){
    while(TOUCH_IsPressed());
    CFG_SetParam(CFG_PARAM_Volt_max_Display,0);
    Volt_max_Display=0;
    ShowVoltage();
    Sleep(2000);
    rqExit3=1;
}

void SetMax(void){
    while(TOUCH_IsPressed());
    Volt_max_Factor=(int)((float)Batt*1000.f/Volt_max_Display);
    CFG_SetParam(CFG_PARAM_Volt_max_Factor, Volt_max_Factor);
    ShowVoltage();
    LCD_Rectangle(LCD_MakePoint(110,110),LCD_MakePoint(330,144),LCD_RED);
    Sleep(2000);
    rqExit3=1;
}

static const TEXTBOX_t volt_menu[] = {
    (TEXTBOX_t){.x0 = COL1, .y0 = 10, .text =  " -0.01 ", .font = FONT_FRANBIG,.width = 90, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = minus10 , .cbparam = 1, .next = (void*)&volt_menu[1] },
    (TEXTBOX_t){.x0 = COL3, .y0 = 10, .text =  " +0.01 ", .font = FONT_FRANBIG,.width = 90, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = plus10 , .cbparam = 1, .next = (void*)&volt_menu[2] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 60, .text =  " -0.1 ", .font = FONT_FRANBIG,.width = 90, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = minus100 , .cbparam = 1, .next = (void*)&volt_menu[3] },
    (TEXTBOX_t){.x0 = COL3, .y0 = 60, .text =  " +0.1 ", .font = FONT_FRANBIG,.width = 90, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = plus100 , .cbparam = 1, .next = (void*)&volt_menu[4] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 110, .text =  " -1.0 ", .font = FONT_FRANBIG,.width = 90, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = minus1000 , .cbparam = 1, .next = (void*)&volt_menu[5] },
    (TEXTBOX_t){.x0 = COL3, .y0 = 110, .text =  " +1.0 ", .font = FONT_FRANBIG,.width = 90, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = plus1000 , .cbparam = 1, .next = (void*)&volt_menu[6] },
    (TEXTBOX_t){.x0 = 110, .y0 = 110, .text =    "Set Max Value", .font = FONT_FRANBIG,.width = 220, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = SetMax , .cbparam = 1, .next = (void*)&volt_menu[7] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 160, .text =    " Volt Max Displ  ", .font = FONT_FRANBIG,.width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = VoltCase1 , .cbparam = 1, .next = (void*)&volt_menu[8] },
    (TEXTBOX_t){.x0 = COL2, .y0 = 160, .text =  " Volt Min Displ ", .font = FONT_FRANBIG,.width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = VoltCase2 , .cbparam = 1, .next = (void*)&volt_menu[9] },
    (TEXTBOX_t){ .x0 = COL2, .y0 = 210, .text = "Voltage Display Off", .font = FONT_FRANBIG, .width = 240, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = VoltOff , .cbparam = 1,.next = (void*)&volt_menu[10]},
    (TEXTBOX_t){ .x0 = COL1, .y0 = 210, .text = " Main Menu ", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = LCD_RED, .cb = (void(*)(void))Exit3, .cbparam = 1,},
};



static void MenuAccu(void){
    while(TOUCH_IsPressed());
    rqExit3=0;
    VoltCase=0;
    LCD_FillAll(LCD_BLACK);
    TEXTBOX_InitContext(&menu3_ctx);
    TEXTBOX_Append(&menu3_ctx, (TEXTBOX_t*)volt_menu);
    TEXTBOX_DrawContext(&menu3_ctx);
    Volt_max_Display=CFG_GetParam(CFG_PARAM_Volt_max_Display);
    Volt_min_Display=CFG_GetParam(CFG_PARAM_Volt_min_Display);
    ShowVoltage();
 for(;;)
    {
        Sleep(0); //for autosleep to work
        if (TEXTBOX_HitTest(&menu3_ctx))
        {
            Sleep(0);
            ShowVoltage();
            if (rqExit3==1)
            {
                rqExit3=0;
                CFG_SetParam(CFG_PARAM_Volt_max_Display,Volt_max_Display);
                CFG_SetParam(CFG_PARAM_Volt_min_Display,Volt_min_Display);
                CFG_Flush();
                rqExitR=true;
                return;
            }
            Sleep(50);
        }
        Sleep(0);
    }

}

extern uint32_t NumKeypad(uint32_t initial, uint32_t min_value, uint32_t max_value, const char* header_text);
static uint32_t date2, time2;
static uint32_t rtctime;

bool TestRTCpresent(void){
unsigned char second1, secondOld;
short AMPM;

    getTime(&rtctime, &secondOld, &AMPM, 0);// time from RTC
    getDate(&date2);
    Sleep(1500);
    getTime(&rtctime, &second1, &AMPM, 0);
    getDate(&date2);

    if(second1!=secondOld){
         RTCpresent=true;
    }
    else RTCpresent=false;
    return RTCpresent;
}

static void MenuRTC(void){// Date/Time real time clock
unsigned char seconOld;
uint32_t date3,time3;
short AMPM;
char txt[20];
int k;

    while(TOUCH_IsPressed());// wait for release
    if(NoDate==1) date=20181001;
    else if(!TestRTCpresent()){
        date3=CFG_GetParam(CFG_PARAM_Date);
        time3=CFG_GetParam(CFG_PARAM_Time);

        if((date3<=19800101)||(date3>20803112)) {
            date3 = 20181001;
            time3 = 1012;
        }
        date=date3;
        time3=GetInternTime(&second);
        if(time3>2359) time3=1200;
        time=time3;
    }
    LCD_FillAll(LCD_BLACK);
    date3=NumKeypad(date,19800101,20801231,"Date") ;
    if(date3!=0) date=date3;// Cancel ??
    else {
        CFG_SetParam(CFG_PARAM_Date, date3); // no date
        CFG_Flush();// write config data into file
        rqExitR=1;// back to main menu
        return;
    }
    if(RTCpresent){
        setDate(date);
    }
    time=NumKeypad(time,0,2359,"Time") ;
    if(RTCpresent==1){
        setTime(time, 0, 0, 0);
    }
    else{
        SetInternTime(time);
    }

    for(k=0;k<2;k++){
        if(RTCpresent==1){
            getTime(&time, &second, &AMPM, 0);
            getDate(&date2);
        }
        else{
            time=GetInternTime(&second);
        }
        sprintf(txt, "Time: %04d : %02d ",time, second);
        FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 240, 80, txt);
        sprintf(txt, "Date: %08d ",date);
        FONT_Write(FONT_FRANBIG, TextColor, BackGrColor, 240, 120, txt);
        Sleep(1000);
    }

    CFG_SetParam(CFG_PARAM_Date, date);
    CFG_SetParam(CFG_PARAM_Time, time);
    CFG_Flush();// write config data into file
    rqExitR=1;// back to main menu
}

static const TEXTBOX_t tb_menu1[] = {
    (TEXTBOX_t){.x0 = COL1, .y0 = 10, .text =  " Colours/Beep ", .font = FONT_FRANBIG,.width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = Colours , .cbparam = 1, .next = (void*)&tb_menu1[1] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 60, .text =  " Configuration ", .font = FONT_FRANBIG,.width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = CFG_ParamWnd , .cbparam = 1, .next = (void*)&tb_menu1[2] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 110, .text =    " Calibration  ", .font = FONT_FRANBIG,.width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = MenuCalibration , .cbparam = 1, .next = (void*)&tb_menu1[3] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 160, .text =  " DSP ", .font = FONT_FRANBIG,.width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = FFTWND_Proc , .cbparam = 1, .next = (void*)&tb_menu1[4] },
    (TEXTBOX_t){ .x0 = COL2, .y0 = 10, .text = " Accu Settings ", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = MenuAccu , .cbparam = 1,.next = (void*)&tb_menu1[5]},
    (TEXTBOX_t){ .x0 = COL2, .y0 = 60, .text = " Date/Time ", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = MenuRTC , .cbparam = 1,.next = (void*)&tb_menu1[6]},

    (TEXTBOX_t){ .x0 = COL2, .y0 = 110, .text = "Manage SnapShot", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = Reload_Proc, .cbparam = 1,.next = (void*)&tb_menu1[7]},

    (TEXTBOX_t){ .x0 = COL2, .y0 = 160, .text = "USB HS Transfer", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = USBD_Proc , .cbparam = 1,.next = (void*)&tb_menu1[8]},

    (TEXTBOX_t){ .x0 = COL1, .y0 = 210, .text = " Main Menu ", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = LCD_RED, .cb = (void(*)(void))Exit, .cbparam = 1,},
};

static void MenuSettings(void){
    while(TOUCH_IsPressed());
    rqExit1=false;
    LCD_FillAll(LCD_BLACK);
    //Initialize textbox context
    TEXTBOX_InitContext(&menu1_ctx);
//HW calibration menu
    TEXTBOX_Append(&menu1_ctx, (TEXTBOX_t*)tb_menu1);
    TEXTBOX_DrawContext(&menu1_ctx);
    InitVoltage();
 for(;;)
    {
        Voltage();
        Sleep(0); //for autosleep to work
        if (TEXTBOX_HitTest(&menu1_ctx))
        {
            Sleep(0);
            LCD_FillAll(LCD_BLACK);
            TEXTBOX_DrawContext(&menu1_ctx);
            if (rqExitR)
            {
                rqExitR=false;
                return;
            }
            Sleep(50);
        }
        Sleep(0);
    }
}

static uint32_t  SImax, fmax, f5;
char str1[60];

uint32_t TestLoopOsci(uint32_t f_begin, uint32_t f_stop){
uint32_t fhz1,step;
unsigned char liney;
    if(f_begin>600000000) {
            step=5000000;
            liney=100;
    }
    else {
        step=1000000;
        liney=40;
    }
    for(fhz1=f_begin;fhz1<=f_stop; fhz1+=step){
        GEN_SetMeasurementFreq(fhz1);
        sprintf(str1, "Fosc : %d MHz", fhz1/1000000);
        FONT_Write(FONT_FRANBIG, CurvColor, BackGrColor, 0, liney, str1);
        Sleep(20);
        DSP_Measure2();
        //if (DSP_MeasuredMagVmv() > 1.f)
        if (DSP_MeasuredMagVmv() > 0.3f)// was 0.3
            FONT_Write(FONT_FRAN, Color1, BackGrColor, 0, 160, "Signal OK   ");
        else {
            FONT_Write(FONT_FRAN, Color1, BackGrColor, 0, 160, "No Signal ");
            break;
        }
    }
    return fhz1;
}

void hit_save1(void){
    CFG_SetParam(CFG_PARAM_SI5351_MAX_FREQ,fmax);
    CFG_SetParam(CFG_PARAM_BAND_FMAX,f5);
    CFG_Flush();
    rqExit=1;
}
void hit_ex1(void){
    CFG_SetParam(CFG_PARAM_SI5351_MAX_FREQ,SImax);
    CFG_Flush();
    rqExit=1;
}

uint32_t SI5351_MAX_FREQ[]={160000000,200000000,260000000,270000000,280000000,290000000,300000000};

TEXTBOX_t hbEx1 = (TEXTBOX_t){.x0 = 10, .y0 = 200, .text = " Cancel and exit ", .font = FONT_FRANBIG,
                            .fgcolor = LCD_BLUE, .bgcolor = LCD_YELLOW, .cb = hit_ex1 };
TEXTBOX_t hbSave1 = (TEXTBOX_t){.x0 = 300, .y0 = 200, .text = " Save and exit ", .font = FONT_FRANBIG,
                            .fgcolor = LCD_RGB(128,128,128), .bgcolor = LCD_RGB(64,64,64), .cb = hit_save1 };

void OsciTest(void){// ************************************************************************************************
uint32_t fmax1,fmax2,fmax3,k;
    rqExit=0;
    while(TOUCH_IsPressed());
    TEXTBOX_InitContext(&osci_ctx);
    TEXTBOX_Append(&osci_ctx, &hbEx1);
    TEXTBOX_Append(&osci_ctx, &hbSave1);
    LCD_FillAll(LCD_BLACK);
    TEXTBOX_DrawContext(&osci_ctx);
    GEN_Init();
    SImax=CFG_GetParam(CFG_PARAM_SI5351_MAX_FREQ);
    CFG_SetParam(CFG_PARAM_SI5351_MAX_FREQ,300000000);
    fmax1=TestLoopOsci(200000000, 300000000);
    Sleep(2000);
    fmax2=TestLoopOsci((fmax1*9)/10, 300000000);
    Sleep(2000);
    fmax3=TestLoopOsci((fmax2*9)/10, 300000000);
    Sleep(2000);
    if(fmax2>fmax3) fmax=fmax3;//
    else fmax=fmax2;
    if(fmax1<fmax) fmax=fmax1;
    fmax-=2000000;
    if(fmax>290000000) fmax=290000000;
    fmax1=TestLoopOsci((fmax/2)*9, 5*fmax);
    Sleep(2000);
    fmax2=TestLoopOsci((fmax/2)*9, 5*fmax);
    Sleep(2000);
    fmax3=TestLoopOsci((fmax/2)*9, 5*fmax);
    Sleep(2000);
    if(fmax1<fmax2) f5=fmax1; else f5=fmax2;//calculate the minimum
    if(fmax3<f5) f5=fmax3;
    GEN_SetMeasurementFreq(0);// oscillator off
    if(5*fmax>f5)fmax=f5/5;
    //else f5=5*fmax;
    for(k=1;k<6;k++){// search for highest value in list, take the lower neighbor
        if(fmax<SI5351_MAX_FREQ[k]){
            fmax=SI5351_MAX_FREQ[k-1];
            break;
        }
    }
    f5=5*fmax;
    sprintf(str1, "FMAX(S15351) %d MHz, MAX_FREQ %d MHz", fmax/1000000, f5/1000000);
    FONT_Write(FONT_FRAN, CurvColor, BackGrColor, 0, 140, str1);
    Sleep(1000);

    for(;;) {
        if (TEXTBOX_HitTest(&osci_ctx))
        {
            if (rqExit)
            {
                return;
            }
            Sleep(50);
        }
        Sleep(0);
    }
}



static const TEXTBOX_t tb_menu2[] = {
    (TEXTBOX_t){.x0 = COL1, .y0 =20, .text =  " OSL Calibration, use calibration kit !!! ", .font = FONT_FRANBIG,.width = 440, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = OSL_CalWnd , .cbparam = 1, .next = (void*)&tb_menu2[1] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 70, .text =    " HW Calibration, only at first run !!!  ", .font = FONT_FRANBIG,.width = 440, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = OSL_CalErrCorr , .cbparam = 1, .next = (void*)&tb_menu2[2] },
    (TEXTBOX_t){.x0 = COL1, .y0 = 120, .text =   "|S21] Gain Calibration for VNA ", .font = FONT_FRANBIG,.width = 440, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = OSL_CalTXCorr , .cbparam = 1, .next = (void*)&tb_menu2[3] },
    (TEXTBOX_t){.x0 = COL1, .y0 =170, .text =  " Oscillator Test (Fmax) ", .font = FONT_FRANBIG,.width = 440, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = M_BGCOLOR, .cb = OsciTest , .cbparam = 1, .next = (void*)&tb_menu2[4] },
    (TEXTBOX_t){ .x0 = COL1, .y0 = 210, .text = " Main Menu ", .font = FONT_FRANBIG, .width = 200, .height = 34, .center = 1,
                 .border = 1, .fgcolor = M_FGCOLOR, .bgcolor = LCD_RED, .cb = (void(*)(void))Exit, .cbparam = 1,},
};

static void MenuCalibration(void){
    while(TOUCH_IsPressed());
    rqExit1=false;
    LCD_FillAll(LCD_BLACK);
    //Initialize textbox context
    TEXTBOX_InitContext(&menu2_ctx);
//HW calibration menu
    TEXTBOX_Append(&menu2_ctx, (TEXTBOX_t*)tb_menu2);
    TEXTBOX_DrawContext(&menu2_ctx);
    InitVoltage();
 for(;;)
    {
        Sleep(0); //for autosleep to work
        if (TEXTBOX_HitTest(&menu2_ctx))
        {
            Voltage();
            Sleep(0);
            LCD_FillAll(LCD_BLACK);
            TEXTBOX_DrawContext(&menu2_ctx);
            if (rqExitR)
            {
                rqExitR=false;
                return;
            }
            Sleep(50);
        }
        Sleep(0);
    }
}
