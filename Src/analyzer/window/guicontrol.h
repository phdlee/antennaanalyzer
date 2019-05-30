/*
 *   KD8CEC
 *   kd8cec@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef _GUICONTROL_H_
#define _GUICONTROL_H_

#include <ctype.h>
#include "main.h"
#include "bitmaps/bitmaps.h"

//================================================================================================
//KD8CEC
//================================================================================================
//Check Touch Area
#define LINE1_TOP      49
#define LINE1_BOTTOM   105
#define LINE2_TOP      125
#define LINE2_BOTTOM   180
#define LINE3_TOP      204
#define LINE3_BOTTOM   260

#define COL1_LEFT      13
#define COL1_RIGHT     93
#define COL2_LEFT      105
#define COL2_RIGHT     187
#define COL3_LEFT      198
#define COL3_RIGHT     280
#define COL4_LEFT      291
#define COL4_RIGHT     371
#define COL5_LEFT      384
#define COL5_RIGHT     464


#define LINE1_BOTTOM   105
#define LINE2_TOP      125
#define LINE2_BOTTOM   180
#define LINE3_TOP      204
#define LINE3_BOTTOM   260

//Count of Buttons in MainMenu
#define BUTTON_COUNT   15

//LINE 1
#define BTN_SINGLE      0   //SINGLE FREQUENCY
#define BTN_SWEEP       1   //FREQUENCY SWEEP
#define BTN_MULTISWR    2   //MULTI SWR
#define BTN_TUNESWR     3   //TUNE SWR
#define BTN_TRACKER     4   //S21 Gain

//LINE 2
#define BTN_FIND        5   //FIND FREQUENCY
#define BTN_QUARTZ      6   //QUARTZ DATA
#define BTN_TDR         7   //TDR
#define BTN_LC          8   //LC METER
#define BTN_DSP         9   //DSP Test


//LINE 3
#define BTN_CONFIG      10
#define BTN_SNAPSHOT    11   //MANAGE SNAPSHOT
#define BTN_USB         12   //USB READER
#define BTN_RFGEN       13   //RF GENERATOR
#define BTN_WSPR        14   //Weak Signal Protocol

//RESERVE
#define BTN_REBOOT      99

const int buttonsArea [BUTTON_COUNT][4] = {{COL1_LEFT, LINE1_TOP, COL1_RIGHT, LINE1_BOTTOM}, {COL2_LEFT, LINE1_TOP, COL2_RIGHT, LINE1_BOTTOM}, {COL3_LEFT, LINE1_TOP, COL3_RIGHT, LINE1_BOTTOM}, {COL4_LEFT, LINE1_TOP, COL4_RIGHT, LINE1_BOTTOM}, {COL5_LEFT, LINE1_TOP, COL5_RIGHT, LINE1_BOTTOM},
{COL1_LEFT, LINE2_TOP, COL1_RIGHT, LINE2_BOTTOM}, {COL2_LEFT, LINE2_TOP, COL2_RIGHT, LINE2_BOTTOM}, {COL3_LEFT, LINE2_TOP, COL3_RIGHT, LINE2_BOTTOM}, {COL4_LEFT, LINE2_TOP, COL4_RIGHT, LINE2_BOTTOM}, {COL5_LEFT, LINE2_TOP, COL5_RIGHT, LINE2_BOTTOM},
{COL1_LEFT, LINE3_TOP, COL1_RIGHT, LINE3_BOTTOM}, {COL2_LEFT, LINE3_TOP, COL2_RIGHT, LINE3_BOTTOM}, {COL3_LEFT, LINE3_TOP, COL3_RIGHT, LINE3_BOTTOM}, {COL4_LEFT, LINE3_TOP, COL4_RIGHT, LINE3_BOTTOM}, {COL5_LEFT, LINE3_TOP, COL5_RIGHT, LINE3_BOTTOM},
};




#endif
