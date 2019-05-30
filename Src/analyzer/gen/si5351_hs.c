/*
 * si5351_hs.h - Si5351 Library for more performance
 *
 * This code controls only one CLOCK per PLL. SI5351a has two PLLs,
 * so you can control only two clocks at the same time. Therefore, it must be
 *  used with other Si5351 control sources.
 *
 * Ian KD8CEC <kd8cec@gmail.com>
 *
 *---------------------------------------------------------------------------
 * I refer to the source code of OE1SEG and the source code of uBITX.
 * In particular, fixing the CLK parameter and controlling the frequency
 * with the PLL parameter used the source of the OE1SEG, which is quite useful.
 *
 * I do not claim any rights in the source code that I write. However, you must
 * comply with the license for the codes I refer to listed below.
 *
 * Reference and using source code
 * OE1SEG Si5351 Control code
 * si5351.ino in uBITX V3 Source code by Jerry
 * wspr.cpp in uBITX v# source code by KD8CEC
 *
 *---------------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "si5351_hs.h"

#include "config.h"

//Ext I2C port used for camera is wired to Arduino UNO connector when SB4 and SB1 jumpers are set instead of SB5 and SB3.
extern void     CAMERA_IO_Init(void);
extern void     CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value);
extern uint8_t  CAMERA_IO_Read(uint8_t addr, uint8_t reg);
extern void     CAMERA_Delay(uint32_t delay);
extern void     CAMERA_IO_WriteBulk(uint8_t addr, uint8_t reg, uint8_t* values, uint16_t nvalues);
extern void Sleep(uint32_t);

//=================================================
//User Implenetion function
//You must create two I2C-related functions for your device.
//The two below are examples of that.
//-------------------------------------------------
extern uint8_t SI5351_HS_Write(uint8_t addr, uint8_t data);
extern uint8_t SI5351_HS_WriteN(uint8_t addr, uint8_t bytes, uint8_t *data);
uint8_t SI5351_HS_Write(uint8_t addr, uint8_t data)
{
	//CAMERA_IO_Write(0xC0, addr, data);
	CAMERA_IO_Write(CFG_GetParam(CFG_PARAM_SI5351_BUS_BASE_ADDR), addr, data);
	return 0;
}

uint8_t SI5351_HS_WriteN(uint8_t addr, uint8_t bytes, uint8_t *data)
{
	//CAMERA_IO_WriteN(0xC0, addr, data, bytes);
    CAMERA_IO_WriteBulk(CFG_GetParam(CFG_PARAM_SI5351_BUS_BASE_ADDR), addr, data, (uint16_t)bytes);

	return 0;
}

/*
uint8_t SI5351_HS_Read(uint8_t addr, uint8_t *data)
{
	*data = CAMERA_IO_Read(0xC0, addr);
	return 0;
}
*/
//==================================================
//Registor Mirror
//--------------------------------------------------
//D7 D6 D5 D4 D3 D2 D1 D0
//CLK7_OEB CLK6_OEB CLK5_OEB CLK4_OEB CLK3_OEB CLK2_OEB CLK1_OEB CLK0_OEB
uint8_t reg_ClkEnabled = 0xFF;        // Registor 3 (0:Enabled, 1:Disabled)
uint8_t reg_ClkControl[7] = {CLK_CONTROL_DEFAULT_VALUE, CLK_CONTROL_DEFAULT_VALUE, CLK_CONTROL_DEFAULT_VALUE, CLK_CONTROL_DEFAULT_VALUE,
														 CLK_CONTROL_DEFAULT_VALUE, CLK_CONTROL_DEFAULT_VALUE, CLK_CONTROL_DEFAULT_VALUE};

//for I2C Send buffer
uint8_t PllAClockValues[8] = {0};

//=================================================
//Init
//-------------------------------------------------
unsigned long _hs_xtal_freq = 27000000;
//uint8_t HS_XTAL_PF = HS_6PF;	//not need stored

void HS_SI5351_Init(unsigned hs_xtal_freq, uint8_t hs_xtal_pf)
{
	_hs_xtal_freq= hs_xtal_freq;
	//HS_XTAL_PF = hs_xtal_pf;	//not need stored

  SI5351_HS_Write(149, 0);                     // SpreadSpectrum off
  SI5351_HS_Write(3, reg_ClkEnabled);          // Disable all CLK output drivers
  SI5351_HS_Write(183, hs_xtal_pf << 6);  // Set 25mhz crystal load capacitance
}

//=================================================
//Clock Port Control (CLK0 .. CLK7)
//-------------------------------------------------
void HS_SetClockUp(uint8_t clkNum, uint8_t isEnabled, uint8_t isApply)
{
	//AN619 (Ref Manual). Page 17
	if (isEnabled)
		reg_ClkControl[clkNum] &= ~(HS_CLK_DOWN << 7);
	else
		reg_ClkControl[clkNum] |= (HS_CLK_DOWN << 7);

	if (isApply)
	{
		SI5351_HS_Write(16 + clkNum, reg_ClkControl[clkNum]);
	}
}

void HS_AllClockDown()
{
	for (int i = 0; i < 7; i++)
	{
		reg_ClkControl[i] |= (HS_CLK_DOWN << 7);
		SI5351_HS_Write(16 + i, reg_ClkControl[i]);             	// Disable output during the following register settings
	}
}
void HS_AllClockUp()
{
	for (int i = 0; i < 7; i++)
	{
		reg_ClkControl[i] &= ~(HS_CLK_DOWN << 7);
		SI5351_HS_Write(16 + i, reg_ClkControl[i]);             	// Disable output during the following register settings
	}
}


void HS_SetClockEnabled(uint8_t clkNum, uint8_t isEnabled, uint8_t isApply)
{
	//AN619 (Ref Manual). Page 19
//CLK7_OEB CLK6_OEB CLK5_OEB CLK4_OEB CLK3_OEB CLK2_OEB CLK1_OEB CLK0_OEB
//uint8_t reg_ClkEnabled = 0xFF;        // Registor 3 (0:Enabled, 1:Disabled)

	if (isEnabled)
		reg_ClkEnabled &= ~(1 << clkNum);
	else
		reg_ClkEnabled |= (1 << clkNum);

	if (isApply)
	{
		SI5351_HS_Write(3, reg_ClkEnabled);
	}
}

void HS_SetPower(uint8_t clkNum, uint8_t txPower, uint8_t isApply)
{
	reg_ClkControl[clkNum] &= ~(3);
	reg_ClkControl[clkNum] |= (txPower);

	if (isApply)
	{
		SI5351_HS_Write(16 + clkNum, reg_ClkControl[clkNum]);
	}
}

/*
#define HS_2MA				0
#define HS_4MA				1
#define HS_6MA				2
#define HS_8MA				3
*/
uint8_t HS_MAToParam(uint8_t txPowerMa)
{
    if (txPowerMa == 2)
        return HS_2MA;
    else if (txPowerMa == 4)
        return HS_4MA;
    else if (txPowerMa == 6)
        return HS_6MA;
    else
        return HS_8MA;
}

void HS_SetPLL(uint8_t clkNum, uint8_t usePLLIndex, uint8_t isApply)	///PLLA : 0, PLLB: 1
{
	if (usePLLIndex == 0)
		reg_ClkControl[clkNum] &= ~(1 << 5);
	else
		reg_ClkControl[clkNum] |= (1 << 5);

	if (isApply)
	{
		SI5351_HS_Write(16 + clkNum, reg_ClkControl[clkNum]);
	}
}


//=================================================
//PLL Configuration (PLLA, PLLB)
//-------------------------------------------------
//refernce source code is OE1SEG,
//Get Parameter for PLL by Frequency
void HS_CalcPLLParam(unsigned long cPartValue,unsigned long targetFrequency, unsigned long *rst_P1, unsigned long *rst_P2, unsigned long *rst_P3)
{
  unsigned long TX_fvco;                       // VCO frequency (600-900 MHz) of PLL
  unsigned long TX_outdivider;                 // Output divider in range [4,6,8-900], even numbers preferred
  byte TX_R = 1;                               // Additional Output Divider in range [1,2,4,...128]
  byte TX_a;                                   // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  unsigned long TX_b;                          // "b" part of Feedback-Multiplier from XTAL to PLL
  float TX_f;                                  // floating variable, needed in calculation
  unsigned long MS0_P1;                        // Si5351a Output Divider register MS0_P1, P2 and P3 are hardcoded below

  TX_outdivider = 900000000 / targetFrequency;    // With 900 MHz beeing the maximum internal PLL-Frequency

  while (TX_outdivider > 900){                    // If output divider out of range (>900) use additional Output divider
    TX_R = TX_R * 2;
    TX_outdivider = TX_outdivider / 2;
  }
  if (TX_outdivider % 2) TX_outdivider--;         // finds the even divider which delivers the intended Frequency

  TX_fvco = TX_outdivider * TX_R * targetFrequency;  // Calculate the PLL-Frequency (given the even divider)

  switch (TX_R){                          // Convert the Output Divider to the bit-setting required in register 44
    case 1: TX_R = 0; break;              // Bits [6:4] = 000
    case 2: TX_R = 16; break;             // Bits [6:4] = 001
    case 4: TX_R = 32; break;             // Bits [6:4] = 010
    case 8: TX_R = 48; break;             // Bits [6:4] = 011
    case 16: TX_R = 64; break;            // Bits [6:4] = 100
    case 32: TX_R = 80; break;            // Bits [6:4] = 101
    case 64: TX_R = 96; break;            // Bits [6:4] = 110
    case 128: TX_R = 112; break;          // Bits [6:4] = 111
  }

  TX_a = TX_fvco / _hs_xtal_freq;		  				// Multiplier to get from Quartz-Oscillator Freq. to PLL-Freq.
  TX_f = TX_fvco - TX_a * _hs_xtal_freq;       // Multiplier = a+b/c
  TX_f = TX_f * cPartValue;               // this is just "int" and "float" mathematics
  TX_f = TX_f / _hs_xtal_freq;
  TX_b = TX_f;

  MS0_P1 = 128 * TX_outdivider - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                          // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below

  TX_f = 128 * TX_b / cPartValue;         // Calculation of Feedback Multisynth registers MSNB_P1 to MSNB_P3
  *rst_P1 = 128 * TX_a + TX_f - 512;
  *rst_P2 = TX_f;
  *rst_P2 = 128 * TX_b - (*rst_P2) * cPartValue;
  *rst_P3 = cPartValue;
}

//Same source as above CalPLLParam.
void HS_BindCLKToPLL(uint8_t clkNum, uint8_t usePLL, unsigned long targetFrequency)
{
  unsigned long outdivider;
  byte TX_R = 1;
  unsigned long MS0_P1;

  outdivider = 900000000 / targetFrequency;

  while (outdivider > 900)
	{
    TX_R = TX_R * 2;
    outdivider = outdivider / 2;
  }
  if (outdivider % 2) outdivider--;


  switch (TX_R){                          // Convert the Output Divider to the bit-setting required in register 44
    case 1: TX_R = 0; break;              // Bits [6:4] = 000
    case 2: TX_R = 16; break;             // Bits [6:4] = 001
    case 4: TX_R = 32; break;             // Bits [6:4] = 010
    case 8: TX_R = 48; break;             // Bits [6:4] = 011
    case 16: TX_R = 64; break;            // Bits [6:4] = 100
    case 32: TX_R = 80; break;            // Bits [6:4] = 101
    case 64: TX_R = 96; break;            // Bits [6:4] = 110
    case 128: TX_R = 112; break;          // Bits [6:4] = 111
  }

  MS0_P1 = 128 * outdivider - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                          // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below
	//Set PLLA
	HS_SetPLL(clkNum, usePLL, HS_FALSE);

	//Set ClockOut
	uint8_t ClkSetVals[8];
	ClkSetVals[0] = 0;
	ClkSetVals[1] = 1;
	if (outdivider == 4)
	{
		ClkSetVals[2] = (12 | TX_R);
		ClkSetVals[3] = 0;
		ClkSetVals[4] = 0;
	}
	else
	{
		ClkSetVals[2] = ((MS0_P1 & 196608) >> 16) | TX_R;
		ClkSetVals[3] = (MS0_P1 & 65280) >> 8;
		ClkSetVals[4] = MS0_P1 & 255;
	}
	ClkSetVals[5] = 0;
	ClkSetVals[6] = 0;
	ClkSetVals[7] = 0;

	SI5351_HS_WriteN(42 + (8 * clkNum), 8, ClkSetVals);
}

//Get Parameter for PLL by Frequency
void HS_ApplyPLLParam(uint8_t usePLL, unsigned long MSN_P1, unsigned long MSN_P2, unsigned long MSN_P3)
{
	PllAClockValues[0] = (MSN_P3 & 65280) >> 8;   // Bits [15:8] of MSNA_P3 in register 26
	PllAClockValues[1] = MSN_P3 & 255;            // Bits [7:0]  of MSNA_P3 in register 27
	PllAClockValues[2] = (MSN_P1 & 196608) >> 16; // Bits [17:16] of MSNA_P1 in bits [1:0] of register 28
	PllAClockValues[3] = (MSN_P1 & 65280) >> 8;   // Bits [15:8]  of MSNA_P1 in register 29
	PllAClockValues[4] = MSN_P1 & 255;            // Bits [7:0]  of MSNA_P1 in register 30
	PllAClockValues[5] = ((MSN_P3 & 983040) >> 12) | ((MSN_P2 & 983040) >> 16); // Parts of MSNA_P3 und MSNA_P1
	PllAClockValues[6] = (MSN_P2 & 65280) >> 8;   // Bits [15:8]  of MSNA_P2 in register 32
	PllAClockValues[7] = MSN_P2 & 255;            // Bits [7:0]  of MSNA_P2 in register 33

	SI5351_HS_WriteN(usePLL == HS_PLLA ? 26 : 34, 8, PllAClockValues);

	SI5351_HS_Write(177, 32);                      // This resets PLL A
}

//High Speed Controlled by P2, Small Range (size of P3) because using just P2
void HS_SetPLLP2(uint8_t usePLL, unsigned long P2, unsigned long P3)	//Shift PLLA Frequency by P2
{
	PllAClockValues[0] = ((P3 & 983040) >> 12) | ((P2 & 983040) >> 16); //P3 is zero, P2 MSB 3bit
	PllAClockValues[1] = (P2 & 65280) >> 8;   													//P2[15:8]
	PllAClockValues[2] = P2 & 255;            													//P2[7:0]

	SI5351_HS_WriteN(usePLL == HS_PLLA ? 31 : 39, 3, PllAClockValues);
}

//Full Range Controlled (speed = HS_SetPLLP2 * 1.8) using P1, P2
void HS_SetPLLP1P2(uint8_t usePLL, unsigned long P1, unsigned long P2, unsigned long P3)	//Shift PLLA Frequency by P2
{
	PllAClockValues[0] = (P1 & 196608) >> 16; // Bits [17:16] of MSNA_P1 in bits [1:0] of register 28
	PllAClockValues[1] = (P1 & 65280) >> 8;   // Bits [15:8]  of MSNA_P1 in register 29
	PllAClockValues[2] = P1 & 255;            // Bits [7:0]  of MSNA_P1 in register 30
	PllAClockValues[3] = ((P3 & 983040) >> 12) | ((P2 & 983040) >> 16); // Parts of MSNA_P3 und MSNA_P1
	PllAClockValues[4] = (P2 & 65280) >> 8;   // Bits [15:8]  of MSNA_P2 in register 32
	PllAClockValues[5] = P2 & 255;            // Bits [7:0]  of MSNA_P2 in register 33

	SI5351_HS_WriteN(usePLL == HS_PLLA ? 28 : 36, 6, PllAClockValues);
}
