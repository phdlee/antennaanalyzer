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


#ifndef SI5351_HS_H_
#define SI5351_HS_H_

typedef uint8_t byte;

#define HS_PLLA				0
#define HS_PLLB				1

#define HS_CLK0				0
#define HS_CLK1				1
#define HS_CLK2				2
#define HS_CLK3				3
#define HS_CLK4				4
#define HS_CLK5				5
#define HS_CLK6				6
#define HS_CLK7				7

//D7 D6 D5 D4 D3 D2 D1 D0
//CLK0_PDN MS0_INT MS0_SRC CLK0_INV CLK0_SRC[1:0] CLK0_IDRV[1:0]
//HS is Just prefix for prevent redifine constant value with other si5351 source and header file
#define HS_CLK_UP 		0
#define HS_CLK_DOWN 	1

#define HS_FRACTIONAL 0
#define HS_INTEGER		1

#define HS_CLKNOTINVT	0
#define HS_CLKINVT		1

#define HS_2MA				0
#define HS_4MA				1
#define HS_6MA				2
#define HS_8MA				3

#define HS_6PF			  1
#define HS_8PF			  2
#define HS_10PF			  3

#define HS_MAX_C_VAL 	1048574	//Max value for C (20 bits) cPartValue is below <= 1048574 (20bit)

#define CLK_CONTROL_DEFAULT_VALUE (HS_CLK_DOWN << 7) | (HS_INTEGER << 6) | (HS_PLLA << 5) | (HS_CLKNOTINVT << 4) | (3 << 2) | (HS_2MA)

#define HS_TRUE 		  1
#define HS_FALSE		  0

//================================================
//functions
//------------------------------------------------
//Init
void HS_SI5351_Init(unsigned hs_xtal_freq, uint8_t hs_xtal_pf);

//-------------------------------------------------
//Clock Port Control (CLK0 .. CLK7)
void HS_SetClockUp(uint8_t clkNum, uint8_t isEnabled, uint8_t isApply);
void HS_AllClockDown();
void HS_AllClockUp();
void HS_SetClockEnabled(uint8_t clkNum, uint8_t isEnabled, uint8_t isApply);
void HS_SetPower(uint8_t clkNum, uint8_t txPower, uint8_t isApply);
void HS_SetPLL(uint8_t clkNum, uint8_t usePLLIndex, uint8_t isApply);	///don't use, use HS_Bind...

//-------------------------------------------------
//PLL Configuration (PLLA, PLLB)
void HS_CalcPLLParam(unsigned long cPartValue,unsigned long targetFrequency, unsigned long *rst_P1, unsigned long *rst_P2, unsigned long *rst_P3);
void HS_ApplyPLLParam(uint8_t usePLL, unsigned long MSN_P1, unsigned long MSN_P2, unsigned long MSN_P3);
void HS_BindCLKToPLL(uint8_t clkNum, uint8_t usePLL, unsigned long targetFrequency);

//-------------------------------------------------
//High Speed Frequency Control by P1, P2
void HS_SetPLLP2(uint8_t usePLL, unsigned long P2, unsigned long P3);	//Shift PLLA Frequency by P2
void HS_SetPLLP1P2(uint8_t usePLL, unsigned long P1, unsigned long P2, unsigned long P3);	//Shift PLLA Frequency by P2

//Utils
uint8_t HS_MAToParam(uint8_t txPowerMa);
#endif
