/*
 * JTEncode.h - JT65/JT9/WSPR/FSQ encoder library for Arduino
 *
 * Copyright (C) 2015-2018 Jason Milldrum <milldrum@gmail.com>
 *
 * Based on the algorithms presented in the WSJT software suite.
 * Thanks to Andy Talbot G4JNT for the whitepaper on the WSPR encoding
 * process that helped me to understand all of this.
 *
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

#ifndef JTENCODE_H
#define JTENCODE_H

#include "int.h"
#include "rs_common.h"

//#include "Arduino.h"

#include <stdint.h>

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#include <avr/pgmspace.h>
#endif

#define JT65_SYMBOL_COUNT                   126
#define JT9_SYMBOL_COUNT                    85
#define JT4_SYMBOL_COUNT                    207
#define WSPR_SYMBOL_COUNT                   162
#define FT8_SYMBOL_COUNT                    79

#define JT65_ENCODE_COUNT                   63
#define JT9_ENCODE_COUNT                    69
#define FT8_ENCODE_COUNT	                77

#define JT9_BIT_COUNT                       206
#define JT4_BIT_COUNT                       206
#define WSPR_BIT_COUNT                      162
#define FT8_BIT_COUNT		                174

// Define the structure of a varicode table
typedef struct fsq_varicode
{
    uint8_t ch;
    uint8_t var[2];
} Varicode;

// The FSQ varicode table, based on the FSQ Varicode V3.0
// document provided by Murray Greenman, ZL1BPU


//public:
//  JTEncode(void);
  void jt65_encode(const char *, uint8_t *);
  void jt9_encode(const char *, uint8_t *);
  void jt4_encode(const char *, uint8_t *);
  void wspr_encode(const char *, const char *, const uint8_t, uint8_t *);
  void fsq_encode(const char *, const char *, uint8_t *);
  void fsq_dir_encode(const char *, const char *, const char, const char *, uint8_t *);
  void ft8_encode_msg(const char *, uint8_t *);	//from ft8_encode -> renamed ft8_encode_msg
  void ft4_encode_msg(const char *, uint8_t *); //Added by KD8CEC

//private:
  uint8_t jt_code(char);
  uint8_t ft_code(char);
  uint8_t wspr_code(char);
  uint8_t gray_code(uint8_t);
  int8_t hex2int(char);
  void jt_message_prep(char *);
  void ft_message_prep(char *);
  void wspr_message_prep(char *, char *, uint8_t);
  void jt65_bit_packing(char *, uint8_t *);
  void jt9_bit_packing(char *, uint8_t *);
  void wspr_bit_packing(uint8_t *);
  void ft8_bit_packing(char*, uint8_t*);
  void jt65_interleave(uint8_t *);
  void jt9_interleave(uint8_t *);
  void wspr_interleave(uint8_t *);
  void jt9_packbits(uint8_t *, uint8_t *);
  void jt_gray_code(uint8_t *, uint8_t);
  void ft8_encode(uint8_t* codeword, uint8_t* symbols);
  void jt65_merge_sync_vector(uint8_t *, uint8_t *);
  void jt9_merge_sync_vector(uint8_t *, uint8_t *);
  void jt4_merge_sync_vector(uint8_t *, uint8_t *);
  void wspr_merge_sync_vector(uint8_t *, uint8_t *);
  void ft8_merge_sync_vector(uint8_t*, uint8_t*);
  void convolve(uint8_t *, uint8_t *, uint8_t, uint8_t);
  void rs_encode(uint8_t *, uint8_t *);
  void encode_rs_int(void *,data_t *, data_t *);
  void free_rs_int(void *);
  void * init_rs_int(int, int, int, int, int, int);
  uint8_t crc8(const char *);
  //void * rs_inst;
  //char callsign[7];
  //char locator[5];
  //uint8_t power;

#endif
