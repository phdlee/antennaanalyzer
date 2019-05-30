/*
 * JTEncode.cpp - JT65/JT9/WSPR/FSQ encoder library for Arduino
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

#include "JTEncode.h"
#include "crc14.h"
#include "generator.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "config.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#include <avr/pgmspace.h>
#endif

//Added by KD8CEC, Convrted to C language
//From JTEncode.h
void * rs_inst;
uint8_t power;
char callsign[7];
char locator[5];

const Varicode fsq_code_table[] =
{
  {' ', {00, 00}}, // space
  {'!', {11, 30}},
  {'"', {12, 30}},
  {'#', {13, 30}},
  {'$', {14, 30}},
  {'%', {15, 30}},
  {'&', {16, 30}},
  {'\'', {17, 30}},
  {'(', {18, 30}},
  {')', {19, 30}},
  {'*', {20, 30}},
  {'+', {21, 30}},
  {',', {27, 29}},
  {'-', {22, 30}},
  {'.', {27, 00}},
  {'/', {23, 30}},
  {'0', {10, 30}},
  {'1', {01, 30}},
  {'2', {02, 30}},
  {'3', {03, 30}},
  {'4', {04, 30}},
  {'5', {05, 30}},
  {'6', {06, 30}},
  {'7', {07, 30}},
  {'8', {8, 30}},
  {'9', {9, 30}},
  {':', {24, 30}},
  {';', {25, 30}},
  {'<', {26, 30}},
  {'=', {00, 31}},
  {'>', {27, 30}},
  {'?', {28, 29}},
  {'@', {00, 29}},
  {'A', {01, 29}},
  {'B', {02, 29}},
  {'C', {03, 29}},
  {'D', {04, 29}},
  {'E', {05, 29}},
  {'F', {06, 29}},
  {'G', {07, 29}},
  {'H', {8, 29}},
  {'I', {9, 29}},
  {'J', {10, 29}},
  {'K', {11, 29}},
  {'L', {12, 29}},
  {'M', {13, 29}},
  {'N', {14, 29}},
  {'O', {15, 29}},
  {'P', {16, 29}},
  {'Q', {17, 29}},
  {'R', {18, 29}},
  {'S', {19, 29}},
  {'T', {20, 29}},
  {'U', {21, 29}},
  {'V', {22, 29}},
  {'W', {23, 29}},
  {'X', {24, 29}},
  {'Y', {25, 29}},
  {'Z', {26, 29}},
  {'[', {01, 31}},
  {'\\', {02, 31}},
  {']', {03, 31}},
  {'^', {04, 31}},
  {'_', {05, 31}},
  {'`', {9, 31}},
  {'a', {01, 00}},
  {'b', {02, 00}},
  {'c', {03, 00}},
  {'d', {04, 00}},
  {'e', {05, 00}},
  {'f', {06, 00}},
  {'g', {07, 00}},
  {'h', {8, 00}},
  {'i', {9, 00}},
  {'j', {10, 00}},
  {'k', {11, 00}},
  {'l', {12, 00}},
  {'m', {13, 00}},
  {'n', {14, 00}},
  {'o', {15, 00}},
  {'p', {16, 00}},
  {'q', {17, 00}},
  {'r', {18, 00}},
  {'s', {19, 00}},
  {'t', {20, 00}},
  {'u', {21, 00}},
  {'v', {22, 00}},
  {'w', {23, 00}},
  {'x', {24, 00}},
  {'y', {25, 00}},
  {'z', {26, 00}},
  {'{', {06, 31}},
  {'|', {07, 31}},
  {'}', {8, 31}},
  {'~', {00, 30}},
  {127, {28, 31}}, // DEL
  {13,  {28, 00}}, // CR
  {10,  {28, 00}}, // LF
  {0,   {28, 30}}, // IDLE
  {241, {10, 31}}, // plus/minus
  {246, {11, 31}}, // division sign
  {248, {12, 31}}, // degrees sign
  {158, {13, 31}}, // multiply sign
  {156, {14, 31}}, // pound sterling sign
  {8,   {27, 31}}  // BS
};

const uint8_t crc8_table[] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};

const uint8_t jt9i[JT9_BIT_COUNT] = {
  0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0x10, 0x90, 0x50, 0x30, 0xb0, 0x70,
  0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0x18, 0x98, 0x58, 0x38, 0xb8, 0x78,
  0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0x14, 0x94, 0x54, 0x34, 0xb4, 0x74,
  0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0x1c, 0x9c, 0x5c, 0x3c, 0xbc, 0x7c,
  0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0x12, 0x92, 0x52, 0x32, 0xb2, 0x72,
  0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0x1a, 0x9a, 0x5a, 0x3a, 0xba, 0x7a,
  0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0x16, 0x96, 0x56, 0x36, 0xb6, 0x76,
  0x0e, 0x8e, 0x4e, 0x2e, 0xae, 0x6e, 0x1e, 0x9e, 0x5e, 0x3e, 0xbe, 0x7e, 0x01,
  0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0x11, 0x91, 0x51, 0x31, 0xb1, 0x71, 0x09,
  0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0x19, 0x99, 0x59, 0x39, 0xb9, 0x79, 0x05,
  0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0x15, 0x95, 0x55, 0x35, 0xb5, 0x75, 0x0d,
  0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0x1d, 0x9d, 0x5d, 0x3d, 0xbd, 0x7d, 0x03,
  0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0x13, 0x93, 0x53, 0x33, 0xb3, 0x73, 0x0b,
  0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0x1b, 0x9b, 0x5b, 0x3b, 0xbb, 0x7b, 0x07,
  0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0x17, 0x97, 0x57, 0x37, 0xb7, 0x77, 0x0f,
  0x8f, 0x4f, 0x2f, 0xaf, 0x6f, 0x1f, 0x9f, 0x5f, 0x3f, 0xbf, 0x7f
};




//#include "Arduino.h"

// Define an upper bound on the number of glyphs.  Defining it this
// way allows adding characters without having to update a hard-coded
// upper bound.
#define NGLYPHS         (sizeof(fsq_code_table)/sizeof(fsq_code_table[0]))

/* Public Class Members */

//Class Creator
void JTEncode(void)
{
  // Initialize the Reed-Solomon encoder
  rs_inst = (struct rs *)(intptr_t)init_rs_int(6, 0x43, 3, 1, 51, 0);
}

/*
 * jt65_encode(const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 * a channel symbol table.
 *
 * message - Plaintext Type 6 message.
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least size JT65_SYMBOL_COUNT to the method.
 *
 */
void jt65_encode(const char * msg, uint8_t * symbols)
{
  char message[14];
  memset(message, 0, 14);
  strcpy(message, msg);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  jt_message_prep(message);

  // Bit packing
  // -----------
  uint8_t c[12];
  jt65_bit_packing(message, c);

  // Reed-Solomon encoding
  // ---------------------
  uint8_t s[JT65_ENCODE_COUNT];
  rs_encode(c, s);

  // Interleaving
  // ------------
  jt65_interleave(s);

  // Gray Code
  // ---------
  jt_gray_code(s, JT65_ENCODE_COUNT);

  // Merge with sync vector
  // ----------------------
  jt65_merge_sync_vector(s, symbols);
}

/*
 * jt9_encode(const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 * a channel symbol table.
 *
 * message - Plaintext Type 6 message.
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least size JT9_SYMBOL_COUNT to the method.
 *
 */
void jt9_encode(const char * msg, uint8_t * symbols)
{
  char message[14];
  memset(message, 0, 14);
  strcpy(message, msg);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  jt_message_prep(message);

  // Bit packing
  // -----------
  uint8_t c[13];
  jt9_bit_packing(message, c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[JT9_BIT_COUNT];
  convolve(c, s, 13, JT9_BIT_COUNT);

  // Interleaving
  // ------------
  jt9_interleave(s);

  // Pack into 3-bit symbols
  // -----------------------
  uint8_t a[JT9_ENCODE_COUNT];
  jt9_packbits(s, a);

  // Gray Code
  // ---------
  jt_gray_code(a, JT9_ENCODE_COUNT);

  // Merge with sync vector
  // ----------------------
  jt9_merge_sync_vector(a, symbols);
}

/*
 * jt4_encode(const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 * a channel symbol table.
 *
 * message - Plaintext Type 6 message.
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least size JT9_SYMBOL_COUNT to the method.
 *
 */
void jt4_encode(const char * msg, uint8_t * symbols)
{
  char message[14];
  memset(message, 0, 14);
  strcpy(message, msg);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  jt_message_prep(message);

  // Bit packing
  // -----------
  uint8_t c[13];
  jt9_bit_packing(message, c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[JT4_SYMBOL_COUNT];
  convolve(c, s, 13, JT4_BIT_COUNT);

  // Interleaving
  // ------------
  jt9_interleave(s);
  memmove(s + 1, s, JT4_BIT_COUNT);
  s[0] = 0; // Append a 0 bit to start of sequence

  // Merge with sync vector
  // ----------------------
  jt4_merge_sync_vector(s, symbols);
}

/*
 * wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 *
 * call - Callsign (6 characters maximum).
 * loc - Maidenhead grid locator (4 characters maximum).
 * dbm - Output power in dBm.
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least size WSPR_SYMBOL_COUNT to the method.
 *
 */
void wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)
{
  char call_[7];
  char loc_[5];
  uint8_t dbm_ = dbm;
  strcpy(call_, call);
  strcpy(loc_, loc);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  wspr_message_prep(call_, loc_, dbm_);

  // Bit packing
  // -----------
  uint8_t c[11];
  wspr_bit_packing(c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[WSPR_SYMBOL_COUNT];
  convolve(c, s, 11, WSPR_BIT_COUNT);

  // Interleaving
  // ------------
  wspr_interleave(s);

  // Merge with sync vector
  // ----------------------
  wspr_merge_sync_vector(s, symbols);
}

/*
 * fsq_encode(const char * from_call, const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message and returns a FSQ channel symbol table.
 *
 * from_call - Callsign of issuing station (maximum size: 20)
 * message - Null-terminated message string, no greater than 130 chars in length
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least the size of the message
 *  plus 5 characters to the method. Terminated in 0xFF.
 *
 */
void fsq_encode(const char * from_call, const char * message, uint8_t * symbols)
{
  char tx_buffer[155];
  char * tx_message;
  uint16_t symbol_pos = 0;
  uint8_t i, fch, vcode1, vcode2, tone;
  uint8_t cur_tone = 0;

  // Clear out the transmit buffer
  // -----------------------------
  memset(tx_buffer, 0, 155);

  // Create the message to be transmitted
  // ------------------------------------
  sprintf(tx_buffer, "  \n%s: %s", from_call, message);

  tx_message = tx_buffer;

  // Iterate through the message and encode
  // --------------------------------------
  while(*tx_message != '\0')
  {
    for(i = 0; i < NGLYPHS; i++)
    {
      uint8_t ch = (uint8_t)*tx_message;

      // Check each element of the varicode table to see if we've found the
      // character we're trying to send.
      fch = fsq_code_table[i].ch;

      if(fch == ch)
      {
          // Found the character, now fetch the varicode chars
          vcode1 = (fsq_code_table[i].var[0]);
          vcode2 = (fsq_code_table[i].var[1]);

          // Transmit the appropriate tone per a varicode char
          if(vcode2 == 0)
          {
            // If the 2nd varicode char is a 0 in the table,
            // we are transmitting a lowercase character, and thus
            // only transmit one tone for this character.

            // Generate tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          else
          {
            // If the 2nd varicode char is anything other than 0 in
            // the table, then we need to transmit both

            // Generate 1st tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;

            // Generate 2nd tone
            cur_tone = ((cur_tone + vcode2 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          break; // We've found and transmitted the char,
             // so exit the for loop
        }
    }

    tx_message++;
  }

  // Message termination
  // ----------------
  symbols[symbol_pos] = 0xff;
}

/*
 * fsq_dir_encode(const char * from_call, const char * to_call, const char cmd, const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message and returns a FSQ channel symbol table.
 *
 * from_call - Callsign from which message is directed (maximum size: 20)
 * to_call - Callsign to which message is directed (maximum size: 20)
 * cmd - Directed command
 * message - Null-terminated message string, no greater than 100 chars in length
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least the size of the message
 *  plus 5 characters to the method. Terminated in 0xFF.
 *
 */
void fsq_dir_encode(const char * from_call, const char * to_call, const char cmd, const char * message, uint8_t * symbols)
{
  char tx_buffer[155];
  char * tx_message;
  uint16_t symbol_pos = 0;
  uint8_t i, fch, vcode1, vcode2, tone, from_call_crc;
  uint8_t cur_tone = 0;

  // Generate a CRC on from_call
  // ---------------------------
  from_call_crc = crc8(from_call);

  // Clear out the transmit buffer
  // -----------------------------
  memset(tx_buffer, 0, 155);

  // Create the message to be transmitted
  // We are building a directed message here.
  // FSQ very specifically needs "  \b  " in
  // directed mode to indicate EOT. A single backspace won't do it.
  sprintf(tx_buffer, "  \n%s:%02x%s%c%s%s", from_call, from_call_crc, to_call, cmd, message, "  \b  ");

  tx_message = tx_buffer;

  // Iterate through the message and encode
  // --------------------------------------
  while(*tx_message != '\0')
  {
    for(i = 0; i < NGLYPHS; i++)
    {
      uint8_t ch = (uint8_t)*tx_message;

      // Check each element of the varicode table to see if we've found the
      // character we're trying to send.
      fch = fsq_code_table[i].ch;

      if(fch == ch)
      {
          // Found the character, now fetch the varicode chars
          vcode1 = (fsq_code_table[i].var[0]);
          vcode2 = (fsq_code_table[i].var[1]);

          // Transmit the appropriate tone per a varicode char
          if(vcode2 == 0)
          {
            // If the 2nd varicode char is a 0 in the table,
            // we are transmitting a lowercase character, and thus
            // only transmit one tone for this character.

            // Generate tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          else
          {
            // If the 2nd varicode char is anything other than 0 in
            // the table, then we need to transmit both

            // Generate 1st tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;

            // Generate 2nd tone
            cur_tone = ((cur_tone + vcode2 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          break; // We've found and transmitted the char,
             // so exit the for loop
        }
    }

    tx_message++;
  }

  // Message termination
  // ----------------
  symbols[symbol_pos] = 0xff;
}

/*
 * ft8_encode(const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters or a telemetry message
 * of up to 18 hexadecimal digit (in string format) and returns a channel symbol table.
 * Encoded for the FT8 protocol used in WSJT-X v2.0 and beyond (79 channel symbols).
 *
 * message - Type 0.0 free text message or Type 0.5 telemetry message.
 * symbols - Array of channel symbols to transmit returned by the method.
 *  Ensure that you pass a uint8_t array of at least size FT8_SYMBOL_COUNT to the method.
 *
 */
void ft8_encode_msg(const char * msg, uint8_t * symbols)	//change function name
{
  uint8_t i;

  char message[19];
  memset(message, 0, 19);
  strcpy(message, msg);

  // Bit packing
  // -----------
  uint8_t c[77];
  memset(c, 0, 77);
  ft8_bit_packing(message, c);

  // Message Encoding
  // ----------------
  uint8_t s[FT8_BIT_COUNT];
  ft8_encode(c, s);

  // Merge with sync vector
  // ----------------------
  ft8_merge_sync_vector(s, symbols);
}

//Added FT4 Protocol by KD8CEC
void ft4_encode_msg(const char * msg, uint8_t * symbols)	//change function name
{
    #define KK 91	//)                     !Information bits (77 + CRC14)
    #define ND 87	//)                     !Data symbols
    #define NS 16	//)                     !Sync symbols
    //#define NN NS+ND	//)                  !Sync and data symbols (103)
    #define NN2 NS+ND+2	//)               !Total channel symbols (105)
    #define NSPS 512	//)                  !Samples per symbol at 12000 S/s
    //#define NZ NSPS*NN	//)                !Sync and Data samples (52736)
    #define NZ2 NSPS*NN2	//)              !Total samples in shaped waveform (53760)
    #define NMAX 18*3456	//)              !Samples in iwave
    //#define NFFT1=2048, NH1=NFFT1/2	//)   !Length of FFTs for symbol spectra
    #define NFFT1 2048
    #define NH1 NFFT1/2	//)   !Length of FFTs for symbol spectra
    #define NSTEP NSPS	//)                !Coarse time-sync step size
    #define NHSYM (NMAX-NFFT1)/NSTEP	//)  !Number of symbol spectra (1/4-sym steps)
    #define NDOWN 16	//)                  !Downsample factor

    uint8_t s[FT8_BIT_COUNT];

    const char icos4a[] = {0,1,3,2};
    const char icos4b[] = {1,0,2,3};
    const char icos4c[] = {2,3,1,0};
    const char icos4d[] = {3,2,0,1};
    const char rvec[] = {0,1,0,0,1,0,1,0,0,1,0,1,1,1,1,0,1,0,0,0,1,0,0,1,1,0,1,1,0,
            1,0,0,1,0,1,1,0,0,0,0,1,0,0,0,1,0,1,0,0,1,1,1,1,0,0,1,0,1,
            0,1,0,1,0,1,1,0,1,1,1,1,1,0,0,0,1,0,1};

    char message[37];
    memset(message, 0, 19);
    strcpy(message, msg);

    for (int i = 0; i < 37; i++)
    {
        if (message[i] == '\0')
            message[i] = ' ';
    }

  // Bit packing
  // -----------
  uint8_t c[77];
  memset(c, 0, 77);
  ft8_bit_packing(message, c);

	//FOR FT4 PROTOCOL
	//msgbits=mod(msgbits+rvec,2)
	for (int i = 0; i < 77; i++)
	{
		c[i] = (c[i] + rvec[i]) % 2;
	}

  //ENCODE174_91
  ft8_encode(c, s);

    //FOR FT4 PROTOCOL from WSJT-X 2.10 RC
	/*
! Grayscale mapping:
! bits   tone
! 00     0
! 01     1
! 11     2
! 10     3

  do i=1,ND
    is=codeword(2*i)+2*codeword(2*i-1)
    if(is.le.1) itmp(i)=is
    if(is.eq.2) itmp(i)=3
    if(is.eq.3) itmp(i)=2
  enddo
	*/
    char itmp[ND];

    for (int i = 0; i < ND; i++)
    {
        int is = s[2*i + 1] + 2 * s[2*i];

        if (is <= 1)
            itmp[i] = is;
        else if (is == 2)
            itmp[i] = 3;
        else if (is == 3)
            itmp[i] = 2;
    }

	memcpy(symbols, icos4a, 4);
	memcpy(symbols + 4, itmp, 29);
	memcpy(symbols + 33, icos4b, 4);
	memcpy(symbols + 37, itmp + 29, 29);
	memcpy(symbols + 66, icos4c, 4);
	memcpy(symbols + 70, itmp + 58, 29);
	memcpy(symbols + 99, icos4d, 4);
}

/* Private Class Members */

uint8_t jt_code(char c)
{
  // Validate the input then return the proper integer code.
  // Return 255 as an error code if the char is not allowed.

  if(isdigit(c))
  {
    return (uint8_t)(c - 48);
  }
  else if(c >= 'A' && c <= 'Z')
  {
    return (uint8_t)(c - 55);
  }
  else if(c == ' ')
  {
    return 36;
  }
  else if(c == '+')
  {
    return 37;
  }
  else if(c == '-')
  {
    return 38;
  }
  else if(c == '.')
  {
    return 39;
  }
  else if(c == '/')
  {
    return 40;
  }
  else if(c == '?')
  {
    return 41;
  }
  else
  {
    return 255;
  }
}

uint8_t ft_code(char c)
{
	/* Validate the input then return the proper integer code */
	// Return 255 as an error code if the char is not allowed

	if(isdigit(c))
	{
		return (uint8_t)(c) - 47;
	}
	else if(c >= 'A' && c <= 'Z')
	{
		return (uint8_t)(c) - 54;
	}
	else if(c == ' ')
	{
		return 0;
	}
  else if(c == '+')
	{
		return 37;
	}
	else if(c == '-')
	{
		return 38;
	}
	else if(c == '.')
	{
		return 39;
	}
	else if(c == '/')
	{
		return 40;
	}
	else if(c == '?')
	{
		return 41;
	}
	else
	{
		return 255;
	}
}

uint8_t wspr_code(char c)
{
  // Validate the input then return the proper integer code.
  // Return 255 as an error code if the char is not allowed.

  if(isdigit(c))
	{
		return (uint8_t)(c - 48);
	}
	else if(c == ' ')
	{
		return 36;
	}
	else if(c >= 'A' && c <= 'Z')
	{
		return (uint8_t)(c - 55);
	}
	else
	{
		return 255;
	}
}

uint8_t gray_code(uint8_t c)
{
  return (c >> 1) ^ c;
}

int8_t hex2int(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    return -1;
}

void jt_message_prep(char * message)
{
  uint8_t i;

  // Pad the message with trailing spaces
  uint8_t len = strlen(message);
  if(len < 13)
  {
    for(i = len; i <= 13; i++)
    {
      message[i] = ' ';
    }
  }

  // Convert all chars to uppercase
  for(i = 0; i < 13; i++)
  {
    if(islower(message[i]))
    {
      message[i] = toupper(message[i]);
    }
  }
}

void ft_message_prep(char * message)
{
  uint8_t i;
  char temp_msg[14];

  snprintf(temp_msg, 14, "%13s", message);

  // Convert all chars to uppercase
  for(i = 0; i < 13; i++)
  {
    if(islower(temp_msg[i]))
    {
      temp_msg[i] = toupper(temp_msg[i]);
    }
  }

  strcpy(message, temp_msg);
}

void wspr_message_prep(char * call, char * loc, uint8_t dbm)
{
  // Callsign validation and padding
  // -------------------------------

	// If only the 2nd character is a digit, then pad with a space.
	// If this happens, then the callsign will be truncated if it is
	// longer than 5 characters.
	if((call[1] >= '0' && call[1] <= '9') && (call[2] < '0' || call[2] > '9'))
	{
		memmove(call + 1, call, 5);
		call[0] = ' ';
	}

	// Now the 3rd charcter in the callsign must be a digit
	if(call[2] < '0' || call[2] > '9')
	{
    // TODO: need a better way to handle this
		call[2] = '0';
	}

	// Ensure that the only allowed characters are digits and
	// uppercase letters
	uint8_t i;
	for(i = 0; i < 6; i++)
	{
		call[i] = toupper(call[i]);
		if(!(isdigit(call[i]) || isupper(call[i])))
		{
			call[i] = ' ';
		}
	}

  memcpy(callsign, call, 6);

	// Grid locator validation
	for(i = 0; i < 4; i++)
	{
		loc[i] = toupper(loc[i]);
		if(!(isdigit(loc[i]) || (loc[i] >= 'A' && loc[i] <= 'R')))
		{
      memcpy(loc, "AA00", 5);
      //loc = "AA00";
		}
	}

  memcpy(locator, loc, 4);

	// Power level validation
	// Only certain increments are allowed
	if(dbm > 60)
	{
		dbm = 60;
	}
  const uint8_t valid_dbm[19] =
    {0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40,
     43, 47, 50, 53, 57, 60};
  for(i = 0; i < 19; i++)
  {
    if(dbm == valid_dbm[i])
    {
      power = dbm;
    }
  }
  // If we got this far, we have an invalid power level, so we'll round down
  for(i = 1; i < 19; i++)
  {
    if(dbm < valid_dbm[i] && dbm >= valid_dbm[i - 1])
    {
      power = valid_dbm[i - 1];
    }
  }
}

void jt65_bit_packing(char * message, uint8_t * c)
{
  uint32_t n1, n2, n3;

  // Find the N values
  n1 = jt_code(message[0]);
  n1 = n1 * 42 + jt_code(message[1]);
  n1 = n1 * 42 + jt_code(message[2]);
  n1 = n1 * 42 + jt_code(message[3]);
  n1 = n1 * 42 + jt_code(message[4]);

  n2 = jt_code(message[5]);
  n2 = n2 * 42 + jt_code(message[6]);
  n2 = n2 * 42 + jt_code(message[7]);
  n2 = n2 * 42 + jt_code(message[8]);
  n2 = n2 * 42 + jt_code(message[9]);

  n3 = jt_code(message[10]);
  n3 = n3 * 42 + jt_code(message[11]);
  n3 = n3 * 42 + jt_code(message[12]);

  // Pack bits 15 and 16 of N3 into N1 and N2,
  // then mask reset of N3 bits
  n1 = (n1 << 1) + ((n3 >> 15) & 1);
  n2 = (n2 << 1) + ((n3 >> 16) & 1);
  n3 = n3 & 0x7fff;

  // Set the freeform message flag
  n3 += 32768;

  c[0] = (n1 >> 22) & 0x003f;
  c[1] = (n1 >> 16) & 0x003f;
  c[2] = (n1 >> 10) & 0x003f;
  c[3] = (n1 >> 4) & 0x003f;
  c[4] = ((n1 & 0x000f) << 2) + ((n2 >> 26) & 0x0003);
  c[5] = (n2 >> 20) & 0x003f;
  c[6] = (n2 >> 14) & 0x003f;
  c[7] = (n2 >> 8) & 0x003f;
  c[8] = (n2 >> 2) & 0x003f;
  c[9] = ((n2 & 0x0003) << 4) + ((n3 >> 12) & 0x000f);
  c[10] = (n3 >> 6) & 0x003f;
  c[11] = n3 & 0x003f;
}

void jt9_bit_packing(char * message, uint8_t * c)
{
  uint32_t n1, n2, n3;

  // Find the N values
  n1 = jt_code(message[0]);
  n1 = n1 * 42 + jt_code(message[1]);
  n1 = n1 * 42 + jt_code(message[2]);
  n1 = n1 * 42 + jt_code(message[3]);
  n1 = n1 * 42 + jt_code(message[4]);

  n2 = jt_code(message[5]);
  n2 = n2 * 42 + jt_code(message[6]);
  n2 = n2 * 42 + jt_code(message[7]);
  n2 = n2 * 42 + jt_code(message[8]);
  n2 = n2 * 42 + jt_code(message[9]);

  n3 = jt_code(message[10]);
  n3 = n3 * 42 + jt_code(message[11]);
  n3 = n3 * 42 + jt_code(message[12]);

  // Pack bits 15 and 16 of N3 into N1 and N2,
  // then mask reset of N3 bits
  n1 = (n1 << 1) + ((n3 >> 15) & 1);
  n2 = (n2 << 1) + ((n3 >> 16) & 1);
  n3 = n3 & 0x7fff;

  // Set the freeform message flag
  n3 += 32768;

  // 71 message bits to pack, plus 1 bit flag for freeform message.
  // 31 zero bits appended to end.
  // N1 and N2 are 28 bits each, N3 is 16 bits
  // A little less work to start with the least-significant bits
  c[3] = (uint8_t)((n1 & 0x0f) << 4);
  n1 = n1 >> 4;
  c[2] = (uint8_t)(n1 & 0xff);
  n1 = n1 >> 8;
  c[1] = (uint8_t)(n1 & 0xff);
  n1 = n1 >> 8;
  c[0] = (uint8_t)(n1 & 0xff);

  c[6] = (uint8_t)(n2 & 0xff);
  n2 = n2 >> 8;
  c[5] = (uint8_t)(n2 & 0xff);
  n2 = n2 >> 8;
  c[4] = (uint8_t)(n2 & 0xff);
  n2 = n2 >> 8;
  c[3] |= (uint8_t)(n2 & 0x0f);

  c[8] = (uint8_t)(n3 & 0xff);
  n3 = n3 >> 8;
  c[7] = (uint8_t)(n3 & 0xff);

  c[9] = 0;
  c[10] = 0;
  c[11] = 0;
  c[12] = 0;
}

void wspr_bit_packing(uint8_t * c)
{
  uint32_t n, m;

	n = wspr_code(callsign[0]);
	n = n * 36 + wspr_code(callsign[1]);
	n = n * 10 + wspr_code(callsign[2]);
	n = n * 27 + (wspr_code(callsign[3]) - 10);
	n = n * 27 + (wspr_code(callsign[4]) - 10);
	n = n * 27 + (wspr_code(callsign[5]) - 10);

	m = ((179 - 10 * (locator[0] - 'A') - (locator[2] - '0')) * 180) +
		(10 * (locator[1] - 'A')) + (locator[3] - '0');
	m = (m * 128) + power + 64;

	// Callsign is 28 bits, locator/power is 22 bits.
	// A little less work to start with the least-significant bits
	c[3] = (uint8_t)((n & 0x0f) << 4);
	n = n >> 4;
	c[2] = (uint8_t)(n & 0xff);
	n = n >> 8;
	c[1] = (uint8_t)(n & 0xff);
	n = n >> 8;
	c[0] = (uint8_t)(n & 0xff);

	c[6] = (uint8_t)((m & 0x03) << 6);
	m = m >> 2;
	c[5] = (uint8_t)(m & 0xff);
	m = m >> 8;
	c[4] = (uint8_t)(m & 0xff);
	m = m >> 8;
	c[3] |= (uint8_t)(m & 0x0f);
	c[7] = 0;
	c[8] = 0;
	c[9] = 0;
	c[10] = 0;
}

void ft8_bit_packing(char* message, uint8_t* codeword)
{
    // Just encoding type 0 free text and type 0.5 telemetry for now

	// The bit packing algorithm is:
	// sum(message(pos) * 42^pos)

	uint8_t i3 = 0;
	uint8_t n3 = 0;
	uint8_t qa[10];
	uint8_t qb[10];
	char c18[19];
	bool telem = false;
	char temp_msg[19];
	memset(qa, 0, 10);
	memset(qb, 0, 10);

	uint8_t i, j, x, i0;
	uint32_t ireg = 0;

	// See if this is a telemetry message
	// Has to be hex digits, can be no more than 18
	for(i = 0; i < 19; ++i)
	{
		if(message[i] == 0 || message[i] == ' ')
		{
			break;
		}
		else if(hex2int(message[i]) == -1)
		{
			telem = false;
			break;
		}
		else
		{
			c18[i] = message[i];
			telem = true;
		}
	}

	// If telemetry
	if(telem)
	{
		// Get the first 18 hex digits
		for(i = 0; i < strlen(message); ++i)
		{
			i0 = i;
			if(message[i] == ' ')
			{
				--i0;
				break;
			}
		}

		memset(c18, 0, 19);
		memmove(c18, message, i0 + 1);
		snprintf(temp_msg, 19, "%*s", 18, c18);

		// Convert all chars to uppercase
	    for(i = 0; i < strlen(temp_msg); i++)
	    {
	      if(islower(temp_msg[i]))
	      {
	        temp_msg[i] = toupper(temp_msg[i]);
	      }
	    }
		strcpy(message, temp_msg);


		uint8_t temp_int;
		temp_int = message[0] == ' ' ? 0 : hex2int(message[0]);
		for(i = 1; i < 4; ++i)
		{
			codeword[i - 1] = (((temp_int << i) & 0x8) >> 3) & 1;
		}
		temp_int = message[1] == ' ' ? 0 : hex2int(message[1]);
		for(i = 0; i < 4; ++i)
		{
			codeword[i + 3] = (((temp_int << i) & 0x8) >> 3) & 1;
		}
		for(i = 0; i < 8; ++i)
		{
			if(message[2 * i + 2] == ' ')
			{
				temp_int = 0;
			}
			else
			{
				temp_int = hex2int(message[2 * i + 2]);
			}
			for(j = 0; j < 4; ++j)
			{
				codeword[(i + 1) * 8 + j - 1] = (((temp_int << j) & 0x8) >> 3) & 1;
			}
			if(message[2 * i + 3] == ' ')
			{
				temp_int = 0;
			}
			else
			{
				temp_int = hex2int(message[2 * i + 3]);
			}
			for(j = 0; j < 4; ++j)
			{
				codeword[(i + 1) * 8 + j + 3] = (((temp_int << j) & 0x8) >> 3) & 1;
			}
		}

		i3 = 0;
		n3 = 5;
	}
	else
	{
		ft_message_prep(message);

		for(i = 0; i < 13; ++i)
		{
			x = ft_code(message[i]);

			// mult
			ireg = 0;
			for(j = 0; j < 9; ++j)
			{
				ireg = (uint8_t)qa[j] * 42 + (uint8_t)((ireg >> 8) & 0xff);
				qb[j] = (uint8_t)(ireg & 0xff);
			}
			qb[9] = (uint8_t)((ireg >> 8) & 0xff);

			// add
			ireg = x << 8;
			for(j = 0; j < 9; ++j)
			{
				ireg = (uint8_t)qb[j] + (uint8_t)((ireg >> 8) & 0xff);
				qa[j] = (uint8_t)(ireg & 0xff);
			}
			qa[9] = (uint8_t)((ireg >> 8) & 0xff);
		}

		// Format bits to output array
		for(i = 1; i < 8; ++i)
		{
			codeword[i - 1] = (((qa[8] << i) & 0x80) >> 7) & 1;
		}
		for(i = 0; i < 8; ++i)
		{
			for(j = 0; j < 8; ++j)
			{
				codeword[(i + 1) * 8 + j - 1] = (((qa[7 - i] << j) & 0x80) >> 7) & 1;
			}
		}
	}

	// Write the message type bits at the end of the array
	for(i = 0; i < 3; ++i)
	{
		codeword[i + 71] = (n3 >> i) & 1;
	}
	for(i = 0; i < 3; ++i)
	{
		codeword[i + 74] = (i3 >> i) & 1;
	}
}

void jt65_interleave(uint8_t * s)
{
  uint8_t i, j;
  uint8_t d[JT65_ENCODE_COUNT];

  // Interleave
  for(i = 0; i < 9; i++)
  {
    for(j = 0; j < 7; j++)
    {
      d[(j * 9) + i] = s[(i * 7) + j];
    }
  }

  memcpy(s, d, JT65_ENCODE_COUNT);
}

void jt9_interleave(uint8_t * s)
{
  uint8_t i, j;
  uint8_t d[JT9_BIT_COUNT];

  // Do the interleave
  for(i = 0; i < JT9_BIT_COUNT; i++)
  {
    //#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    #if defined(__arm__)
    d[jt9i[i]] = s[i];
    #else
    j = pgm_read_byte(&jt9i[i]);
    d[j] = s[i];
    #endif
  }

  memcpy(s, d, JT9_BIT_COUNT);
}

void wspr_interleave(uint8_t * s)
{
  uint8_t d[WSPR_BIT_COUNT];
	uint8_t rev, index_temp, i, j, k;

	i = 0;

	for(j = 0; j < 255; j++)
	{
		// Bit reverse the index
		index_temp = j;
		rev = 0;

		for(k = 0; k < 8; k++)
		{
			if(index_temp & 0x01)
			{
				rev = rev | (1 << (7 - k));
			}
			index_temp = index_temp >> 1;
		}

		if(rev < WSPR_BIT_COUNT)
		{
			d[rev] = s[i];
			i++;
		}

		if(i >= WSPR_BIT_COUNT)
		{
			break;
		}
	}

  memcpy(s, d, WSPR_BIT_COUNT);
}

void jt9_packbits(uint8_t * d, uint8_t * a)
{
  uint8_t i, k;
  k = 0;
  memset(a, 0, JT9_ENCODE_COUNT);

  for(i = 0; i < JT9_ENCODE_COUNT; i++)
  {
    a[i] = (d[k] & 1) << 2;
    k++;

    a[i] |= ((d[k] & 1) << 1);
    k++;

    a[i] |= (d[k] & 1);
    k++;
  }
}

void jt_gray_code(uint8_t * g, uint8_t symbol_count)
{
  uint8_t i;

  for(i = 0; i < symbol_count; i++)
  {
    g[i] = gray_code(g[i]);
  }
}

void ft8_encode(uint8_t* codeword, uint8_t* symbols)
{
	const uint8_t FT8_N = 174;
	const uint8_t FT8_K = 91;
	const uint8_t FT8_M = FT8_N - FT8_K;

	uint8_t tempchar[FT8_K];
	uint8_t message91[FT8_K];
	uint8_t pchecks[FT8_M];
	uint8_t i1_msg_bytes[12];
	uint8_t i, j;
	uint16_t ncrc14;

	crc_t crc;
	crc_cfg_t crc_cfg;
	crc_cfg.reflect_in = 0;
	crc_cfg.xor_in = 0;
	crc_cfg.reflect_out = 0;
	crc_cfg.xor_out = 0;
	crc = crc_init(&crc_cfg);

	// Add 14-bit CRC to form 91-bit message
	memset(tempchar, 0, 91);
	memcpy(tempchar, codeword, 77);
	tempchar[77] = 0;
	tempchar[78] = 0;
	tempchar[79] = 0;
	memset(i1_msg_bytes, 0, 12);
	for(i = 0; i < 10; ++i)
	{
		for(j = 0; j < 8; ++j)
		{
			i1_msg_bytes[i] <<= 1;
			i1_msg_bytes[i] |= tempchar[i * 8 + j];
		}
	}

	ncrc14 = crc_update(&crc_cfg, crc, (unsigned char *)i1_msg_bytes, 12);
	crc = crc_finalize(&crc_cfg, crc);

	for(i = 0; i < 14; ++i)
	{
		if((((ncrc14 << (i + 2)) & 0x8000) >> 15) & 1)
		{
			tempchar[i + 77] = 1;
		}
		else
		{
			tempchar[i + 77] = 0;
		}
	}
	memcpy(message91, tempchar, 91);

	for(i = 0; i < FT8_M; ++i)
	{
		uint32_t nsum = 0;
		for(j = 0; j < FT8_K; ++j)
		{
      #if defined(__arm__)
      uint8_t bits = generator_bits[i][j / 8];
      #else
      uint8_t bits = pgm_read_byte(&(generator_bits[i][j / 8]));
      #endif
			bits <<= (j % 8);
			bits &= 0x80;
			bits >>= 7;
			bits &= 1;
			nsum += (message91[j] * bits);
		}
		pchecks[i] = nsum % 2;
	}

	memcpy(symbols, message91, FT8_K);
	memcpy(symbols + FT8_K, pchecks, FT8_M);
}

void jt65_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i, j = 0;
  const uint8_t sync_vector[JT65_SYMBOL_COUNT] =
  {1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0,
   0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1,
   0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1,
   0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
   1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1,
   0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1,
   1, 1, 1, 1, 1, 1};

  for(i = 0; i < JT65_SYMBOL_COUNT; i++)
  {
    if(sync_vector[i])
    {
      symbols[i] = 0;
    }
    else
    {
      symbols[i] = g[j] + 2;
      j++;
    }
  }
}

void jt9_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i, j = 0;
  const uint8_t sync_vector[JT9_SYMBOL_COUNT] =
  {1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1,
   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 1, 0, 1};

  for(i = 0; i < JT9_SYMBOL_COUNT; i++)
  {
    if(sync_vector[i])
    {
      symbols[i] = 0;
    }
    else
    {
      symbols[i] = g[j] + 1;
      j++;
    }
  }
}

void jt4_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i;
  const uint8_t sync_vector[JT4_SYMBOL_COUNT] =
	{0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0,
   0, 0, 0, 0, 1, 1, 0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,0 ,1 ,1,
   0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0,
   1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0,
   0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0,
   1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1,
   1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1,
   0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1,
   1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1,
   0, 1, 1, 1, 1, 0, 1, 0, 1};

	for(i = 0; i < JT4_SYMBOL_COUNT; i++)
	{
		symbols[i] = sync_vector[i] + (2 * g[i]);
	}
}

void wspr_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i;
  const uint8_t sync_vector[WSPR_SYMBOL_COUNT] =
	{1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0,
	 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
	 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
	 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0,
	 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
	 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
	 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
	 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0};

	for(i = 0; i < WSPR_SYMBOL_COUNT; i++)
	{
		symbols[i] = sync_vector[i] + (2 * g[i]);
	}
}

void ft8_merge_sync_vector(uint8_t* symbols, uint8_t* output)
{
	const uint8_t costas7x7[7] = {3, 1, 4, 0, 6, 5, 2};
	const uint8_t graymap[8] = {0, 1, 3, 2, 5, 6, 4, 7};
	uint8_t i, j, k, idx;

	// Insert Costas sync arrays
	memcpy(output, costas7x7, 7);
	memcpy(output + 36, costas7x7, 7);
	memcpy(output + FT8_SYMBOL_COUNT - 7, costas7x7, 7);

	k = 6;
	for(j = 0; j < 58; ++j) // 58 data symbols
	{
		i = 3 * j;
		++k;
		if(j == 29)
		{
			k += 7;
		}
		idx = symbols[i] * 4 + symbols[i + 1] * 2 + symbols[i + 2];
		output[k] = graymap[idx];
	}
}

void convolve(uint8_t * c, uint8_t * s, uint8_t message_size, uint8_t bit_size)
{
  uint32_t reg_0 = 0;
  uint32_t reg_1 = 0;
  uint32_t reg_temp = 0;
  uint8_t input_bit, parity_bit;
  uint8_t bit_count = 0;
  uint8_t i, j, k;

  for(i = 0; i < message_size; i++)
  {
    for(j = 0; j < 8; j++)
    {
      // Set input bit according the MSB of current element
      input_bit = (((c[i] << j) & 0x80) == 0x80) ? 1 : 0;

      // Shift both registers and put in the new input bit
      reg_0 = reg_0 << 1;
      reg_1 = reg_1 << 1;
      reg_0 |= (uint32_t)input_bit;
      reg_1 |= (uint32_t)input_bit;

      // AND Register 0 with feedback taps, calculate parity
      reg_temp = reg_0 & 0xf2d05351;
      parity_bit = 0;
      for(k = 0; k < 32; k++)
      {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
      s[bit_count] = parity_bit;
      bit_count++;

      // AND Register 1 with feedback taps, calculate parity
      reg_temp = reg_1 & 0xe4613c47;
      parity_bit = 0;
      for(k = 0; k < 32; k++)
      {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
      s[bit_count] = parity_bit;
      bit_count++;
      if(bit_count >= bit_size)
      {
        break;
      }
    }
  }
}

void rs_encode(uint8_t * data, uint8_t * symbols)
{
  // Adapted from wrapkarn.c in the WSJT-X source code
  uint8_t dat1[12];
  uint8_t b[51];
  uint8_t sym[JT65_ENCODE_COUNT];
  uint8_t i;

  // Reverse data order for the Karn codec.
  for(i = 0; i < 12; i++)
  {
    dat1[i] = data[11 - i];
  }

  // Compute the parity symbols
  encode_rs_int(rs_inst, dat1, b);

  // Move parity symbols and data into symbols array, in reverse order.
  for (i = 0; i < 51; i++)
  {
    sym[50 - i] = b[i];
  }

  for (i = 0; i < 12; i++)
  {
    sym[i + 51] = dat1[11 - i];
  }

  memcpy(symbols, sym, JT65_ENCODE_COUNT);
}

uint8_t crc8(const char * text)
{
  uint8_t crc = '\0';
  uint8_t ch;

  int i;
  for(i = 0; i < strlen(text); i++)
  {
    ch = text[i];
    //#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    #if defined(__arm__)
    crc = crc8_table[(crc) ^ ch];
    #else
    crc = pgm_read_byte(&(crc8_table[(crc) ^ ch]));
    #endif
    crc &= 0xFF;
  }

  return crc;
}
