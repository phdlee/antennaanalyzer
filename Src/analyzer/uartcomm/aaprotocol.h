/*
 *   KD8CEC
 *   kd8cec@gmail.com
 *
 */

#ifndef AAPROTOCOL_H_INCLUDED
#define AAPROTOCOL_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void PROTOCOL_Reset(void);
void PROTOCOL_Handler(void);
void PROTOCOL_Handler_VNA(void);
#ifdef __cplusplus
}
#endif

#endif //AAUART_H_INCLUDED
