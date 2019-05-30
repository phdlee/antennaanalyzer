/*
 *   KD8CEC
 *   kd8cec@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef AUDIODSP_H_INCLUDED
#define AUDIODSP_H_INCLUDED

#include <stdint.h>
#include <complex.h>
#include "stm32746g_discovery_audio.h"

#define AUDIO_TRANSFER_NONE     0
#define AUDIO_TRANSFER_HALF     1
#define AUDIO_TRANSFER_COMPLETE 2

extern uint8_t Audio_Status;
extern uint8_t Audio_Play_Status;

//Callback function using HAL Driver
void BSP_AUDIO_IN_TransferComplete_CallBack(void);
void BSP_AUDIO_IN_HalfTransfer_CallBack(void);
void BSP_AUDIO_IN_Error_CallBack(void);

#endif //DSP_H_INCLUDED
