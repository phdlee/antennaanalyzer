/*
 *   KD8CEC
 *   kd8cec@gmail.com
 *
 *   for Audio Callback
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <string.h>
#include <math.h>
#include <limits.h>
#include <complex.h>
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"

#include "config.h"
#include "crash.h"

#include "audioirq.h"

uint8_t Audio_Status;
uint8_t Audio_Play_Status;
//uint8_t Audio_Error;

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  Audio_Status = AUDIO_TRANSFER_COMPLETE;
  return;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  Audio_Status = AUDIO_TRANSFER_HALF;
  return;
}

void BSP_AUDIO_IN_Error_CallBack(void)
{
  //Audio_Error = 1;
  //DBG_Printf("DMA  ERROR     ");
}


void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
    Audio_Play_Status  = AUDIO_TRANSFER_COMPLETE;
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
    Audio_Play_Status  = AUDIO_TRANSFER_HALF;
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

