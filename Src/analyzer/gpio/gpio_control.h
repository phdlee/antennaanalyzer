//========================================================================================
//GPIO CONTROL for Version 0.4 ~ 1.0
//by KD8CEC
//04/15/2019
//
//PTT, CWTX, CW PADDLE
//----------------------------------------------------------------------------------------

#ifndef GPIO_CONTROL_H_
#define GPIO_CONTROL_H_

#include <stdio.h>
#include <stdint.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_def.h"
#include "stm32746g_discovery.h"
#include "stm32f7xx_hal_adc.h"


//GPIO for PTT Control by KD8CEC
//B4
#define PTT_TX_PIN  GPIO_PIN_4
#define PTT_TX_GPIO GPIOB
extern uint8_t ptt_status;
void SET_PTT(uint8_t isPTTOn);
void GPIO_PTT_Setup(void);

#define CW_TX_PIN  GPIO_PIN_3
#define CW_TX_GPIO GPIOI
extern uint8_t cw_status;
void SET_CWTX(uint8_t isCWOn);
void GPIO_CWTX_Setup(void);

//GPIO for CW PADDLE or Straight Key Control by KD8CEC
//PG6, PG7   //INPUT
#define CW_PADDLE_DOT_PIN  GPIO_PIN_6
#define CW_PADDLE_DIT_PIN  GPIO_PIN_7
#define CW_PADDLE_GPIO GPIOG

//0: polling Mode, 1 : using Interrupt
#define CW_INT_MODE 0

void GPIO_CWPaddle_Setup(void);

#if CW_INT_MODE == 1
//o : off, off
//1 : on, off
//2 : off, on
//3 : on , on
int Get_Paddle_Status(void);
//extern uint8_t cw_paddle_DIT;
//extern uint8_t cw_paddle_DOT;

#endif


#endif
