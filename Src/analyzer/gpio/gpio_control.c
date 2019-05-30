//========================================================================================
//GPIO CONTROL for Version 0.4 ~ 1.0
//by KD8CEC
//04/15/2019
//
//PTT, CWTX, CW PADDLE
//----------------------------------------------------------------------------------------
//GPIO for PTT Control by KD8CEC
//B4

#include "gpio_control.h"

uint8_t ptt_status = 0;
void SET_PTT(uint8_t isPTTOn)
{
    ptt_status = isPTTOn;
    HAL_GPIO_WritePin(PTT_TX_GPIO, PTT_TX_PIN, ptt_status);
}

void GPIO_PTT_Setup(void)
{
    GPIO_InitTypeDef gpioInitStructure;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpioInitStructure.Pin       = PTT_TX_PIN;
    gpioInitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
    gpioInitStructure.Pull      = GPIO_NOPULL;   //
    gpioInitStructure.Speed     = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(PTT_TX_GPIO,  &gpioInitStructure);
}

uint8_t cw_status = 0;
void SET_CWTX(uint8_t isCWOn)
{
    cw_status = isCWOn;
    HAL_GPIO_WritePin(CW_TX_GPIO, CW_TX_PIN, cw_status);
}

void GPIO_CWTX_Setup(void)
{
    GPIO_InitTypeDef gpioInitStructure;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpioInitStructure.Pin       = CW_TX_PIN;
    gpioInitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
    gpioInitStructure.Pull      = GPIO_NOPULL;   //
    gpioInitStructure.Speed     = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(CW_TX_GPIO,  &gpioInitStructure);
}

//GPIO for CW PADDLE or Straight Key Control by KD8CEC
//PG6, PG7   //INPUT

#if CW_INT_MODE == 1
/*
    void EXTI9_5_IRQHandler(void)
    {
        if(EXTI->PR & (1 << 6))
        {
            HAL_GPIO_TogglePin(WSPR_TX_GPIO, WSPR_TX_PIN);
            HAL_Delay(100);
            EXTI->PR = (1 << 6);
        }
    }
*/

    void GPIO_CWPaddle_Setup(void)
    {
        GPIO_InitTypeDef gpioInitStructure;

        GPIO_InitTypeDef gpioIn;
        __HAL_RCC_GPIOG_CLK_ENABLE();
        gpioIn.Pin = CW_PADDLE_DOT_PIN;
        gpioIn.Mode = GPIO_MODE_INPUT;
        gpioIn.Pull = GPIO_PULLUP;   //
        gpioIn.Speed = GPIO_SPEED_FAST;

        //Interrupt
        gpioIn.Mode = GPIO_MODE_IT_RISING_FALLING;
        HAL_GPIO_Init(CW_PADDLE_GPIO, &gpioIn);

        gpioIn.Pin = CW_PADDLE_DIT_PIN;
        HAL_GPIO_Init(CW_PADDLE_GPIO, &gpioIn);

        //For Interrupt
        HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x00);
        HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));
    }

#else

    void GPIO_CWPaddle_Setup(void)
    {
        GPIO_InitTypeDef gpioInitStructure;

        GPIO_InitTypeDef gpioIn;
        __HAL_RCC_GPIOG_CLK_ENABLE();
        gpioIn.Pin = CW_PADDLE_DOT_PIN;
        gpioIn.Mode = GPIO_MODE_INPUT;
        gpioIn.Pull = GPIO_PULLUP;   //
        gpioIn.Speed = GPIO_SPEED_FAST;

        //Interrupt
        //gpioIn.Mode = GPIO_MODE_IT_RISING_FALLING;
        HAL_GPIO_Init(CW_PADDLE_GPIO, &gpioIn);

        gpioIn.Pin = CW_PADDLE_DIT_PIN;
        HAL_GPIO_Init(CW_PADDLE_GPIO, &gpioIn);

        //For Interrupt
        //HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x00);
        //HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));
    }

    int Get_Paddle_Status(void)
    {
        return (HAL_GPIO_ReadPin(CW_PADDLE_GPIO, CW_PADDLE_DIT_PIN) == GPIO_PIN_RESET) |
            ((HAL_GPIO_ReadPin(CW_PADDLE_GPIO, CW_PADDLE_DOT_PIN) == GPIO_PIN_RESET) << 1);
    }

#endif

//----------------------------------------------------------------------------------------
