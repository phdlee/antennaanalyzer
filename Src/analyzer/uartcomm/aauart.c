/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   Modified : by KD8CEC , Added feather for DEBUG
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "main.h"
#include "stm32f7xx_hal_uart.h"
#include "config.h"
#include "aauart.h"
#include "fifo.h"
#include <stdarg.h>

static FIFO_Descr rxfifo;
static volatile int32_t AAUART_busy = 0;
static volatile uint32_t rx_overflow_ctr = 0;
static UART_HandleTypeDef UartHandle = {0};

static const uint8_t* volatile txptr = 0;
static volatile uint32_t txctr = 0;

//==================================================================
//KD8CEC ROUTINE for Debug
//==================================================================
static UART_HandleTypeDef DBGUartHandle = {0};
static FIFO_Descr DBGrxfifo;

static uint8_t DBG_ACTIVE = 0;

void DBGUART_Init(void)
{
    DBGUartHandle.Init.BaudRate = 115200; //9600;
    DBGUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    DBGUartHandle.Init.StopBits = UART_STOPBITS_1;
    DBGUartHandle.Init.Parity = UART_PARITY_NONE;
    DBGUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    DBGUartHandle.Init.Mode = UART_MODE_TX_RX;
    BSP_COM_Init(COM2, &DBGUartHandle);

    DBG_ACTIVE = 1;
}

void DBGUART_Char(int ch)
{
    //Check Init UART
    if (! DBG_ACTIVE)
        return;

    DBGUartHandle.Instance->TDR = ch;
    while(!(DBGUartHandle.Instance->ISR & USART_ISR_TXE));
}

void DBG_Str(const char* str)
{
    for (int i = 0; i < strlen(str); i++)
        DBGUART_Char(str[i]);
    DBGUART_Char(10);
    DBGUART_Char(13);
}

static char tmpBuf[256];

int DBG_Printf(const char *fmt, ...)
{

    int np = 0;
    va_list ap;
    va_start(ap, fmt);
    np = vsnprintf(tmpBuf, 255, fmt, ap);
    tmpBuf[np] = '\0';
    va_end(ap);
    DBG_Str(tmpBuf);
}

//=======================================================================
//END OF KD8CEC ROUTINE for Debug
//=======================================================================


void AAUART_Init(void)
{
    uint32_t comport = CFG_GetParam(CFG_PARAM_COM_PORT);
    uint32_t comspeed = CFG_GetParam(CFG_PARAM_COM_SPEED);
    int IRQn;

    FIFO_Init(&rxfifo);

    if (COM1 == comport)
        IRQn = DISCOVERY_COM1_IRQn;
    else if (COM2 == comport)
        IRQn = DISCOVERY_COM2_IRQn;
    else
        return;

    UartHandle.Init.BaudRate = comspeed;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;

    BSP_COM_Init(comport, &UartHandle);
    // NVIC for USART
    HAL_NVIC_SetPriority(IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(IRQn);
    MODIFY_REG(UartHandle.Instance->CR1, 0, USART_CR1_RXNEIE); //Enable RXNE interrupts
}

//======================================================
void AAUART_IRQHandler(void)
{
    uint32_t isrflags   = READ_REG(UartHandle.Instance->ISR);
    volatile uint8_t byte;
    UartHandle.Instance->ICR  = 0x3ffff; //Clear all flags
    if ((isrflags & USART_ISR_TC) != RESET)
    {
        UartHandle.Instance->ICR  = USART_ISR_TC;
        if (txctr)
        {
            UartHandle.Instance->TDR = *txptr++;  //Writing TDR clears interrupt flag
            txctr--;
        }
        else
        {
            //All bytes have been transmitted
            //Disable TXE interrupt
            MODIFY_REG(UartHandle.Instance->CR1, 0, USART_CR1_TCIE);
            AAUART_busy = 0;
        }
    }

    if ((isrflags & USART_ISR_RXNE) != RESET)
    {
        //Read one byte from the receive data register
        byte = (uint8_t)(UartHandle.Instance->RDR); //Reading RDR clears interrupt flag
        if (FIFO_OK != FIFO_Put(&rxfifo, byte))
        {
            rx_overflow_ctr++;
        }
    }
    __DSB();
}

uint32_t AAUART_IsBusy(void)
{
    return AAUART_busy;
}
//======================================================
int AAUART_Getchar(void)
{
    uint8_t ch;
    __disable_irq();
    FIFO_STATUS res = FIFO_Get(&rxfifo, &ch);
    __enable_irq();
    if (FIFO_OK != res)
        return EOF;
    return (int)ch;
}

//======================================================
void AAUART_PutString(const char* str)
{
    AAUART_PutBytes((const uint8_t*)str, strlen(str));
}

//======================================================
void AAUART_PutBytes(const uint8_t* bytes, uint32_t len)
{
    if (0 == len || 0 == bytes)
        return;
    while (AAUART_busy); //Block until previous transmission is over
    txctr = len - 1;
    txptr = &bytes[1];
    AAUART_busy = 1;
    UartHandle.Instance->TDR = bytes[0];
    SET_BIT(UartHandle.Instance->CR1, USART_CR1_TCIE);
}

//======================================================
uint32_t AAUART_GetRxOvfCount(void)
{
    return rx_overflow_ctr;
}
