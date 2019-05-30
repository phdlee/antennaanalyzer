#include "touch.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_lcd.h"
#include "lcd.h"
#include "config.h"

void TOUCH_Init(void)
{
    BSP_TS_Init(FT5336_MAX_WIDTH, FT5336_MAX_HEIGHT);
}

static uint32_t wakeup_touch(void)
{
    extern volatile uint32_t autosleep_timer;
    autosleep_timer = CFG_GetParam(CFG_PARAM_LOWPWR_TIME);
    if (LCD_IsOff())
    {
        BSP_LCD_DisplayOn();
        TS_StateTypeDef ts = { 0 };
        do
        {
            BSP_TS_GetState(&ts);
        } while (ts.touchDetected);
        return 1;
    }
    return 0;
}

uint8_t TOUCH_Poll(LCDPoint* pCoord)
{
    TS_StateTypeDef ts = { 0 };
    BSP_TS_GetState(&ts);
    if (ts.touchDetected)
    {
        if (wakeup_touch())
            return 0;
        pCoord->x = ts.touchX[0];
        pCoord->y = ts.touchY[0];
    }
    return ts.touchDetected;
}

uint8_t TOUCH_IsPressed(void)
{
    TS_StateTypeDef ts = { 0 };
    BSP_TS_GetState(&ts);
    if (ts.touchDetected)
    {
        if (wakeup_touch())
            return 0;
    }
    return ts.touchDetected;
}

//KD8CEC
int GetTouchIndex(LCDPoint pt, const int checkButtons[][4], int checkCount)
{
    #define TOUCH_TOP_MARGIN 5
    int rst = -1;

    for (int i = 0; i < checkCount; i++)
    {
        if (pt.x >= checkButtons[i][BUTTON_LEFT] && pt.x <= checkButtons[i][BUTTON_RIGHT] &&
            pt.y + TOUCH_TOP_MARGIN >= checkButtons[i][BUTTON_TOP] && pt.y <= checkButtons[i][BUTTON_BOTTOM])
        {

            return i;
        }
    }
    return -1;
}


