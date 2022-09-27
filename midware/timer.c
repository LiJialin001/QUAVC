#include "timer.h"

/******************************************************************************
  * 函数名称：Timer_Init
  * 函数描述：定时器初始化
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *
  *
******************************************************************************/
void Timer_Init()
{
    Timer_B_initUpModeParam Timer_B_initUpModeParam_cache;
    
    // Timer B0 1ms Period
    // 25 000 000 / 20 = 1 250 000
    Timer_B_initUpModeParam_cache.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
    Timer_B_initUpModeParam_cache.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_20;
    Timer_B_initUpModeParam_cache.timerPeriod = 1250 - 1;
    Timer_B_initUpModeParam_cache.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_ENABLE;
    Timer_B_initUpModeParam_cache.captureCompareInterruptEnable_CCR0_CCIE = TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE;
    Timer_B_initUpModeParam_cache.timerClear = TIMER_B_DO_CLEAR;
    Timer_B_initUpModeParam_cache.startTimer = true;
    Timer_B_initUpMode(TIMER_B0_BASE, &Timer_B_initUpModeParam_cache);
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
