#include "stm32f10x.h"

unsigned long sysTime;

void sysTickInit(void)
{
    SysTick->LOAD = 72000 - 1;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
 * 功能: SysTick 中断服务
 * 参数: 无
 * 返回: 无
 */
void SysTick_Handler(void)
{
    sysTime++;
    asm volatile("" : : : "memory");
}
