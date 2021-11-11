#include "stm32f10x.h"
#include "svpwm.h"
#include "sysTick.h"

static unsigned long timer1;

void deviceInit(void)
{
    RCC->APB2ENR = RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    GPIOB->CRH = (GPIOB->CRH & ( ~ GPIO_CRH_CNF9)) | GPIO_CRH_MODE9;
    sysTickInit();
    svpwmInit();
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(4, 1, 0));
    NVIC_EnableIRQ(TIM4_IRQn);
}

int main()
{
    unsigned long phase = 0;
    deviceInit();
    for(;;)
    {
        asm volatile("wfi \n");
        if((sysTime - timer1) > 2)
        {
            timer1 = sysTime;
            phase += 1;
            if(phase >= (6 * PHASE_S))
                phase = 0;
            svpwmSet(phase, 255);
        }
    }
    return 0;
}
