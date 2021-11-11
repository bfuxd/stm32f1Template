#include "stm32f10x.h"
#include "svpwm.h"

static unsigned long phase, m;

/**
 * SVPWM 加速表
 * 由 ./svpwmTools/svpwm.m 生成
 */
const static unsigned char wave[] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
    254, 254, 254, 254, 254, 254, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253,
    253, 252, 252, 252, 252, 252, 252, 252, 252, 252, 251, 251, 251, 251, 251, 251,
    251, 251, 250, 250, 250, 250, 250, 250, 250, 249, 249, 249, 249, 249, 249, 248,
    248, 248, 248, 248, 248, 247, 247, 247, 247, 247, 246, 246, 246, 246, 246, 245,
    245, 245, 245, 245, 244, 244, 244, 244, 244, 243, 243, 243, 243, 243, 242, 242,
    242, 242, 241, 241, 241, 241, 240, 240, 240, 240, 239, 239, 239, 239, 238, 238,
    238, 237, 236, 236, 235, 234, 233, 232, 232, 231, 230, 229, 228, 228, 227, 226,
    225, 224, 224, 223, 222, 221, 220, 219, 219, 218, 217, 216, 215, 215, 214, 213,
    212, 211, 210, 210, 209, 208, 207, 206, 205, 204, 204, 203, 202, 201, 200, 199,
    198, 198, 197, 196, 195, 194, 193, 192, 192, 191, 190, 189, 188, 187, 186, 186,
    185, 184, 183, 182, 181, 180, 179, 179, 178, 177, 176, 175, 174, 173, 172, 171,
    171, 170, 169, 168, 167, 166, 165, 164, 163, 163, 162, 161, 160, 159, 158, 157,
    156, 155, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146, 146, 145, 144, 143,
    142, 141, 140, 139, 138, 137, 137, 136, 135, 134, 133, 132, 131, 130, 129, 128
};

/**
 * 功能: TIM4 中断服务函数
 *       这里根据相位 phase 生成对应波形
 *       执行耗时 3 ~ 4 us
 */
void TIM4_IRQHandler(void)
{
    unsigned long section, phase_, a, b, c, t;

    TIM4->SR &= ~TIM_SR_UIF;
    if(!(TIM4->CR1 & TIM_CR1_DIR))
    {
        section = phase / (PHASE_S << 1);
        phase_ = phase - section * (PHASE_S << 1);
        t = (PHASE_S >> 1);
        if(phase_ < t)
        {
            a = wave[t - 1 - phase_];
            b = PWM_MAX - wave[phase_ + t];
            c = PWM_MAX - a;
        }
        else if(phase_ < (PHASE_S + t))
        {
            a = wave[phase_ - t];
            b = wave[PHASE_S + t - phase_ - 1];
            if(phase_ < PHASE_S)
            {
                c = PWM_MAX - a;
            }
            else
            {
                c = PWM_MAX - b;
            }
        }
        else
        {
            a = PWM_MAX - wave[(PHASE_S << 1) + t - phase_ - 1];
            b = wave[phase_ - PHASE_S - t];
            c = PWM_MAX - b;
        }
        a *= m;
        a /= PWM_MAX;
        b *= m;
        b /= PWM_MAX;
        c *= m;
        c /= PWM_MAX;
        if(section == 0)
        {
            TIM4->CCR1 = a;
            TIM4->CCR2 = b;
            TIM4->CCR3 = c;
        }
        else if(section == 1)
        {
            TIM4->CCR1 = c;
            TIM4->CCR2 = a;
            TIM4->CCR3 = b;
        }
        else if(section == 2)
        {
            TIM4->CCR1 = b;
            TIM4->CCR2 = c;
            TIM4->CCR3 = a;
        }
    }
}

/**
 * 功能: 初始化以 TIM4 为基础的 SVPWM
 * 参数: 无
 * 返回: 无
 */
void svpwmInit(void)
{
    GPIOB->CRL = (GPIOB->CRL & ( ~ (GPIO_CRL_CNF6 | GPIO_CRL_CNF7))) | (GPIO_CRL_MODE6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);
    GPIOB->CRH = (GPIOB->CRH & ( ~ GPIO_CRH_CNF8)) | (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1) ;
    //GPIOB->CRH = (GPIOB->CRH & ( ~ (GPIO_CRH_CNF8 | GPIO_CRH_CNF9))) | (GPIO_CRH_MODE8 | GPIO_CRH_MODE9 | GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1);
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CCER = 0;
    TIM4->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM4->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
    //TIM4->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
    TIM4->CCER = TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

    TIM4->PSC = 4; // 分频系数
    TIM4->ARR = PWM_MAX + 10; // "+10" 避免占空比 100% 以适应带自举的栅极驱动器
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;

    TIM4->EGR |= TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * 功能: 设定 SVPWM 相位与幅值
 * 参数: [in] p 相位
 * 参数: [in] p 幅值
 * 返回: 无
 */
void svpwmSet(unsigned long p, unsigned long n)
{
    if(p >= (6 * PHASE_S))
        phase = 0;
    else
        phase = p;
    m = n;
}
