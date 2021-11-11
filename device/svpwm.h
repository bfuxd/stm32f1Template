#ifndef __SVPWM_H
#define __SVPWM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

// 务必与 ./svpwmTools/svpwm.m 参数一致
#define PWM_MAX   255
#define PHASE_S   256

/**
 * 功能: 初始化以 TIM4 为基础的 SVPWM
 * 参数: 无
 * 返回: 无
 */
extern void svpwmInit(void);

/**
 * 功能: 设定 SVPWM 相位与幅值
 * 参数: [in] p 相位
 * 参数: [in] p 幅值
 * 返回: 无
 */
void svpwmSet(unsigned long p, unsigned long n);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SVPWM_H */
