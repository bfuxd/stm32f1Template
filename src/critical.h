#ifndef __CRITICAL_H
#define __CRITICAL_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/**
 * 功能: 进入临界区
 * 参数: 无
 * 返回: 进入前的状态
 */
extern unsigned long ENTER_CRITICAL(void);

/**
 * 功能: 退出临界区
 * 参数: 上次进入临界区前的状态
 * 返回: 无
 */
extern void EXIT_CRITICAL(unsigned long primask);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CRITICAL_H */
