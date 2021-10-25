#ifndef __MEM_H
#define __MEM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/**
 * 功能: 向堆空间申请内存
 * 参数: [in] size 申请的空间大小 (以字节为单位)
 * 返回: 成功申请则返回内存首地址, 否则返回 NULL
 * 注意: 不要越界访问内存, 否则后果不可预料
 */
extern void *smalloc(unsigned long size);

/**
 * 功能: 释放从堆空间申请的内存
 * 参数: [in] pointer 申请的内存首地址
 * 返回: 无
 */
extern void sfree(void *pointer);

/**
 * 功能: 查询堆空间的空闲空间
 * 参数: 无
 * 返回: 空闲空间大小 (以字节为单位)
 */
extern unsigned long getFreeHeapSize(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MEM_H */
