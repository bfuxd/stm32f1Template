#include "critical.h"

// _sheap_a, _eheap_a 定义在 startup_stm32f10x.s 中
extern const unsigned long _sheap_a;
extern const unsigned long _eheap_a;

// 动态内存分配的范围: 全部未用内存
#define MEM_START  _sheap_a
#define MEM_END    _eheap_a

typedef struct mem_control_block
{
    unsigned long flg0 :  1; // 有效
    unsigned long flg1 :  1; // 保留
    unsigned long size : 30; // 以字为单位
    struct mem_control_block* prev; // 前块指针
} mcb_t;

static unsigned long freeBytes;

void* smalloc(unsigned long size)
{
    static unsigned long heapStart;
    unsigned long i, j;
    unsigned long primask;
    void *ret;

    if(!size) return (void*)0;
    size = (size + 3) >> 2;
    // 关闭中断, 保证线程安全
    primask = ENTER_CRITICAL();
    // 第一次分配时, 全部空闲内存创建一整个空闲内存块
    if(!heapStart)
    {
        heapStart = MEM_START;
        ((mcb_t*)heapStart)->size = (MEM_END - heapStart - sizeof(mcb_t)) >> 2;
        ((mcb_t*)heapStart)->flg0 = 0;
        ((mcb_t*)heapStart)->prev = 0;
        freeBytes = MEM_END - MEM_START - sizeof(mcb_t);
    }
    // 从头开始寻找一块可用内存块
    for(i = heapStart; i < MEM_END; i += (((mcb_t*)i)->size << 2) + sizeof(mcb_t))
    {
        if((!(((mcb_t*)i)->flg0)) && ((((mcb_t*)i)->size)) >= size)
        {
            j = ((mcb_t*)i)->size - size;
            ((mcb_t*)i)->flg0 = 1;
            // 如果剩余的碎片无法管理, 则并入本块, 否则创建一个新的空闲块
            if(j <= (sizeof(mcb_t) >> 2))
            {
                freeBytes -= (((mcb_t*)i)->size << 2);
                ret = (void*)(i + sizeof(mcb_t));
                break;
            }
            j = i + (size << 2) + sizeof(mcb_t);
            ((mcb_t*)j)->flg0 = 0;
            ((mcb_t*)j)->size = ((mcb_t*)i)->size - size - (sizeof(mcb_t) >> 2);
            ((mcb_t*)j)->prev = (mcb_t*)i;
            ((mcb_t*)i)->size = size;
            freeBytes -= (j - i);
            ret = (void*)(i + sizeof(mcb_t));
            break;
        }
    }
    if(i >= MEM_END)
    {
        // 分配失败
        ret = (void*)0;
    }
    // 恢复之前的中断状态
    EXIT_CRITICAL(primask);
    return ret;
}

void sfree(void *pointer)
{
    unsigned long neighbor;
    unsigned long primask;

    if(!pointer) return;
    pointer -= sizeof(mcb_t);
    // 关闭中断, 保证线程安全
    primask = ENTER_CRITICAL();
    ((mcb_t*)pointer)->flg0 = 0;
    freeBytes += (((mcb_t*)pointer)->size << 2);
    // 与后块合并
    neighbor = (unsigned long)pointer + (((mcb_t*)pointer)->size << 2) + sizeof(mcb_t);
    if(!(((mcb_t*)neighbor)->flg0))
    {
        ((mcb_t*)pointer)->size += ((mcb_t*)neighbor)->size + (sizeof(mcb_t) >> 2);
        ((mcb_t*)(neighbor + (((mcb_t*)neighbor)->size << 2) + sizeof(mcb_t)))->prev = (mcb_t*)pointer;
        freeBytes += sizeof(mcb_t);
    }
    // 与前块合并
    neighbor = (unsigned long)((mcb_t*)pointer)->prev;
    if(neighbor && (!(((mcb_t*)neighbor)->flg0)))
    {
        ((mcb_t*)neighbor)->size += ((mcb_t*)pointer)->size + (sizeof(mcb_t) >> 2);
        ((mcb_t*)(pointer + (((mcb_t*)pointer)->size << 2) + sizeof(mcb_t)))->prev = (mcb_t*)neighbor;
        freeBytes += sizeof(mcb_t);
    }
    // 恢复之前的中断状态
    EXIT_CRITICAL(primask);
}

unsigned long getFreeHeapSize(void)
{
    return freeBytes;
}
