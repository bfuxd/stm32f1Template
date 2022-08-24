#include "critical.h"

// _sheap_a, _eheap_a 定义在 startup_gd32f1x0.s 中
extern const unsigned long _sheap_a;
extern const unsigned long _eheap_a;

// 动态内存分配的范围: 全部未用内存, 请注意保留充足的栈空间
#define MEM_START  _sheap_a
#define MEM_END    _eheap_a

/**
 * 最小延迟分配策略
 * 当定义这个宏后 smalloc 函数将从上次分配找到的空闲块之后寻找
 * 容量足够使用的空闲块, 遇到堆顶则返回堆底寻找, 找完一圈也没有
 * 合适的内存块, 则返回 NULL.
 * 这种分配策略的特点是低延迟, 高碎片化
 * 
 * 注意: 这些内存分配策略中必须选择一种, 且只能选择一种
 */
#define MEM_DELAY_MIN
/**
 * 最小碎片分配策略
 * 当定义这个宏后 smalloc 函数每次都会从堆底向上寻找容量足够使
 * 用的空闲块, 直到堆顶也没有合适的内存块, 则返回 NULL.
 * 这种分配策略的特点是低碎片化, 多次分配后再分配会有较大延迟
 * 
 * 注意: 这些内存分配策略中必须选择一种, 且只能选择一种
 */
//#define MEM_FRAGMETNATION_MIN

typedef struct mem_control_block
{
    unsigned long flg0 :  1; // 有效
    unsigned long flg1 :  1; // 保留
    unsigned long size : 30; // 以字为单位
    struct mem_control_block* prev; // 前块指针
} mcb_t;

static unsigned long freeBytes;
#ifdef MEM_DELAY_MIN
static mcb_t *last;
#endif

void* smalloc(unsigned long size)
{
    static mcb_t *heapStart;
    mcb_t *i, *j, *neighbor;;
    unsigned long primask;
    void *ret;

    if(!size) return (void*)0;
    size = (size + 3) >> 2;
    // 关闭中断, 保证线程安全
    primask = ENTER_CRITICAL();
    // 第一次分配时, 全部空闲内存创建一整个空闲内存块
    if(!heapStart)
    {
        heapStart = (mcb_t*)MEM_START;
        #ifdef MEM_DELAY_MIN
        last = (mcb_t*)MEM_START;
        #endif
        heapStart->size = (MEM_END - (unsigned long)heapStart - sizeof(mcb_t)) >> 2;
        heapStart->flg0 = 0;
        heapStart->prev = (mcb_t*)0;
        freeBytes = MEM_END - MEM_START - sizeof(mcb_t);
    }

    ret = (void*)0;

    #ifdef MEM_FRAGMETNATION_MIN
    // 从头开始寻找一块可用内存块
    for(i = heapStart; (unsigned long)i < MEM_END; i = (void*)i + (i->size << 2) + sizeof(mcb_t))
    {
        if((!i->flg0) && (i->size >= size))
        {
            i->flg0 = 1;
            // 如果剩余的碎片无法管理, 则并入本块, 否则创建一个新的空闲块
            if((i->size - size) <= (sizeof(mcb_t) >> 2))
            {
                freeBytes -= i->size << 2;
            }
            else
            {
                j = (void*)i + (size << 2) + sizeof(mcb_t);
                j->flg0 = 0;
                j->size = i->size - size - (sizeof(mcb_t) >> 2);
                j->prev = i;
                neighbor = (void*)j + (j->size << 2) + sizeof(mcb_t);
                // 如果新块还有后块, 则链接到新块 j
                if((unsigned long)neighbor < MEM_END)
                    neighbor->prev = j;
                i->size = size;
                freeBytes -= (unsigned long)j - (unsigned long)i;
            }
            ret = (void*)i + sizeof(mcb_t);
            break;
        }
    }
    #endif

    #ifdef MEM_DELAY_MIN
    i = last;
    do
    {
        if((!i->flg0) && (i->size >= size))
        {
            i->flg0 = 1;
            // 如果剩余的碎片无法管理, 则并入本块, 否则创建一个新的空闲块
            if((i->size - size) <= (sizeof(mcb_t) >> 2))
            {
                freeBytes -= i->size << 2;
            }
            else
            {
                j = (void*)i + (size << 2) + sizeof(mcb_t);
                j->flg0 = 0;
                j->size = i->size - size - (sizeof(mcb_t) >> 2);
                j->prev = i;
                neighbor = (void*)j + (j->size << 2) + sizeof(mcb_t);
                // 如果新块还有后块, 则链接到新块 j
                if((unsigned long)neighbor < MEM_END)
                    neighbor->prev = j;
                i->size = size;
                freeBytes -= (unsigned long)j - (unsigned long)i;
            }
            ret = (void*)i + sizeof(mcb_t);
            break;
        }
        i = (void*)i + (i->size << 2) + sizeof(mcb_t);
        if((unsigned long)i >= MEM_END)
            i = heapStart;
    }
    while(i != last);
    if(ret)
    {
        last = (void*)i + (i->size << 2) + sizeof(mcb_t);
        if((unsigned long)last >= MEM_END)
            last = heapStart;
    }
    #endif

    // 恢复之前的中断状态
    EXIT_CRITICAL(primask);
    return ret;
}

void sfree(void *pointer)
{
    mcb_t *neighbor;
    unsigned long primask;

    if(!pointer) return;
    pointer -= sizeof(mcb_t);
    // 关闭中断, 保证线程安全
    primask = ENTER_CRITICAL();
    #ifdef MEM_DELAY_MIN
    last = (mcb_t*)pointer;
    #endif
    ((mcb_t*)pointer)->flg0 = 0;
    freeBytes += ((mcb_t*)pointer)->size << 2;
    // 当存在空闲后块时与后块合并
    neighbor = (void*)pointer + (((mcb_t*)pointer)->size << 2) + sizeof(mcb_t);
    if(((unsigned long)neighbor < MEM_END) && (!neighbor->flg0))
    {
        ((mcb_t*)pointer)->size += neighbor->size + (sizeof(mcb_t) >> 2);
        neighbor = (void*)neighbor + (neighbor->size << 2) + sizeof(mcb_t);
        // 如果后块还有后块, 则链接到本块
        if((unsigned long)neighbor < MEM_END)
            neighbor->prev = pointer;
        freeBytes += sizeof(mcb_t);
    }
    // 当存在空闲前块时与前块合并
    neighbor = ((mcb_t*)pointer)->prev;
    if(neighbor && (!neighbor->flg0))
    {
        neighbor->size += ((mcb_t*)pointer)->size + (sizeof(mcb_t) >> 2);
        #ifdef MEM_DELAY_MIN
        last = neighbor;
        #endif
        neighbor = (void*)pointer + (((mcb_t*)pointer)->size << 2) + sizeof(mcb_t);
        // 如果存在后块, 则链接到前块
        if((unsigned long)neighbor < MEM_END)
            neighbor->prev = ((mcb_t*)pointer)->prev;
        freeBytes += sizeof(mcb_t);
    }
    // 恢复之前的中断状态
    EXIT_CRITICAL(primask);
}

unsigned long getFreeHeapSize(void)
{
    return freeBytes;
}
