#include "message.h"
#include "critical.h"

void sendMessage(message_t **queue, message_t *msg)
{
    message_t **ppMessage;
    unsigned long primask;

    msg->next = 0;
    for(ppMessage = queue; ; )
    {
        // 关闭中断, 保证线程安全
        primask = ENTER_CRITICAL();
        if(!(*ppMessage))
        {
            (*ppMessage) = msg;
            // 恢复之前的中断状态
            EXIT_CRITICAL(primask);
            break;
        }
        else
        {
            ppMessage = &((*ppMessage)->next);
        }
        // 恢复之前的中断状态
        EXIT_CRITICAL(primask);
    }
}

message_t *getMessage(message_t **queue)
{
    message_t *pMessage;
    unsigned long primask;

    // 关闭中断, 保证线程安全
    primask = ENTER_CRITICAL();
    pMessage = *queue;
    if(pMessage)
        *queue = (*queue)->next;
    // 恢复之前的中断状态
    EXIT_CRITICAL(primask);
    return pMessage;
}

message_t *waitMessage(message_t **queue)
{
    message_t *pMessage;
    unsigned long primask;

GET_MSG:
    // 关闭中断, 保证线程安全
    primask = ENTER_CRITICAL();
    pMessage = *queue;
    if(pMessage)
        *queue = (*queue)->next;
    // 恢复之前的中断状态
    EXIT_CRITICAL(primask);
    if(!pMessage)
    {
        // CPU 等待中断发生
        asm volatile("wfi \n");
        goto GET_MSG;
    }
    return pMessage;
}
