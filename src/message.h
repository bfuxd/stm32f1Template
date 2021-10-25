#ifndef __MESSAGE_H
#define __MESSAGE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


enum {
    MSG_EINT
};

typedef struct message_t
{
    unsigned long what;
    void *obj;
    struct message_t *next;
} message_t;

/**
 * 功能: 向消息队列发送一条消息
 * 参数: [in, out] queue 指向要操作的消息队列指针的指针
 * 参数: [in] msg 指向消息的指针, 一般通过动态内存分配获取空间
 * 返回: 消息指针
 */
extern void sendMessage(message_t **queue, message_t *msg);

/**
 * 功能: 从消息队列取出一条消息
 * 参数: [in, out] queue 指向要操作的消息队列指针的指针
 * 返回: 消息指针 (如果队列为空则返回 NULL)
 */
extern message_t *Message(message_t **queue);

/**
 * 功能: 从消息队列取出一条消息 (阻塞)
 * 参数: [in, out] queue 指向要操作的消息队列指针的指针
 * 返回: 消息指针
 */
extern message_t *waitMessage(message_t **queue);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MESSAGE_H */
