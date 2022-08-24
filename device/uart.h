#ifndef UART_H_
#define UART_H_

/* 各缓冲区大小，务必 4 字节对齐 */
#define UART1_SEND_TEMP_LEN     128
#define UART4_SEND_TEMP_LEN     128
#define UART1_RECEIVE_TEMP_LEN  128
#define UART4_RECEIVE_TEMP_LEN  128


void uart1_init(void(*p_call)(void *, unsigned int));
void uart1_send(void *data, unsigned int num);
void uart4_init(void(*p_call)(void *, unsigned int));
void uart4_send(void *data, unsigned int num);


#endif
