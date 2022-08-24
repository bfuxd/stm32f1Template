
#include "uart.h"
#include "dma.h"
#include "stm32f10x.h"

// 串口发送缓冲区
static unsigned char send_temp1[UART1_SEND_TEMP_LEN];
static unsigned char send_temp4[UART4_SEND_TEMP_LEN];
// 串口接收缓冲区
static unsigned char receive_temp1[UART1_RECEIVE_TEMP_LEN];
static unsigned char receive_temp4[UART4_RECEIVE_TEMP_LEN];
// 串口接收回调指针
static void(*uart1_call_back)(void *, unsigned int);
static void(*uart4_call_back)(void *, unsigned int);

/*
 * 功能：初始化 UART1
 * 参数：数据接收完成的回调函数指针
 * 返回：无
 */
void uart1_init(void(*p_call)(void *, unsigned int))
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
	// 注册回调
	uart1_call_back = p_call;
	// 波特率 115.2kbps
	USART1->BRR = (39<<4)|(1);
	USART1->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
	// PA9-10端口
	GPIOA->CRH &= 0xFFFFF00F;
	GPIOA->CRH |= GPIO_CRH_CNF9 | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE9;
	GPIOA->ODR |= (1<<10);
}

/*
 * 功能：初始化 UART4
 * 参数：数据接收完成的回调函数指针
 * 返回：无
 */
void uart4_init(void(*p_call)(void *, unsigned int))
{
	// 注册回调
	uart4_call_back = p_call;
	// 波特率 115.2kbps
	UART4->BRR = (19<<4)|(8);
	UART4->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
	// PC10-11端口
	GPIOC->CRH &= 0xFFFF00FF;
	GPIOC->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_CNF11_1 | GPIO_CRH_MODE10;
	GPIOC->ODR |= (1<<11);
}

/*
 * 功能：串口 1 DMA 发送数据完成回调，串口寄存器的数据还没发完
 * 参数：无
 * 返回：无
 */
static void uart1_dma_send_complete(void)
{
	USART1->CR3 &= ~USART_CR3_DMAT;
	dma1_ch4_disable();
}

/*
 * 功能：串口1发送数据
 * 参数：data-要发送的数据首地址，num-数据长度(字节)
 * 返回：无
 */
void uart1_send(void *data, unsigned int num)
{
	unsigned int i;
	if(!num)
		return;
	// 等待上次发送完成
	while(!(USART1->SR & USART_SR_TC));
	// 复制数据到缓冲区
	for(i = 0;i < num;i+=4)
		*((unsigned int *)(send_temp1 + i)) = *((unsigned int *)(data + i));
	USART1->CR3 |= USART_CR3_DMAT;
	// 开启 DMA
	dma1_set_ch4(DMA_CCR1_MINC | DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN, \
	num, (unsigned int)&USART1->DR, (unsigned int)send_temp1, 0, 0, uart1_dma_send_complete);
}

/*
 * 功能：串口 4 DMA 发送数据完成回调，串口寄存器的数据还没发完
 * 参数：无
 * 返回：无
 */
static void uart4_dma_send_complete(void)
{
	UART4->CR3 &= ~USART_CR3_DMAT;
	dma2_ch5_disable();
}

/*
 * 功能：串口1发送数据
 * 参数：data-要发送的数据首地址，num-数据长度(字节)
 * 返回：无
 */
void uart4_send(void *data, unsigned int num)
{
	unsigned int i;
	if(!num)
		return;
	// 等待上次发送完成
	while(!(UART4->SR & USART_SR_TC));
	// 复制数据到缓冲区
	for(i = 0;i < num;i+=4)
		*((unsigned int *)(send_temp4 + i)) = *((unsigned int *)(data + i));
	UART4->CR3 |= USART_CR3_DMAT;
	// 开启 DMA
	dma2_set_ch5(DMA_CCR1_MINC | DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN, \
	num, (unsigned int)&UART4->DR, (unsigned int)send_temp4, 0, 0, uart4_dma_send_complete);
}

/*
 * 功能：串口 1 DMA 接收数据完成回调，缓冲区满，数据流仍未断开
 * 参数：无
 * 返回：无
 */
static void uart1_dma_receive_complete(void)
{
	// 不再接收剩余数据
	USART1->CR3 &= ~USART_CR3_DMAR;
	dma1_ch5_disable();
	USART1->CR1 |= USART_CR1_RXNEIE;
}

/*
 * 功能：串口1中断服务
 * 参数：无
 * 返回：无
 */
void USART1_IRQHandler(void)
{
	/* 这种动态分配 DMA 的接收操作十分危险
	 * 当波特率变态高（比如 4500000bps）或
	 * DMA 被其他设备占用时，有较大可能丢失
	 * 部分数据。
	 * 实测 2250000bps 时可正确接收。理论上
	 * 4500000bps 也是可以的。
	 * 总之要保证串口对 DMA 的独占性，或采
	 * 用完全中断接收的方式实现该功能。
	 */
	// 对 USART_DR 的读操作清除中断标志
	if(USART1->SR & USART_SR_RXNE)
	{
		USART1->CR1 &= ~USART_CR1_RXNEIE;
		USART1->CR3 |= USART_CR3_DMAR;
		// 开启 DMA
		dma1_set_ch5(DMA_CCR1_MINC | DMA_CCR1_TCIE | DMA_CCR1_EN, UART1_RECEIVE_TEMP_LEN, \
		(unsigned int)&USART1->DR, (unsigned int)receive_temp1, 0, 0, uart1_dma_receive_complete);
	}
	// 先读 USART_SR，然后读 USART_DR 清除中断标志
	if(USART1->SR & USART_SR_IDLE)
	{
		USART1->DR; // 不加这句不能清除中断标志
		USART1->CR3 &= ~USART_CR3_DMAR;
		dma1_ch5_disable();
		USART1->CR1 |= USART_CR1_RXNEIE;
		uart1_call_back(receive_temp1, UART1_RECEIVE_TEMP_LEN - DMA1_Channel5->CNDTR);
	}
}

/*
 * 功能：串口 4 DMA 接收数据完成回调，缓冲区满，数据流仍未断开
 * 参数：无
 * 返回：无
 */
static void uart4_dma_receive_complete(void)
{
	// 不再接收剩余数据
	UART4->CR3 &= ~USART_CR3_DMAR;
	dma2_ch3_disable();
	UART4->CR1 |= USART_CR1_RXNEIE;
}

/*
 * 功能：串口1中断服务
 * 参数：无
 * 返回：无
 */
void UART4_IRQHandler(void)
{
	/* 这种动态分配 DMA 的接收操作十分危险
	 * 当 DMA 被其他设备占用时，有较大可能
	 * 丢失部分数据。
	 * 总之要保证串口对 DMA 的独占性，或采
	 * 用完全中断接收的方式实现该功能。
	 */
	// 对 UART_DR 的读操作清除中断标志
	if(UART4->SR & USART_SR_RXNE)
	{
		UART4->CR1 &= ~USART_CR1_RXNEIE;
		UART4->CR3 |= USART_CR3_DMAR;
		// 开启 DMA
		dma2_set_ch3(DMA_CCR1_MINC | DMA_CCR1_TCIE | DMA_CCR1_EN, UART4_RECEIVE_TEMP_LEN, \
		(unsigned int)&UART4->DR, (unsigned int)receive_temp4, 0, 0, uart4_dma_receive_complete);
	}
	// 先读 UART_SR，然后读 USART_DR 清除中断标志
	if(UART4->SR & USART_SR_IDLE)
	{
		UART4->DR; // 不加这句不能清除中断标志
		UART4->CR3 &= ~USART_CR3_DMAR;
		dma2_ch3_disable();
		UART4->CR1 |= USART_CR1_RXNEIE;
		uart4_call_back(receive_temp4, UART4_RECEIVE_TEMP_LEN - DMA2_Channel3->CNDTR);
	}
}
