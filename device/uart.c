
#include "uart.h"
#include "dma.h"
#include "stm32f10x.h"

// ���ڷ��ͻ�����
static unsigned char send_temp1[UART1_SEND_TEMP_LEN];
static unsigned char send_temp4[UART4_SEND_TEMP_LEN];
// ���ڽ��ջ�����
static unsigned char receive_temp1[UART1_RECEIVE_TEMP_LEN];
static unsigned char receive_temp4[UART4_RECEIVE_TEMP_LEN];
// ���ڽ��ջص�ָ��
static void(*uart1_call_back)(void *, unsigned int);
static void(*uart4_call_back)(void *, unsigned int);

/*
 * ���ܣ���ʼ�� UART1
 * ���������ݽ�����ɵĻص�����ָ��
 * ���أ���
 */
void uart1_init(void(*p_call)(void *, unsigned int))
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
	// ע��ص�
	uart1_call_back = p_call;
	// ������ 115.2kbps
	USART1->BRR = (39<<4)|(1);
	USART1->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
	// PA9-10�˿�
	GPIOA->CRH &= 0xFFFFF00F;
	GPIOA->CRH |= GPIO_CRH_CNF9 | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE9;
	GPIOA->ODR |= (1<<10);
}

/*
 * ���ܣ���ʼ�� UART4
 * ���������ݽ�����ɵĻص�����ָ��
 * ���أ���
 */
void uart4_init(void(*p_call)(void *, unsigned int))
{
	// ע��ص�
	uart4_call_back = p_call;
	// ������ 115.2kbps
	UART4->BRR = (19<<4)|(8);
	UART4->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
	// PC10-11�˿�
	GPIOC->CRH &= 0xFFFF00FF;
	GPIOC->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_CNF11_1 | GPIO_CRH_MODE10;
	GPIOC->ODR |= (1<<11);
}

/*
 * ���ܣ����� 1 DMA ����������ɻص������ڼĴ��������ݻ�û����
 * ��������
 * ���أ���
 */
static void uart1_dma_send_complete(void)
{
	USART1->CR3 &= ~USART_CR3_DMAT;
	dma1_ch4_disable();
}

/*
 * ���ܣ�����1��������
 * ������data-Ҫ���͵������׵�ַ��num-���ݳ���(�ֽ�)
 * ���أ���
 */
void uart1_send(void *data, unsigned int num)
{
	unsigned int i;
	if(!num)
		return;
	// �ȴ��ϴη������
	while(!(USART1->SR & USART_SR_TC));
	// �������ݵ�������
	for(i = 0;i < num;i+=4)
		*((unsigned int *)(send_temp1 + i)) = *((unsigned int *)(data + i));
	USART1->CR3 |= USART_CR3_DMAT;
	// ���� DMA
	dma1_set_ch4(DMA_CCR1_MINC | DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN, \
	num, (unsigned int)&USART1->DR, (unsigned int)send_temp1, 0, 0, uart1_dma_send_complete);
}

/*
 * ���ܣ����� 4 DMA ����������ɻص������ڼĴ��������ݻ�û����
 * ��������
 * ���أ���
 */
static void uart4_dma_send_complete(void)
{
	UART4->CR3 &= ~USART_CR3_DMAT;
	dma2_ch5_disable();
}

/*
 * ���ܣ�����1��������
 * ������data-Ҫ���͵������׵�ַ��num-���ݳ���(�ֽ�)
 * ���أ���
 */
void uart4_send(void *data, unsigned int num)
{
	unsigned int i;
	if(!num)
		return;
	// �ȴ��ϴη������
	while(!(UART4->SR & USART_SR_TC));
	// �������ݵ�������
	for(i = 0;i < num;i+=4)
		*((unsigned int *)(send_temp4 + i)) = *((unsigned int *)(data + i));
	UART4->CR3 |= USART_CR3_DMAT;
	// ���� DMA
	dma2_set_ch5(DMA_CCR1_MINC | DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN, \
	num, (unsigned int)&UART4->DR, (unsigned int)send_temp4, 0, 0, uart4_dma_send_complete);
}

/*
 * ���ܣ����� 1 DMA ����������ɻص���������������������δ�Ͽ�
 * ��������
 * ���أ���
 */
static void uart1_dma_receive_complete(void)
{
	// ���ٽ���ʣ������
	USART1->CR3 &= ~USART_CR3_DMAR;
	dma1_ch5_disable();
	USART1->CR1 |= USART_CR1_RXNEIE;
}

/*
 * ���ܣ�����1�жϷ���
 * ��������
 * ���أ���
 */
void USART1_IRQHandler(void)
{
	/* ���ֶ�̬���� DMA �Ľ��ղ���ʮ��Σ��
	 * �������ʱ�̬�ߣ����� 4500000bps����
	 * DMA �������豸ռ��ʱ���нϴ���ܶ�ʧ
	 * �������ݡ�
	 * ʵ�� 2250000bps ʱ����ȷ���ա�������
	 * 4500000bps Ҳ�ǿ��Եġ�
	 * ��֮Ҫ��֤���ڶ� DMA �Ķ�ռ�ԣ����
	 * ����ȫ�жϽ��յķ�ʽʵ�ָù��ܡ�
	 */
	// �� USART_DR �Ķ���������жϱ�־
	if(USART1->SR & USART_SR_RXNE)
	{
		USART1->CR1 &= ~USART_CR1_RXNEIE;
		USART1->CR3 |= USART_CR3_DMAR;
		// ���� DMA
		dma1_set_ch5(DMA_CCR1_MINC | DMA_CCR1_TCIE | DMA_CCR1_EN, UART1_RECEIVE_TEMP_LEN, \
		(unsigned int)&USART1->DR, (unsigned int)receive_temp1, 0, 0, uart1_dma_receive_complete);
	}
	// �ȶ� USART_SR��Ȼ��� USART_DR ����жϱ�־
	if(USART1->SR & USART_SR_IDLE)
	{
		USART1->DR; // ������䲻������жϱ�־
		USART1->CR3 &= ~USART_CR3_DMAR;
		dma1_ch5_disable();
		USART1->CR1 |= USART_CR1_RXNEIE;
		uart1_call_back(receive_temp1, UART1_RECEIVE_TEMP_LEN - DMA1_Channel5->CNDTR);
	}
}

/*
 * ���ܣ����� 4 DMA ����������ɻص���������������������δ�Ͽ�
 * ��������
 * ���أ���
 */
static void uart4_dma_receive_complete(void)
{
	// ���ٽ���ʣ������
	UART4->CR3 &= ~USART_CR3_DMAR;
	dma2_ch3_disable();
	UART4->CR1 |= USART_CR1_RXNEIE;
}

/*
 * ���ܣ�����1�жϷ���
 * ��������
 * ���أ���
 */
void UART4_IRQHandler(void)
{
	/* ���ֶ�̬���� DMA �Ľ��ղ���ʮ��Σ��
	 * �� DMA �������豸ռ��ʱ���нϴ����
	 * ��ʧ�������ݡ�
	 * ��֮Ҫ��֤���ڶ� DMA �Ķ�ռ�ԣ����
	 * ����ȫ�жϽ��յķ�ʽʵ�ָù��ܡ�
	 */
	// �� UART_DR �Ķ���������жϱ�־
	if(UART4->SR & USART_SR_RXNE)
	{
		UART4->CR1 &= ~USART_CR1_RXNEIE;
		UART4->CR3 |= USART_CR3_DMAR;
		// ���� DMA
		dma2_set_ch3(DMA_CCR1_MINC | DMA_CCR1_TCIE | DMA_CCR1_EN, UART4_RECEIVE_TEMP_LEN, \
		(unsigned int)&UART4->DR, (unsigned int)receive_temp4, 0, 0, uart4_dma_receive_complete);
	}
	// �ȶ� UART_SR��Ȼ��� USART_DR ����жϱ�־
	if(UART4->SR & USART_SR_IDLE)
	{
		UART4->DR; // ������䲻������жϱ�־
		UART4->CR3 &= ~USART_CR3_DMAR;
		dma2_ch3_disable();
		UART4->CR1 |= USART_CR1_RXNEIE;
		uart4_call_back(receive_temp4, UART4_RECEIVE_TEMP_LEN - DMA2_Channel3->CNDTR);
	}
}
