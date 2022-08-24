
#include "dma.h"
#include "stm32f10x.h"

/* 各通道的错误、过半、完成中断 */
static void(*dma1_TEI[7])(void);
static void(*dma2_TEI[5])(void);
static void(*dma1_HTI[7])(void);
static void(*dma2_HTI[5])(void);
static void(*dma1_TCI[7])(void);
static void(*dma2_TCI[5])(void);

/*
 * 功能：配置 DMA 一个通道
 * 参数：cfg-控制寄存器，num-传输量，最大 65535，pAddr-外设地址，mAddr-内存地址，
 *       p_tei-传输错误回调，p_hti-传输过半回调，p_tci-传输完成回调
 * 返回：无
 */
void dma1_set_ch1(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel1->CCR & DMA_CCR1_EN);
	DMA1_Channel1->CNDTR = num;
	DMA1_Channel1->CPAR = pAddr;
	DMA1_Channel1->CMAR = mAddr;
	dma1_TEI[0] = p_tei;
	dma1_HTI[0] = p_hti;
	dma1_TCI[0] = p_tci;
	DMA1_Channel1->CCR = cfg;
}

void dma1_set_ch2(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel2->CCR & DMA_CCR2_EN);
	DMA1_Channel2->CNDTR = num;
	DMA1_Channel2->CPAR = pAddr;
	DMA1_Channel2->CMAR = mAddr;
	dma1_TEI[1] = p_tei;
	dma1_HTI[1] = p_hti;
	dma1_TCI[1] = p_tci;
	DMA1_Channel2->CCR = cfg;
}

void dma1_set_ch3(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel3->CCR & DMA_CCR3_EN);
	DMA1_Channel3->CNDTR = num;
	DMA1_Channel3->CPAR = pAddr;
	DMA1_Channel3->CMAR = mAddr;
	dma1_TEI[2] = p_tei;
	dma1_HTI[2] = p_hti;
	dma1_TCI[2] = p_tci;
	DMA1_Channel3->CCR = cfg;
}

void dma1_set_ch4(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel4->CCR & DMA_CCR4_EN);
	DMA1_Channel4->CNDTR = num;
	DMA1_Channel4->CPAR = pAddr;
	DMA1_Channel4->CMAR = mAddr;
	dma1_TEI[3] = p_tei;
	dma1_HTI[3] = p_hti;
	dma1_TCI[3] = p_tci;
	DMA1_Channel4->CCR = cfg;
}

void dma1_set_ch5(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel5->CCR & DMA_CCR5_EN);
	DMA1_Channel5->CNDTR = num;
	DMA1_Channel5->CPAR = pAddr;
	DMA1_Channel5->CMAR = mAddr;
	dma1_TEI[4] = p_tei;
	dma1_HTI[4] = p_hti;
	dma1_TCI[4] = p_tci;
	DMA1_Channel5->CCR = cfg;
}

void dma1_set_ch6(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel6->CCR & DMA_CCR6_EN);
	DMA1_Channel6->CNDTR = num;
	DMA1_Channel6->CPAR = pAddr;
	DMA1_Channel6->CMAR = mAddr;
	dma1_TEI[5] = p_tei;
	dma1_HTI[5] = p_hti;
	dma1_TCI[5] = p_tci;
	DMA1_Channel6->CCR = cfg;
}

void dma1_set_ch7(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA1_Channel7->CCR & DMA_CCR7_EN);
	DMA1_Channel7->CNDTR = num;
	DMA1_Channel7->CPAR = pAddr;
	DMA1_Channel7->CMAR = mAddr;
	dma1_TEI[6] = p_tei;
	dma1_HTI[6] = p_hti;
	dma1_TCI[6] = p_tci;
	DMA1_Channel7->CCR = cfg;
}

void dma2_set_ch1(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA2_Channel1->CCR & DMA_CCR1_EN);
	DMA2_Channel1->CNDTR = num;
	DMA2_Channel1->CPAR = pAddr;
	DMA2_Channel1->CMAR = mAddr;
	dma2_TEI[0] = p_tei;
	dma2_HTI[0] = p_hti;
	dma2_TCI[0] = p_tci;
	DMA2_Channel1->CCR = cfg;
}

void dma2_set_ch2(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA2_Channel2->CCR & DMA_CCR2_EN);
	DMA2_Channel2->CNDTR = num;
	DMA2_Channel2->CPAR = pAddr;
	DMA2_Channel2->CMAR = mAddr;
	dma2_TEI[1] = p_tei;
	dma2_HTI[1] = p_hti;
	dma2_TCI[1] = p_tci;
	DMA2_Channel2->CCR = cfg;
}

void dma2_set_ch3(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA2_Channel3->CCR & DMA_CCR3_EN);
	DMA2_Channel3->CNDTR = num;
	DMA2_Channel3->CPAR = pAddr;
	DMA2_Channel3->CMAR = mAddr;
	dma2_TEI[2] = p_tei;
	dma2_HTI[2] = p_hti;
	dma2_TCI[2] = p_tci;
	DMA2_Channel3->CCR = cfg;
}

void dma2_set_ch4(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA2_Channel4->CCR & DMA_CCR4_EN);
	DMA2_Channel4->CNDTR = num;
	DMA2_Channel4->CPAR = pAddr;
	DMA2_Channel4->CMAR = mAddr;
	dma2_TEI[3] = p_tei;
	dma2_HTI[3] = p_hti;
	dma2_TCI[3] = p_tci;
	DMA2_Channel4->CCR = cfg;
}

void dma2_set_ch5(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void))
{
	while(DMA2_Channel5->CCR & DMA_CCR5_EN);
	DMA2_Channel5->CNDTR = num;
	DMA2_Channel5->CPAR = pAddr;
	DMA2_Channel5->CMAR = mAddr;
	dma2_TEI[4] = p_tei;
	dma2_HTI[4] = p_hti;
	dma2_TCI[4] = p_tci;
	DMA2_Channel5->CCR = cfg;
}

/*
 * DMA 中断，勿直接调用！
 */
void DMA1_Channel1_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF1) && (DMA1_Channel1->CCR & DMA_CCR1_TEIE))
		dma1_TEI[0]();
	if((DMA1->ISR & DMA_ISR_HTIF1) && (DMA1_Channel1->CCR & DMA_CCR1_HTIE))
		dma1_HTI[0]();
	if((DMA1->ISR & DMA_ISR_TCIF1) && (DMA1_Channel1->CCR & DMA_CCR1_TCIE))
		dma1_TCI[0]();
	DMA1->IFCR |= DMA_IFCR_CGIF1;
}

void DMA1_Channel2_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF2) && (DMA1_Channel2->CCR & DMA_CCR1_TEIE))
		dma1_TEI[1]();
	if((DMA1->ISR & DMA_ISR_HTIF2) && (DMA1_Channel2->CCR & DMA_CCR1_HTIE))
		dma1_HTI[1]();
	if((DMA1->ISR & DMA_ISR_TCIF2) && (DMA1_Channel2->CCR & DMA_CCR1_TCIE))
		dma1_TCI[1]();
	DMA1->IFCR |= DMA_IFCR_CGIF2;
}

void DMA1_Channel3_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF3) && (DMA1_Channel3->CCR & DMA_CCR1_TEIE))
		dma1_TEI[2]();
	if((DMA1->ISR & DMA_ISR_HTIF3) && (DMA1_Channel3->CCR & DMA_CCR1_HTIE))
		dma1_HTI[2]();
	if((DMA1->ISR & DMA_ISR_TCIF3) && (DMA1_Channel3->CCR & DMA_CCR1_TCIE))
		dma1_TCI[2]();
	DMA1->IFCR |= DMA_IFCR_CGIF3;
}

void DMA1_Channel4_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF4) && (DMA1_Channel4->CCR & DMA_CCR1_TEIE))
		dma1_TEI[3]();
	if((DMA1->ISR & DMA_ISR_HTIF4) && (DMA1_Channel4->CCR & DMA_CCR1_HTIE))
		dma1_HTI[3]();
	if((DMA1->ISR & DMA_ISR_TCIF4) && (DMA1_Channel4->CCR & DMA_CCR1_TCIE))
		dma1_TCI[3]();
	DMA1->IFCR |= DMA_IFCR_CGIF4;
}

void DMA1_Channel5_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF5) && (DMA1_Channel5->CCR & DMA_CCR1_TEIE))
		dma1_TEI[4]();
	if((DMA1->ISR & DMA_ISR_HTIF5) && (DMA1_Channel5->CCR & DMA_CCR1_HTIE))
		dma1_HTI[4]();
	if((DMA1->ISR & DMA_ISR_TCIF5) && (DMA1_Channel5->CCR & DMA_CCR1_TCIE))
		dma1_TCI[4]();
	DMA1->IFCR |= DMA_IFCR_CGIF5;
}

void DMA1_Channel6_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF6) && (DMA1_Channel6->CCR & DMA_CCR1_TEIE))
		dma1_TEI[5]();
	if((DMA1->ISR & DMA_ISR_HTIF6) && (DMA1_Channel6->CCR & DMA_CCR1_HTIE))
		dma1_HTI[5]();
	if((DMA1->ISR & DMA_ISR_TCIF6) && (DMA1_Channel6->CCR & DMA_CCR1_TCIE))
		dma1_TCI[5]();
	DMA1->IFCR |= DMA_IFCR_CGIF6;
}

void DMA1_Channel7_IRQHandler(void)
{
	if((DMA1->ISR & DMA_ISR_TEIF7) && (DMA1_Channel7->CCR & DMA_CCR1_TEIE))
		dma1_TEI[6]();
	if((DMA1->ISR & DMA_ISR_HTIF7) && (DMA1_Channel7->CCR & DMA_CCR1_HTIE))
		dma1_HTI[6]();
	if((DMA1->ISR & DMA_ISR_TCIF7) && (DMA1_Channel7->CCR & DMA_CCR1_TCIE))
		dma1_TCI[6]();
	DMA1->IFCR |= DMA_IFCR_CGIF7;
}

void DMA2_Channel1_IRQHandler(void)
{
	if((DMA2->ISR & DMA_ISR_TEIF1) && (DMA2_Channel1->CCR & DMA_CCR2_TEIE))
		dma2_TEI[0]();
	if((DMA2->ISR & DMA_ISR_HTIF1) && (DMA2_Channel1->CCR & DMA_CCR2_HTIE))
		dma2_HTI[0]();
	if((DMA2->ISR & DMA_ISR_TCIF1) && (DMA2_Channel1->CCR & DMA_CCR2_TCIE))
		dma2_TCI[0]();
	DMA2->IFCR |= DMA_IFCR_CGIF1;
}

void DMA2_Channel2_IRQHandler(void)
{
	if((DMA2->ISR & DMA_ISR_TEIF2) && (DMA2_Channel2->CCR & DMA_CCR2_TEIE))
		dma2_TEI[1]();
	if((DMA2->ISR & DMA_ISR_HTIF2) && (DMA2_Channel2->CCR & DMA_CCR2_HTIE))
		dma2_HTI[1]();
	if((DMA2->ISR & DMA_ISR_TCIF2) && (DMA2_Channel2->CCR & DMA_CCR2_TCIE))
		dma2_TCI[1]();
	DMA2->IFCR |= DMA_IFCR_CGIF2;
}

void DMA2_Channel3_IRQHandler(void)
{
	if((DMA2->ISR & DMA_ISR_TEIF3) && (DMA2_Channel3->CCR & DMA_CCR2_TEIE))
		dma2_TEI[2]();
	if((DMA2->ISR & DMA_ISR_HTIF3) && (DMA2_Channel3->CCR & DMA_CCR2_HTIE))
		dma2_HTI[2]();
	if((DMA2->ISR & DMA_ISR_TCIF3) && (DMA2_Channel3->CCR & DMA_CCR2_TCIE))
		dma2_TCI[2]();
	DMA2->IFCR |= DMA_IFCR_CGIF3;
}

void DMA2_Channel4_5_IRQHandler(void)
{
	if(DMA2->ISR & DMA_ISR_GIF4)
	{
		if((DMA2->ISR & DMA_ISR_TEIF4) && (DMA2_Channel4->CCR & DMA_CCR2_TEIE))
			dma2_TEI[3]();
		if((DMA2->ISR & DMA_ISR_HTIF4) && (DMA2_Channel4->CCR & DMA_CCR2_HTIE))
			dma2_HTI[3]();
		if((DMA2->ISR & DMA_ISR_TCIF4) && (DMA2_Channel4->CCR & DMA_CCR2_TCIE))
			dma2_TCI[3]();
		DMA2->IFCR |= DMA_IFCR_CGIF4;
	}
	if(DMA2->ISR & DMA_ISR_GIF5)
	{
		if((DMA2->ISR & DMA_ISR_TEIF5) && (DMA2_Channel5->CCR & DMA_CCR2_TEIE))
			dma2_TEI[4]();
		if((DMA2->ISR & DMA_ISR_HTIF5) && (DMA2_Channel5->CCR & DMA_CCR2_HTIE))
			dma2_HTI[4]();
		if((DMA2->ISR & DMA_ISR_TCIF5) && (DMA2_Channel5->CCR & DMA_CCR2_TCIE))
			dma2_TCI[4]();
		DMA2->IFCR |= DMA_IFCR_CGIF5;
	}
}
