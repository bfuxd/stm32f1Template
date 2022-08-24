#ifndef DMA_H_
#define DMA_H_

#define dma1_ch1_disable() DMA1_Channel1->CCR &= ~DMA_CCR1_EN
#define dma1_ch2_disable() DMA1_Channel2->CCR &= ~DMA_CCR2_EN
#define dma1_ch3_disable() DMA1_Channel3->CCR &= ~DMA_CCR3_EN
#define dma1_ch4_disable() DMA1_Channel4->CCR &= ~DMA_CCR4_EN
#define dma1_ch5_disable() DMA1_Channel5->CCR &= ~DMA_CCR5_EN
#define dma1_ch6_disable() DMA1_Channel6->CCR &= ~DMA_CCR6_EN
#define dma1_ch7_disable() DMA1_Channel7->CCR &= ~DMA_CCR7_EN
#define dma2_ch1_disable() DMA2_Channel1->CCR &= ~DMA_CCR1_EN
#define dma2_ch2_disable() DMA2_Channel2->CCR &= ~DMA_CCR2_EN
#define dma2_ch3_disable() DMA2_Channel3->CCR &= ~DMA_CCR3_EN
#define dma2_ch4_disable() DMA2_Channel4->CCR &= ~DMA_CCR4_EN
#define dma2_ch5_disable() DMA2_Channel5->CCR &= ~DMA_CCR5_EN

void dma1_set_ch1(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma1_set_ch2(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma1_set_ch3(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma1_set_ch4(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma1_set_ch5(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma1_set_ch6(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma1_set_ch7(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma2_set_ch1(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma2_set_ch2(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma2_set_ch3(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma2_set_ch4(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));
void dma2_set_ch5(unsigned int cfg, unsigned int num, unsigned pAddr, unsigned int mAddr, \
	void(*p_tei)(void), void(*p_hti)(void), void(*p_tci)(void));

#endif
