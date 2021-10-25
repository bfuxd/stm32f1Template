#ifndef __STM32F10x_H
#define __STM32F10x_H

#ifdef __cplusplus
 extern "C" {
#endif


#define STM32F10X_HD


#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD) && !defined (STM32F10X_HD_VL) && !defined (STM32F10X_XL) && !defined (STM32F10X_CL)
  /* #define STM32F10X_LD */     /*!< STM32F10X_LD: STM32 Low density devices */
  /* #define STM32F10X_LD_VL */  /*!< STM32F10X_LD_VL: STM32 Low density Value Line devices */
  /* #define STM32F10X_MD */     /*!< STM32F10X_MD: STM32 Medium density devices */
  /* #define STM32F10X_MD_VL */  /*!< STM32F10X_MD_VL: STM32 Medium density Value Line devices */
  /* #define STM32F10X_HD */     /*!< STM32F10X_HD: STM32 High density devices */
  /* #define STM32F10X_HD_VL */  /*!< STM32F10X_HD_VL: STM32 High density value line devices */
  /* #define STM32F10X_XL */     /*!< STM32F10X_XL: STM32 XL-density devices */
  /* #define STM32F10X_CL */     /*!< STM32F10X_CL: STM32 Connectivity line devices */
#endif
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.

 - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
   where the Flash memory density ranges between 16 and 32 Kbytes.
 - Low-density value line devices are STM32F100xx microcontrollers where the Flash
   memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
   where the Flash memory density ranges between 64 and 128 Kbytes.
 - Medium-density value line devices are STM32F100xx microcontrollers where the
   Flash memory density ranges between 64 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - High-density value line devices are STM32F100xx microcontrollers where the
   Flash memory density ranges between 256 and 512 Kbytes.
 - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
  */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD) && !defined (STM32F10X_HD_VL) && !defined (STM32F10X_XL) && !defined (STM32F10X_CL)
 #error "Please select first the target STM32F10x device used in your application (in stm32f10x.h file)"
#endif

#if !defined  USE_STDPERIPH_DRIVER
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
  /*#define USE_STDPERIPH_DRIVER*/
#endif

/**
 * @brief In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application

   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */
#if !defined  HSE_VALUE
 #ifdef STM32F10X_CL
  #define HSE_VALUE    ((unsigned long)25000000) /*!< Value of the External oscillator in Hz */
 #else
  #define HSE_VALUE    ((unsigned long)8000000) /*!< Value of the External oscillator in Hz */
 #endif /* STM32F10X_CL */
#endif /* HSE_VALUE */


/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup
   Timeout value
   */
#define HSE_STARTUP_TIMEOUT   ((unsigned short)0x0500) /*!< Time out for HSE start up */

#define HSI_VALUE    ((unsigned long)8000000) /*!< Value of the Internal oscillator in Hz*/

/**
 * @brief STM32F10x Standard Peripheral Library version number
   */
#define __STM32F10X_STDPERIPH_VERSION_MAIN   (0x03) /*!< [31:24] main version */
#define __STM32F10X_STDPERIPH_VERSION_SUB1   (0x05) /*!< [23:16] sub1 version */
#define __STM32F10X_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32F10X_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F10X_STDPERIPH_VERSION       ( (__STM32F10X_STDPERIPH_VERSION_MAIN << 24)\
                                             |(__STM32F10X_STDPERIPH_VERSION_SUB1 << 16)\
                                             |(__STM32F10X_STDPERIPH_VERSION_SUB2 << 8)\
                                             |(__STM32F10X_STDPERIPH_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
 * @brief Configuration of the Cortex-M3 Processor and Core Peripherals
 */
#ifdef STM32F10X_XL
 #define __MPU_PRESENT             1 /*!< STM32 XL-density devices provide an MPU */
#else
 #define __MPU_PRESENT             0 /*!< Other STM32 devices does not provide an MPU */
#endif /* STM32F10X_XL */
#define __NVIC_PRIO_BITS          4 /*!< STM32 uses 4 Bits for the Priority Levels    */
#define __Vendor_SysTickConfig    0 /*!< Set to 1 if different SysTick Config is used */

/**
 * @brief STM32F10x Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

/******  STM32 specific Interrupt Numbers *********************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
  TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
  RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                      */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                      */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                      */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                      */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                      */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                      */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                      */

#ifdef STM32F10X_LD
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42      /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
#endif /* STM32F10X_LD */

#ifdef STM32F10X_LD_VL
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn                    = 42,     /*!< HDMI-CEC Interrupt                                   */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn                   = 55      /*!< TIM7 Interrupt                                       */
#endif /* STM32F10X_LD_VL */

#ifdef STM32F10X_MD
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42      /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
#endif /* STM32F10X_MD */

#ifdef STM32F10X_MD_VL
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn                    = 42,     /*!< HDMI-CEC Interrupt                                   */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn                   = 55      /*!< TIM7 Interrupt                                       */
#endif /* STM32F10X_MD_VL */

#ifdef STM32F10X_HD
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                 */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt               */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                       */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn        = 59      /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
#endif /* STM32F10X_HD */

#ifdef STM32F10X_HD_VL
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn                    = 42,     /*!< HDMI-CEC Interrupt                                   */
  TIM12_IRQn                  = 43,     /*!< TIM12 global Interrupt                               */
  TIM13_IRQn                  = 44,     /*!< TIM13 global Interrupt                               */
  TIM14_IRQn                  = 45,     /*!< TIM14 global Interrupt                               */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn                   = 55,     /*!< TIM7 Interrupt                                       */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
  DMA2_Channel5_IRQn          = 60      /*!< DMA2 Channel 5 global Interrupt (DMA2 Channel 5 is
                                             mapped at position 60 only if the MISC_REMAP bit in
                                             the AFIO_MAPR2 register is set)                      */
#endif /* STM32F10X_HD_VL */

#ifdef STM32F10X_XL
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break Interrupt and TIM9 global Interrupt       */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global Interrupt     */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global Interrupt      */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global Interrupt     */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                       */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn        = 59      /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
#endif /* STM32F10X_XL */

#ifdef STM32F10X_CL
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  CAN1_TX_IRQn                = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  CAN1_RX0_IRQn               = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS WakeUp from suspend through EXTI Line Interrupt */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                      */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                      */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                            */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt          */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                    */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                   */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                   */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                   */
  OTG_FS_IRQn                 = 67      /*!< USB OTG FS global Interrupt                          */
#endif /* STM32F10X_CL */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm3.h"

/** @addtogroup Exported_types
  * @{
  */


typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/*!< STM32F10x Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSEStartUp_TimeOut   HSE_STARTUP_TIMEOUT
#define HSE_Value            HSE_VALUE
#define HSI_Value            HSI_VALUE
/**
  * @}
  */

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  volatile unsigned long SR;
  volatile unsigned long CR1;
  volatile unsigned long CR2;
  volatile unsigned long SMPR1;
  volatile unsigned long SMPR2;
  volatile unsigned long JOFR1;
  volatile unsigned long JOFR2;
  volatile unsigned long JOFR3;
  volatile unsigned long JOFR4;
  volatile unsigned long HTR;
  volatile unsigned long LTR;
  volatile unsigned long SQR1;
  volatile unsigned long SQR2;
  volatile unsigned long SQR3;
  volatile unsigned long JSQR;
  volatile unsigned long JDR1;
  volatile unsigned long JDR2;
  volatile unsigned long JDR3;
  volatile unsigned long JDR4;
  volatile unsigned long DR;
} ADC_TypeDef;

/**
  * @brief Backup Registers
  */

typedef struct
{
  unsigned long  RESERVED0;
  volatile unsigned short DR1;
  unsigned short  RESERVED1;
  volatile unsigned short DR2;
  unsigned short  RESERVED2;
  volatile unsigned short DR3;
  unsigned short  RESERVED3;
  volatile unsigned short DR4;
  unsigned short  RESERVED4;
  volatile unsigned short DR5;
  unsigned short  RESERVED5;
  volatile unsigned short DR6;
  unsigned short  RESERVED6;
  volatile unsigned short DR7;
  unsigned short  RESERVED7;
  volatile unsigned short DR8;
  unsigned short  RESERVED8;
  volatile unsigned short DR9;
  unsigned short  RESERVED9;
  volatile unsigned short DR10;
  unsigned short  RESERVED10;
  volatile unsigned short RTCCR;
  unsigned short  RESERVED11;
  volatile unsigned short CR;
  unsigned short  RESERVED12;
  volatile unsigned short CSR;
  unsigned short  RESERVED13[5];
  volatile unsigned short DR11;
  unsigned short  RESERVED14;
  volatile unsigned short DR12;
  unsigned short  RESERVED15;
  volatile unsigned short DR13;
  unsigned short  RESERVED16;
  volatile unsigned short DR14;
  unsigned short  RESERVED17;
  volatile unsigned short DR15;
  unsigned short  RESERVED18;
  volatile unsigned short DR16;
  unsigned short  RESERVED19;
  volatile unsigned short DR17;
  unsigned short  RESERVED20;
  volatile unsigned short DR18;
  unsigned short  RESERVED21;
  volatile unsigned short DR19;
  unsigned short  RESERVED22;
  volatile unsigned short DR20;
  unsigned short  RESERVED23;
  volatile unsigned short DR21;
  unsigned short  RESERVED24;
  volatile unsigned short DR22;
  unsigned short  RESERVED25;
  volatile unsigned short DR23;
  unsigned short  RESERVED26;
  volatile unsigned short DR24;
  unsigned short  RESERVED27;
  volatile unsigned short DR25;
  unsigned short  RESERVED28;
  volatile unsigned short DR26;
  unsigned short  RESERVED29;
  volatile unsigned short DR27;
  unsigned short  RESERVED30;
  volatile unsigned short DR28;
  unsigned short  RESERVED31;
  volatile unsigned short DR29;
  unsigned short  RESERVED32;
  volatile unsigned short DR30;
  unsigned short  RESERVED33;
  volatile unsigned short DR31;
  unsigned short  RESERVED34;
  volatile unsigned short DR32;
  unsigned short  RESERVED35;
  volatile unsigned short DR33;
  unsigned short  RESERVED36;
  volatile unsigned short DR34;
  unsigned short  RESERVED37;
  volatile unsigned short DR35;
  unsigned short  RESERVED38;
  volatile unsigned short DR36;
  unsigned short  RESERVED39;
  volatile unsigned short DR37;
  unsigned short  RESERVED40;
  volatile unsigned short DR38;
  unsigned short  RESERVED41;
  volatile unsigned short DR39;
  unsigned short  RESERVED42;
  volatile unsigned short DR40;
  unsigned short  RESERVED43;
  volatile unsigned short DR41;
  unsigned short  RESERVED44;
  volatile unsigned short DR42;
  unsigned short  RESERVED45;
} BKP_TypeDef;

/**
  * @brief Controller Area Network TxMailBox
  */

typedef struct
{
  volatile unsigned long TIR;
  volatile unsigned long TDTR;
  volatile unsigned long TDLR;
  volatile unsigned long TDHR;
} CAN_TxMailBox_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */

typedef struct
{
  volatile unsigned long RIR;
  volatile unsigned long RDTR;
  volatile unsigned long RDLR;
  volatile unsigned long RDHR;
} CAN_FIFOMailBox_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */

typedef struct
{
  volatile unsigned long FR1;
  volatile unsigned long FR2;
} CAN_FilterRegister_TypeDef;

/**
  * @brief Controller Area Network
  */

typedef struct
{
  volatile unsigned long MCR;
  volatile unsigned long MSR;
  volatile unsigned long TSR;
  volatile unsigned long RF0R;
  volatile unsigned long RF1R;
  volatile unsigned long IER;
  volatile unsigned long ESR;
  volatile unsigned long BTR;
  unsigned long  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  unsigned long  RESERVED1[12];
  volatile unsigned long FMR;
  volatile unsigned long FM1R;
  unsigned long  RESERVED2;
  volatile unsigned long FS1R;
  unsigned long  RESERVED3;
  volatile unsigned long FFA1R;
  unsigned long  RESERVED4;
  volatile unsigned long FA1R;
  unsigned long  RESERVED5[8];
#ifndef STM32F10X_CL
  CAN_FilterRegister_TypeDef sFilterRegister[14];
#else
  CAN_FilterRegister_TypeDef sFilterRegister[28];
#endif /* STM32F10X_CL */
} CAN_TypeDef;

/**
  * @brief Consumer Electronics Control (CEC)
  */
typedef struct
{
  volatile unsigned long CFGR;
  volatile unsigned long OAR;
  volatile unsigned long PRES;
  volatile unsigned long ESR;
  volatile unsigned long CSR;
  volatile unsigned long TXD;
  volatile unsigned long RXD;
} CEC_TypeDef;

/**
  * @brief CRC calculation unit
  */

typedef struct
{
  volatile unsigned long DR;
  volatile unsigned char  IDR;
  unsigned char   RESERVED0;
  unsigned short  RESERVED1;
  volatile unsigned long CR;
} CRC_TypeDef;

/**
  * @brief Digital to Analog Converter
  */

typedef struct
{
  volatile unsigned long CR;
  volatile unsigned long SWTRIGR;
  volatile unsigned long DHR12R1;
  volatile unsigned long DHR12L1;
  volatile unsigned long DHR8R1;
  volatile unsigned long DHR12R2;
  volatile unsigned long DHR12L2;
  volatile unsigned long DHR8R2;
  volatile unsigned long DHR12RD;
  volatile unsigned long DHR12LD;
  volatile unsigned long DHR8RD;
  volatile unsigned long DOR1;
  volatile unsigned long DOR2;
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
  volatile unsigned long SR;
#endif
} DAC_TypeDef;

/**
  * @brief Debug MCU
  */

typedef struct
{
  volatile unsigned long IDCODE;
  volatile unsigned long CR;	
}DBGMCU_TypeDef;

/**
  * @brief DMA Controller
  */

typedef struct
{
  volatile unsigned long CCR;
  volatile unsigned long CNDTR;
  volatile unsigned long CPAR;
  volatile unsigned long CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile unsigned long ISR;
  volatile unsigned long IFCR;
} DMA_TypeDef;

/**
  * @brief Ethernet MAC
  */

typedef struct
{
  volatile unsigned long MACCR;
  volatile unsigned long MACFFR;
  volatile unsigned long MACHTHR;
  volatile unsigned long MACHTLR;
  volatile unsigned long MACMIIAR;
  volatile unsigned long MACMIIDR;
  volatile unsigned long MACFCR;
  volatile unsigned long MACVLANTR;             /*    8 */
  unsigned long RESERVED0[2];
  volatile unsigned long MACRWUFFR;             /*   11 */
  volatile unsigned long MACPMTCSR;
  unsigned long RESERVED1[2];
  volatile unsigned long MACSR;                 /*   15 */
  volatile unsigned long MACIMR;
  volatile unsigned long MACA0HR;
  volatile unsigned long MACA0LR;
  volatile unsigned long MACA1HR;
  volatile unsigned long MACA1LR;
  volatile unsigned long MACA2HR;
  volatile unsigned long MACA2LR;
  volatile unsigned long MACA3HR;
  volatile unsigned long MACA3LR;               /*   24 */
  unsigned long RESERVED2[40];
  volatile unsigned long MMCCR;                 /*   65 */
  volatile unsigned long MMCRIR;
  volatile unsigned long MMCTIR;
  volatile unsigned long MMCRIMR;
  volatile unsigned long MMCTIMR;               /*   69 */
  unsigned long RESERVED3[14];
  volatile unsigned long MMCTGFSCCR;            /*   84 */
  volatile unsigned long MMCTGFMSCCR;
  unsigned long RESERVED4[5];
  volatile unsigned long MMCTGFCR;
  unsigned long RESERVED5[10];
  volatile unsigned long MMCRFCECR;
  volatile unsigned long MMCRFAECR;
  unsigned long RESERVED6[10];
  volatile unsigned long MMCRGUFCR;
  unsigned long RESERVED7[334];
  volatile unsigned long PTPTSCR;
  volatile unsigned long PTPSSIR;
  volatile unsigned long PTPTSHR;
  volatile unsigned long PTPTSLR;
  volatile unsigned long PTPTSHUR;
  volatile unsigned long PTPTSLUR;
  volatile unsigned long PTPTSAR;
  volatile unsigned long PTPTTHR;
  volatile unsigned long PTPTTLR;
  unsigned long RESERVED8[567];
  volatile unsigned long DMABMR;
  volatile unsigned long DMATPDR;
  volatile unsigned long DMARPDR;
  volatile unsigned long DMARDLAR;
  volatile unsigned long DMATDLAR;
  volatile unsigned long DMASR;
  volatile unsigned long DMAOMR;
  volatile unsigned long DMAIER;
  volatile unsigned long DMAMFBOCR;
  unsigned long RESERVED9[9];
  volatile unsigned long DMACHTDR;
  volatile unsigned long DMACHRDR;
  volatile unsigned long DMACHTBAR;
  volatile unsigned long DMACHRBAR;
} ETH_TypeDef;

/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  volatile unsigned long IMR;
  volatile unsigned long EMR;
  volatile unsigned long RTSR;
  volatile unsigned long FTSR;
  volatile unsigned long SWIER;
  volatile unsigned long PR;
} EXTI_TypeDef;

/**
  * @brief FLASH Registers
  */

typedef struct
{
  volatile unsigned long ACR;
  volatile unsigned long KEYR;
  volatile unsigned long OPTKEYR;
  volatile unsigned long SR;
  volatile unsigned long CR;
  volatile unsigned long AR;
  volatile unsigned long RESERVED;
  volatile unsigned long OBR;
  volatile unsigned long WRPR;
#ifdef STM32F10X_XL
  unsigned long RESERVED1[8];
  volatile unsigned long KEYR2;
  unsigned long RESERVED2;
  volatile unsigned long SR2;
  volatile unsigned long CR2;
  volatile unsigned long AR2;
#endif /* STM32F10X_XL */
} FLASH_TypeDef;

/**
  * @brief Option Bytes Registers
  */

typedef struct
{
  volatile unsigned short RDP;
  volatile unsigned short USER;
  volatile unsigned short Data0;
  volatile unsigned short Data1;
  volatile unsigned short WRP0;
  volatile unsigned short WRP1;
  volatile unsigned short WRP2;
  volatile unsigned short WRP3;
} OB_TypeDef;

/**
  * @brief Flexible Static Memory Controller
  */

typedef struct
{
  volatile unsigned long BTCR[8];
} FSMC_Bank1_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank1E
  */

typedef struct
{
  volatile unsigned long BWTR[7];
} FSMC_Bank1E_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank2
  */

typedef struct
{
  volatile unsigned long PCR2;
  volatile unsigned long SR2;
  volatile unsigned long PMEM2;
  volatile unsigned long PATT2;
  unsigned long  RESERVED0;
  volatile unsigned long ECCR2;
} FSMC_Bank2_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank3
  */

typedef struct
{
  volatile unsigned long PCR3;
  volatile unsigned long SR3;
  volatile unsigned long PMEM3;
  volatile unsigned long PATT3;
  unsigned long  RESERVED0;
  volatile unsigned long ECCR3;
} FSMC_Bank3_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank4
  */

typedef struct
{
  volatile unsigned long PCR4;
  volatile unsigned long SR4;
  volatile unsigned long PMEM4;
  volatile unsigned long PATT4;
  volatile unsigned long PIO4;
} FSMC_Bank4_TypeDef;

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  volatile unsigned long CRL;
  volatile unsigned long CRH;
  volatile unsigned long IDR;
  volatile unsigned long ODR;
  volatile unsigned long BSRR;
  volatile unsigned long BRR;
  volatile unsigned long LCKR;
} GPIO_TypeDef;

/**
  * @brief Alternate Function I/O
  */

typedef struct
{
  volatile unsigned long EVCR;
  volatile unsigned long MAPR;
  volatile unsigned long EXTICR[4];
  unsigned long RESERVED0;
  volatile unsigned long MAPR2;
} AFIO_TypeDef;
/**
  * @brief Inter Integrated Circuit Interface
  */

typedef struct
{
  volatile unsigned short CR1;
  unsigned short  RESERVED0;
  volatile unsigned short CR2;
  unsigned short  RESERVED1;
  volatile unsigned short OAR1;
  unsigned short  RESERVED2;
  volatile unsigned short OAR2;
  unsigned short  RESERVED3;
  volatile unsigned short DR;
  unsigned short  RESERVED4;
  volatile unsigned short SR1;
  unsigned short  RESERVED5;
  volatile unsigned short SR2;
  unsigned short  RESERVED6;
  volatile unsigned short CCR;
  unsigned short  RESERVED7;
  volatile unsigned short TRISE;
  unsigned short  RESERVED8;
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */

typedef struct
{
  volatile unsigned long KR;
  volatile unsigned long PR;
  volatile unsigned long RLR;
  volatile unsigned long SR;
} IWDG_TypeDef;

/**
  * @brief Power Control
  */

typedef struct
{
  volatile unsigned long CR;
  volatile unsigned long CSR;
} PWR_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  volatile unsigned long CR;
  volatile unsigned long CFGR;
  volatile unsigned long CIR;
  volatile unsigned long APB2RSTR;
  volatile unsigned long APB1RSTR;
  volatile unsigned long AHBENR;
  volatile unsigned long APB2ENR;
  volatile unsigned long APB1ENR;
  volatile unsigned long BDCR;
  volatile unsigned long CSR;

#ifdef STM32F10X_CL
  volatile unsigned long AHBRSTR;
  volatile unsigned long CFGR2;
#endif /* STM32F10X_CL */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
  unsigned long RESERVED0;
  volatile unsigned long CFGR2;
#endif /* STM32F10X_LD_VL || STM32F10X_MD_VL || STM32F10X_HD_VL */
} RCC_TypeDef;

/**
  * @brief Real-Time Clock
  */

typedef struct
{
  volatile unsigned short CRH;
  unsigned short  RESERVED0;
  volatile unsigned short CRL;
  unsigned short  RESERVED1;
  volatile unsigned short PRLH;
  unsigned short  RESERVED2;
  volatile unsigned short PRLL;
  unsigned short  RESERVED3;
  volatile unsigned short DIVH;
  unsigned short  RESERVED4;
  volatile unsigned short DIVL;
  unsigned short  RESERVED5;
  volatile unsigned short CNTH;
  unsigned short  RESERVED6;
  volatile unsigned short CNTL;
  unsigned short  RESERVED7;
  volatile unsigned short ALRH;
  unsigned short  RESERVED8;
  volatile unsigned short ALRL;
  unsigned short  RESERVED9;
} RTC_TypeDef;

/**
  * @brief SD host Interface
  */

typedef struct
{
  volatile unsigned long POWER;
  volatile unsigned long CLKCR;
  volatile unsigned long ARG;
  volatile unsigned long CMD;
  volatile const unsigned long RESPCMD;
  volatile const unsigned long RESP1;
  volatile const unsigned long RESP2;
  volatile const unsigned long RESP3;
  volatile const unsigned long RESP4;
  volatile unsigned long DTIMER;
  volatile unsigned long DLEN;
  volatile unsigned long DCTRL;
  volatile const unsigned long DCOUNT;
  volatile const unsigned long STA;
  volatile unsigned long ICR;
  volatile unsigned long MASK;
  unsigned long  RESERVED0[2];
  volatile const unsigned long FIFOCNT;
  unsigned long  RESERVED1[13];
  volatile unsigned long FIFO;
} SDIO_TypeDef;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  volatile unsigned short CR1;
  unsigned short  RESERVED0;
  volatile unsigned short CR2;
  unsigned short  RESERVED1;
  volatile unsigned short SR;
  unsigned short  RESERVED2;
  volatile unsigned short DR;
  unsigned short  RESERVED3;
  volatile unsigned short CRCPR;
  unsigned short  RESERVED4;
  volatile unsigned short RXCRCR;
  unsigned short  RESERVED5;
  volatile unsigned short TXCRCR;
  unsigned short  RESERVED6;
  volatile unsigned short I2SCFGR;
  unsigned short  RESERVED7;
  volatile unsigned short I2SPR;
  unsigned short  RESERVED8;
} SPI_TypeDef;

/**
  * @brief TIM
  */

typedef struct
{
  volatile unsigned short CR1;
  unsigned short  RESERVED0;
  volatile unsigned short CR2;
  unsigned short  RESERVED1;
  volatile unsigned short SMCR;
  unsigned short  RESERVED2;
  volatile unsigned short DIER;
  unsigned short  RESERVED3;
  volatile unsigned short SR;
  unsigned short  RESERVED4;
  volatile unsigned short EGR;
  unsigned short  RESERVED5;
  volatile unsigned short CCMR1;
  unsigned short  RESERVED6;
  volatile unsigned short CCMR2;
  unsigned short  RESERVED7;
  volatile unsigned short CCER;
  unsigned short  RESERVED8;
  volatile unsigned short CNT;
  unsigned short  RESERVED9;
  volatile unsigned short PSC;
  unsigned short  RESERVED10;
  volatile unsigned short ARR;
  unsigned short  RESERVED11;
  volatile unsigned short RCR;
  unsigned short  RESERVED12;
  volatile unsigned short CCR1;
  unsigned short  RESERVED13;
  volatile unsigned short CCR2;
  unsigned short  RESERVED14;
  volatile unsigned short CCR3;
  unsigned short  RESERVED15;
  volatile unsigned short CCR4;
  unsigned short  RESERVED16;
  volatile unsigned short BDTR;
  unsigned short  RESERVED17;
  volatile unsigned short DCR;
  unsigned short  RESERVED18;
  volatile unsigned short DMAR;
  unsigned short  RESERVED19;
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  volatile unsigned short SR;
  unsigned short  RESERVED0;
  volatile unsigned short DR;
  unsigned short  RESERVED1;
  volatile unsigned short BRR;
  unsigned short  RESERVED2;
  volatile unsigned short CR1;
  unsigned short  RESERVED3;
  volatile unsigned short CR2;
  unsigned short  RESERVED4;
  volatile unsigned short CR3;
  unsigned short  RESERVED5;
  volatile unsigned short GTPR;
  unsigned short  RESERVED6;
} USART_TypeDef;

/**
  * @brief Window WATCHDOG
  */

typedef struct
{
  volatile unsigned long CR;
  volatile unsigned long CFR;
  volatile unsigned long SR;
} WWDG_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */


#define FLASH_BASE            ((unsigned long)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((unsigned long)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((unsigned long)0x40000000) /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          ((unsigned long)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((unsigned long)0x42000000) /*!< Peripheral base address in the bit-band region */

#define FSMC_R_BASE           ((unsigned long)0xA0000000) /*!< FSMC registers base address */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)
#define CEC_BASE              (APB1PERIPH_BASE + 0x7800)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x3400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM15_BASE            (APB2PERIPH_BASE + 0x4000)
#define TIM16_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM17_BASE            (APB2PERIPH_BASE + 0x4800)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4C00)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x5000)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x5400)

#define SDIO_BASE             (PERIPH_BASE + 0x18000)

#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x0080)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x0408)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x041C)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x0430)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x0444)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x0458)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)

#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000) /*!< Flash registers base address */
#define OB_BASE               ((unsigned long)0x1FFFF800)    /*!< Flash Option Bytes base address */

#define ETH_BASE              (AHBPERIPH_BASE + 0x8000)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000)

#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000) /*!< FSMC Bank1 registers base address */
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104) /*!< FSMC Bank1E registers base address */
#define FSMC_Bank2_R_BASE     (FSMC_R_BASE + 0x0060) /*!< FSMC Bank2 registers base address */
#define FSMC_Bank3_R_BASE     (FSMC_R_BASE + 0x0080) /*!< FSMC Bank3 registers base address */
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0) /*!< FSMC Bank4 registers base address */

#define DBGMCU_BASE          ((unsigned long)0xE0042000) /*!< Debug MCU registers base address */

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */

#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#define BKP                 ((BKP_TypeDef *) BKP_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define CEC                 ((CEC_TypeDef *) CEC_BASE)
#define AFIO                ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TIM17               ((TIM_TypeDef *) TIM17_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE)
#define ETH                 ((ETH_TypeDef *) ETH_BASE)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2          ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
#define FSMC_Bank3          ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((unsigned long)0xFFFFFFFF) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((unsigned char)0xFF)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((unsigned char)0x01)        /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((unsigned short)0x0001)     /*!< Low-Power Deepsleep */
#define  PWR_CR_PDDS                         ((unsigned short)0x0002)     /*!< Power Down Deepsleep */
#define  PWR_CR_CWUF                         ((unsigned short)0x0004)     /*!< Clear Wakeup Flag */
#define  PWR_CR_CSBF                         ((unsigned short)0x0008)     /*!< Clear Standby Flag */
#define  PWR_CR_PVDE                         ((unsigned short)0x0010)     /*!< Power Voltage Detector Enable */

#define  PWR_CR_PLS                          ((unsigned short)0x00E0)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((unsigned short)0x0020)     /*!< Bit 0 */
#define  PWR_CR_PLS_1                        ((unsigned short)0x0040)     /*!< Bit 1 */
#define  PWR_CR_PLS_2                        ((unsigned short)0x0080)     /*!< Bit 2 */

/*!< PVD level configuration */
#define  PWR_CR_PLS_2V2                      ((unsigned short)0x0000)     /*!< PVD level 2.2V */
#define  PWR_CR_PLS_2V3                      ((unsigned short)0x0020)     /*!< PVD level 2.3V */
#define  PWR_CR_PLS_2V4                      ((unsigned short)0x0040)     /*!< PVD level 2.4V */
#define  PWR_CR_PLS_2V5                      ((unsigned short)0x0060)     /*!< PVD level 2.5V */
#define  PWR_CR_PLS_2V6                      ((unsigned short)0x0080)     /*!< PVD level 2.6V */
#define  PWR_CR_PLS_2V7                      ((unsigned short)0x00A0)     /*!< PVD level 2.7V */
#define  PWR_CR_PLS_2V8                      ((unsigned short)0x00C0)     /*!< PVD level 2.8V */
#define  PWR_CR_PLS_2V9                      ((unsigned short)0x00E0)     /*!< PVD level 2.9V */

#define  PWR_CR_DBP                          ((unsigned short)0x0100)     /*!< Disable Backup Domain write protection */


/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((unsigned short)0x0001)     /*!< Wakeup Flag */
#define  PWR_CSR_SBF                         ((unsigned short)0x0002)     /*!< Standby Flag */
#define  PWR_CSR_PVDO                        ((unsigned short)0x0004)     /*!< PVD Output */
#define  PWR_CSR_EWUP                        ((unsigned short)0x0100)     /*!< Enable WKUP pin */

/******************************************************************************/
/*                                                                            */
/*                            Backup registers                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for BKP_DR1 register  ********************/
#define  BKP_DR1_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR2 register  ********************/
#define  BKP_DR2_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR3 register  ********************/
#define  BKP_DR3_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR4 register  ********************/
#define  BKP_DR4_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR5 register  ********************/
#define  BKP_DR5_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR6 register  ********************/
#define  BKP_DR6_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR7 register  ********************/
#define  BKP_DR7_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR8 register  ********************/
#define  BKP_DR8_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR9 register  ********************/
#define  BKP_DR9_D                           ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR10 register  *******************/
#define  BKP_DR10_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR11 register  *******************/
#define  BKP_DR11_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR12 register  *******************/
#define  BKP_DR12_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR13 register  *******************/
#define  BKP_DR13_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR14 register  *******************/
#define  BKP_DR14_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR15 register  *******************/
#define  BKP_DR15_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR16 register  *******************/
#define  BKP_DR16_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR17 register  *******************/
#define  BKP_DR17_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/******************  Bit definition for BKP_DR18 register  ********************/
#define  BKP_DR18_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR19 register  *******************/
#define  BKP_DR19_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR20 register  *******************/
#define  BKP_DR20_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR21 register  *******************/
#define  BKP_DR21_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR22 register  *******************/
#define  BKP_DR22_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR23 register  *******************/
#define  BKP_DR23_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR24 register  *******************/
#define  BKP_DR24_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR25 register  *******************/
#define  BKP_DR25_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR26 register  *******************/
#define  BKP_DR26_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR27 register  *******************/
#define  BKP_DR27_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR28 register  *******************/
#define  BKP_DR28_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR29 register  *******************/
#define  BKP_DR29_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR30 register  *******************/
#define  BKP_DR30_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR31 register  *******************/
#define  BKP_DR31_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR32 register  *******************/
#define  BKP_DR32_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR33 register  *******************/
#define  BKP_DR33_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR34 register  *******************/
#define  BKP_DR34_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR35 register  *******************/
#define  BKP_DR35_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR36 register  *******************/
#define  BKP_DR36_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR37 register  *******************/
#define  BKP_DR37_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR38 register  *******************/
#define  BKP_DR38_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR39 register  *******************/
#define  BKP_DR39_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR40 register  *******************/
#define  BKP_DR40_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR41 register  *******************/
#define  BKP_DR41_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR42 register  *******************/
#define  BKP_DR42_D                          ((unsigned short)0xFFFF)     /*!< Backup data */

/******************  Bit definition for BKP_RTCCR register  *******************/
#define  BKP_RTCCR_CAL                       ((unsigned short)0x007F)     /*!< Calibration value */
#define  BKP_RTCCR_CCO                       ((unsigned short)0x0080)     /*!< Calibration Clock Output */
#define  BKP_RTCCR_ASOE                      ((unsigned short)0x0100)     /*!< Alarm or Second Output Enable */
#define  BKP_RTCCR_ASOS                      ((unsigned short)0x0200)     /*!< Alarm or Second Output Selection */

/********************  Bit definition for BKP_CR register  ********************/
#define  BKP_CR_TPE                          ((unsigned char)0x01)        /*!< TAMPER pin enable */
#define  BKP_CR_TPAL                         ((unsigned char)0x02)        /*!< TAMPER pin active level */

/*******************  Bit definition for BKP_CSR register  ********************/
#define  BKP_CSR_CTE                         ((unsigned short)0x0001)     /*!< Clear Tamper event */
#define  BKP_CSR_CTI                         ((unsigned short)0x0002)     /*!< Clear Tamper Interrupt */
#define  BKP_CSR_TPIE                        ((unsigned short)0x0004)     /*!< TAMPER Pin interrupt enable */
#define  BKP_CSR_TEF                         ((unsigned short)0x0100)     /*!< Tamper Event Flag */
#define  BKP_CSR_TIF                         ((unsigned short)0x0200)     /*!< Tamper Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((unsigned long)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIRDY                       ((unsigned long)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM                      ((unsigned long)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CR_HSICAL                       ((unsigned long)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSEON                        ((unsigned long)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((unsigned long)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CR_HSEBYP                       ((unsigned long)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((unsigned long)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CR_PLLON                        ((unsigned long)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLRDY                       ((unsigned long)0x02000000)        /*!< PLL clock ready flag */

#ifdef STM32F10X_CL
 #define  RCC_CR_PLL2ON                       ((unsigned long)0x04000000)        /*!< PLL2 enable */
 #define  RCC_CR_PLL2RDY                      ((unsigned long)0x08000000)        /*!< PLL2 clock ready flag */
 #define  RCC_CR_PLL3ON                       ((unsigned long)0x10000000)        /*!< PLL3 enable */
 #define  RCC_CR_PLL3RDY                      ((unsigned long)0x20000000)        /*!< PLL3 clock ready flag */
#endif /* STM32F10X_CL */

/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((unsigned long)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((unsigned long)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((unsigned long)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((unsigned long)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((unsigned long)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((unsigned long)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((unsigned long)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((unsigned long)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((unsigned long)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((unsigned long)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((unsigned long)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((unsigned long)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((unsigned long)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((unsigned long)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((unsigned long)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((unsigned long)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((unsigned long)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((unsigned long)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((unsigned long)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((unsigned long)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((unsigned long)0x00000400)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((unsigned long)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((unsigned long)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((unsigned long)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((unsigned long)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((unsigned long)0x00000700)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((unsigned long)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((unsigned long)0x00000800)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((unsigned long)0x00001000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((unsigned long)0x00002000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((unsigned long)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((unsigned long)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((unsigned long)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((unsigned long)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((unsigned long)0x00003800)        /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define  RCC_CFGR_ADCPRE                     ((unsigned long)0x0000C000)        /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_CFGR_ADCPRE_0                   ((unsigned long)0x00004000)        /*!< Bit 0 */
#define  RCC_CFGR_ADCPRE_1                   ((unsigned long)0x00008000)        /*!< Bit 1 */

#define  RCC_CFGR_ADCPRE_DIV2                ((unsigned long)0x00000000)        /*!< PCLK2 divided by 2 */
#define  RCC_CFGR_ADCPRE_DIV4                ((unsigned long)0x00004000)        /*!< PCLK2 divided by 4 */
#define  RCC_CFGR_ADCPRE_DIV6                ((unsigned long)0x00008000)        /*!< PCLK2 divided by 6 */
#define  RCC_CFGR_ADCPRE_DIV8                ((unsigned long)0x0000C000)        /*!< PCLK2 divided by 8 */

#define  RCC_CFGR_PLLSRC                     ((unsigned long)0x00010000)        /*!< PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE                   ((unsigned long)0x00020000)        /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMULL                    ((unsigned long)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((unsigned long)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((unsigned long)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((unsigned long)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((unsigned long)0x00200000)        /*!< Bit 3 */

#ifdef STM32F10X_CL
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((unsigned long)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((unsigned long)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((unsigned long)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((unsigned long)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL4                  ((unsigned long)0x00080000)        /*!< PLL input clock * 4 */
 #define  RCC_CFGR_PLLMULL5                  ((unsigned long)0x000C0000)        /*!< PLL input clock * 5 */
 #define  RCC_CFGR_PLLMULL6                  ((unsigned long)0x00100000)        /*!< PLL input clock * 6 */
 #define  RCC_CFGR_PLLMULL7                  ((unsigned long)0x00140000)        /*!< PLL input clock * 7 */
 #define  RCC_CFGR_PLLMULL8                  ((unsigned long)0x00180000)        /*!< PLL input clock * 8 */
 #define  RCC_CFGR_PLLMULL9                  ((unsigned long)0x001C0000)        /*!< PLL input clock * 9 */
 #define  RCC_CFGR_PLLMULL6_5                ((unsigned long)0x00340000)        /*!< PLL input clock * 6.5 */

 #define  RCC_CFGR_OTGFSPRE                  ((unsigned long)0x00400000)        /*!< USB OTG FS prescaler */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((unsigned long)0x0F000000)        /*!< MCO[3:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((unsigned long)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((unsigned long)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((unsigned long)0x04000000)        /*!< Bit 2 */
 #define  RCC_CFGR_MCO_3                     ((unsigned long)0x08000000)        /*!< Bit 3 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((unsigned long)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((unsigned long)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((unsigned long)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((unsigned long)0x06000000)        /*!< HSE clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLLCLK_Div2           ((unsigned long)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
 #define  RCC_CFGR_MCO_PLL2CLK               ((unsigned long)0x08000000)        /*!< PLL2 clock selected as MCO source*/
 #define  RCC_CFGR_MCO_PLL3CLK_Div2          ((unsigned long)0x09000000)        /*!< PLL3 clock divided by 2 selected as MCO source*/
 #define  RCC_CFGR_MCO_Ext_HSE               ((unsigned long)0x0A000000)        /*!< XT1 external 3-25 MHz oscillator clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLL3CLK               ((unsigned long)0x0B000000)        /*!< PLL3 clock selected as MCO source */
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((unsigned long)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((unsigned long)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((unsigned long)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((unsigned long)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((unsigned long)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((unsigned long)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((unsigned long)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((unsigned long)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((unsigned long)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((unsigned long)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((unsigned long)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((unsigned long)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((unsigned long)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((unsigned long)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((unsigned long)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((unsigned long)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((unsigned long)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((unsigned long)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((unsigned long)0x00380000)        /*!< PLL input clock*16 */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((unsigned long)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((unsigned long)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((unsigned long)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((unsigned long)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((unsigned long)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((unsigned long)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((unsigned long)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((unsigned long)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((unsigned long)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#else
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((unsigned long)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_HSE                ((unsigned long)0x00010000)        /*!< HSE clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_HSE              ((unsigned long)0x00000000)        /*!< HSE clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_HSE_Div2         ((unsigned long)0x00020000)        /*!< HSE clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((unsigned long)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((unsigned long)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((unsigned long)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((unsigned long)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((unsigned long)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((unsigned long)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((unsigned long)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((unsigned long)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((unsigned long)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((unsigned long)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((unsigned long)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((unsigned long)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((unsigned long)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((unsigned long)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((unsigned long)0x00380000)        /*!< PLL input clock*16 */
 #define  RCC_CFGR_USBPRE                    ((unsigned long)0x00400000)        /*!< USB Device prescaler */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((unsigned long)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((unsigned long)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((unsigned long)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((unsigned long)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((unsigned long)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((unsigned long)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((unsigned long)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((unsigned long)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((unsigned long)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#endif /* STM32F10X_CL */

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define  RCC_CIR_LSIRDYF                     ((unsigned long)0x00000001)        /*!< LSI Ready Interrupt flag */
#define  RCC_CIR_LSERDYF                     ((unsigned long)0x00000002)        /*!< LSE Ready Interrupt flag */
#define  RCC_CIR_HSIRDYF                     ((unsigned long)0x00000004)        /*!< HSI Ready Interrupt flag */
#define  RCC_CIR_HSERDYF                     ((unsigned long)0x00000008)        /*!< HSE Ready Interrupt flag */
#define  RCC_CIR_PLLRDYF                     ((unsigned long)0x00000010)        /*!< PLL Ready Interrupt flag */
#define  RCC_CIR_CSSF                        ((unsigned long)0x00000080)        /*!< Clock Security System Interrupt flag */
#define  RCC_CIR_LSIRDYIE                    ((unsigned long)0x00000100)        /*!< LSI Ready Interrupt Enable */
#define  RCC_CIR_LSERDYIE                    ((unsigned long)0x00000200)        /*!< LSE Ready Interrupt Enable */
#define  RCC_CIR_HSIRDYIE                    ((unsigned long)0x00000400)        /*!< HSI Ready Interrupt Enable */
#define  RCC_CIR_HSERDYIE                    ((unsigned long)0x00000800)        /*!< HSE Ready Interrupt Enable */
#define  RCC_CIR_PLLRDYIE                    ((unsigned long)0x00001000)        /*!< PLL Ready Interrupt Enable */
#define  RCC_CIR_LSIRDYC                     ((unsigned long)0x00010000)        /*!< LSI Ready Interrupt Clear */
#define  RCC_CIR_LSERDYC                     ((unsigned long)0x00020000)        /*!< LSE Ready Interrupt Clear */
#define  RCC_CIR_HSIRDYC                     ((unsigned long)0x00040000)        /*!< HSI Ready Interrupt Clear */
#define  RCC_CIR_HSERDYC                     ((unsigned long)0x00080000)        /*!< HSE Ready Interrupt Clear */
#define  RCC_CIR_PLLRDYC                     ((unsigned long)0x00100000)        /*!< PLL Ready Interrupt Clear */
#define  RCC_CIR_CSSC                        ((unsigned long)0x00800000)        /*!< Clock Security System Interrupt Clear */

#ifdef STM32F10X_CL
 #define  RCC_CIR_PLL2RDYF                    ((unsigned long)0x00000020)        /*!< PLL2 Ready Interrupt flag */
 #define  RCC_CIR_PLL3RDYF                    ((unsigned long)0x00000040)        /*!< PLL3 Ready Interrupt flag */
 #define  RCC_CIR_PLL2RDYIE                   ((unsigned long)0x00002000)        /*!< PLL2 Ready Interrupt Enable */
 #define  RCC_CIR_PLL3RDYIE                   ((unsigned long)0x00004000)        /*!< PLL3 Ready Interrupt Enable */
 #define  RCC_CIR_PLL2RDYC                    ((unsigned long)0x00200000)        /*!< PLL2 Ready Interrupt Clear */
 #define  RCC_CIR_PLL3RDYC                    ((unsigned long)0x00400000)        /*!< PLL3 Ready Interrupt Clear */
#endif /* STM32F10X_CL */

/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define  RCC_APB2RSTR_AFIORST                ((unsigned long)0x00000001)        /*!< Alternate Function I/O reset */
#define  RCC_APB2RSTR_IOPARST                ((unsigned long)0x00000004)        /*!< I/O port A reset */
#define  RCC_APB2RSTR_IOPBRST                ((unsigned long)0x00000008)        /*!< I/O port B reset */
#define  RCC_APB2RSTR_IOPCRST                ((unsigned long)0x00000010)        /*!< I/O port C reset */
#define  RCC_APB2RSTR_IOPDRST                ((unsigned long)0x00000020)        /*!< I/O port D reset */
#define  RCC_APB2RSTR_ADC1RST                ((unsigned long)0x00000200)        /*!< ADC 1 interface reset */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB2RSTR_ADC2RST                ((unsigned long)0x00000400)        /*!< ADC 2 interface reset */
#endif

#define  RCC_APB2RSTR_TIM1RST                ((unsigned long)0x00000800)        /*!< TIM1 Timer reset */
#define  RCC_APB2RSTR_SPI1RST                ((unsigned long)0x00001000)        /*!< SPI 1 reset */
#define  RCC_APB2RSTR_USART1RST              ((unsigned long)0x00004000)        /*!< USART1 reset */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
#define  RCC_APB2RSTR_TIM15RST               ((unsigned long)0x00010000)        /*!< TIM15 Timer reset */
#define  RCC_APB2RSTR_TIM16RST               ((unsigned long)0x00020000)        /*!< TIM16 Timer reset */
#define  RCC_APB2RSTR_TIM17RST               ((unsigned long)0x00040000)        /*!< TIM17 Timer reset */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB2RSTR_IOPERST               ((unsigned long)0x00000040)        /*!< I/O port E reset */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_APB2RSTR_IOPFRST               ((unsigned long)0x00000080)        /*!< I/O port F reset */
 #define  RCC_APB2RSTR_IOPGRST               ((unsigned long)0x00000100)        /*!< I/O port G reset */
 #define  RCC_APB2RSTR_TIM8RST               ((unsigned long)0x00002000)        /*!< TIM8 Timer reset */
 #define  RCC_APB2RSTR_ADC3RST               ((unsigned long)0x00008000)        /*!< ADC3 interface reset */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_APB2RSTR_IOPFRST               ((unsigned long)0x00000080)        /*!< I/O port F reset */
 #define  RCC_APB2RSTR_IOPGRST               ((unsigned long)0x00000100)        /*!< I/O port G reset */
#endif

#ifdef STM32F10X_XL
 #define  RCC_APB2RSTR_TIM9RST               ((unsigned long)0x00080000)         /*!< TIM9 Timer reset */
 #define  RCC_APB2RSTR_TIM10RST              ((unsigned long)0x00100000)         /*!< TIM10 Timer reset */
 #define  RCC_APB2RSTR_TIM11RST              ((unsigned long)0x00200000)         /*!< TIM11 Timer reset */
#endif /* STM32F10X_XL */

/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define  RCC_APB1RSTR_TIM2RST                ((unsigned long)0x00000001)        /*!< Timer 2 reset */
#define  RCC_APB1RSTR_TIM3RST                ((unsigned long)0x00000002)        /*!< Timer 3 reset */
#define  RCC_APB1RSTR_WWDGRST                ((unsigned long)0x00000800)        /*!< Window Watchdog reset */
#define  RCC_APB1RSTR_USART2RST              ((unsigned long)0x00020000)        /*!< USART 2 reset */
#define  RCC_APB1RSTR_I2C1RST                ((unsigned long)0x00200000)        /*!< I2C 1 reset */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB1RSTR_CAN1RST                ((unsigned long)0x02000000)        /*!< CAN1 reset */
#endif

#define  RCC_APB1RSTR_BKPRST                 ((unsigned long)0x08000000)        /*!< Backup interface reset */
#define  RCC_APB1RSTR_PWRRST                 ((unsigned long)0x10000000)        /*!< Power interface reset */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB1RSTR_TIM4RST               ((unsigned long)0x00000004)        /*!< Timer 4 reset */
 #define  RCC_APB1RSTR_SPI2RST               ((unsigned long)0x00004000)        /*!< SPI 2 reset */
 #define  RCC_APB1RSTR_USART3RST             ((unsigned long)0x00040000)        /*!< USART 3 reset */
 #define  RCC_APB1RSTR_I2C2RST               ((unsigned long)0x00400000)        /*!< I2C 2 reset */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_MD) || defined (STM32F10X_LD) || defined  (STM32F10X_XL)
 #define  RCC_APB1RSTR_USBRST                ((unsigned long)0x00800000)        /*!< USB Device reset */
#endif

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
 #define  RCC_APB1RSTR_TIM5RST                ((unsigned long)0x00000008)        /*!< Timer 5 reset */
 #define  RCC_APB1RSTR_TIM6RST                ((unsigned long)0x00000010)        /*!< Timer 6 reset */
 #define  RCC_APB1RSTR_TIM7RST                ((unsigned long)0x00000020)        /*!< Timer 7 reset */
 #define  RCC_APB1RSTR_SPI3RST                ((unsigned long)0x00008000)        /*!< SPI 3 reset */
 #define  RCC_APB1RSTR_UART4RST               ((unsigned long)0x00080000)        /*!< UART 4 reset */
 #define  RCC_APB1RSTR_UART5RST               ((unsigned long)0x00100000)        /*!< UART 5 reset */
 #define  RCC_APB1RSTR_DACRST                 ((unsigned long)0x20000000)        /*!< DAC interface reset */
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || defined  (STM32F10X_HD_VL)
 #define  RCC_APB1RSTR_TIM6RST                ((unsigned long)0x00000010)        /*!< Timer 6 reset */
 #define  RCC_APB1RSTR_TIM7RST                ((unsigned long)0x00000020)        /*!< Timer 7 reset */
 #define  RCC_APB1RSTR_DACRST                 ((unsigned long)0x20000000)        /*!< DAC interface reset */
 #define  RCC_APB1RSTR_CECRST                 ((unsigned long)0x40000000)        /*!< CEC interface reset */
#endif

#if defined  (STM32F10X_HD_VL)
 #define  RCC_APB1RSTR_TIM5RST                ((unsigned long)0x00000008)        /*!< Timer 5 reset */
 #define  RCC_APB1RSTR_TIM12RST               ((unsigned long)0x00000040)        /*!< TIM12 Timer reset */
 #define  RCC_APB1RSTR_TIM13RST               ((unsigned long)0x00000080)        /*!< TIM13 Timer reset */
 #define  RCC_APB1RSTR_TIM14RST               ((unsigned long)0x00000100)        /*!< TIM14 Timer reset */
 #define  RCC_APB1RSTR_SPI3RST                ((unsigned long)0x00008000)        /*!< SPI 3 reset */
 #define  RCC_APB1RSTR_UART4RST               ((unsigned long)0x00080000)        /*!< UART 4 reset */
 #define  RCC_APB1RSTR_UART5RST               ((unsigned long)0x00100000)        /*!< UART 5 reset */
#endif

#ifdef STM32F10X_CL
 #define  RCC_APB1RSTR_CAN2RST                ((unsigned long)0x04000000)        /*!< CAN2 reset */
#endif /* STM32F10X_CL */

#ifdef STM32F10X_XL
 #define  RCC_APB1RSTR_TIM12RST               ((unsigned long)0x00000040)         /*!< TIM12 Timer reset */
 #define  RCC_APB1RSTR_TIM13RST               ((unsigned long)0x00000080)         /*!< TIM13 Timer reset */
 #define  RCC_APB1RSTR_TIM14RST               ((unsigned long)0x00000100)         /*!< TIM14 Timer reset */
#endif /* STM32F10X_XL */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define  RCC_AHBENR_DMA1EN                   ((unsigned short)0x0001)            /*!< DMA1 clock enable */
#define  RCC_AHBENR_SRAMEN                   ((unsigned short)0x0004)            /*!< SRAM interface clock enable */
#define  RCC_AHBENR_FLITFEN                  ((unsigned short)0x0010)            /*!< FLITF clock enable */
#define  RCC_AHBENR_CRCEN                    ((unsigned short)0x0040)            /*!< CRC clock enable */

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_HD_VL)
 #define  RCC_AHBENR_DMA2EN                  ((unsigned short)0x0002)            /*!< DMA2 clock enable */
#endif

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_AHBENR_FSMCEN                  ((unsigned short)0x0100)            /*!< FSMC clock enable */
 #define  RCC_AHBENR_SDIOEN                  ((unsigned short)0x0400)            /*!< SDIO clock enable */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_AHBENR_FSMCEN                  ((unsigned short)0x0100)            /*!< FSMC clock enable */
#endif

#ifdef STM32F10X_CL
 #define  RCC_AHBENR_OTGFSEN                 ((unsigned long)0x00001000)         /*!< USB OTG FS clock enable */
 #define  RCC_AHBENR_ETHMACEN                ((unsigned long)0x00004000)         /*!< ETHERNET MAC clock enable */
 #define  RCC_AHBENR_ETHMACTXEN              ((unsigned long)0x00008000)         /*!< ETHERNET MAC Tx clock enable */
 #define  RCC_AHBENR_ETHMACRXEN              ((unsigned long)0x00010000)         /*!< ETHERNET MAC Rx clock enable */
#endif /* STM32F10X_CL */

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define  RCC_APB2ENR_AFIOEN                  ((unsigned long)0x00000001)         /*!< Alternate Function I/O clock enable */
#define  RCC_APB2ENR_IOPAEN                  ((unsigned long)0x00000004)         /*!< I/O port A clock enable */
#define  RCC_APB2ENR_IOPBEN                  ((unsigned long)0x00000008)         /*!< I/O port B clock enable */
#define  RCC_APB2ENR_IOPCEN                  ((unsigned long)0x00000010)         /*!< I/O port C clock enable */
#define  RCC_APB2ENR_IOPDEN                  ((unsigned long)0x00000020)         /*!< I/O port D clock enable */
#define  RCC_APB2ENR_ADC1EN                  ((unsigned long)0x00000200)         /*!< ADC 1 interface clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB2ENR_ADC2EN                  ((unsigned long)0x00000400)         /*!< ADC 2 interface clock enable */
#endif

#define  RCC_APB2ENR_TIM1EN                  ((unsigned long)0x00000800)         /*!< TIM1 Timer clock enable */
#define  RCC_APB2ENR_SPI1EN                  ((unsigned long)0x00001000)         /*!< SPI 1 clock enable */
#define  RCC_APB2ENR_USART1EN                ((unsigned long)0x00004000)         /*!< USART1 clock enable */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
#define  RCC_APB2ENR_TIM15EN                 ((unsigned long)0x00010000)         /*!< TIM15 Timer clock enable */
#define  RCC_APB2ENR_TIM16EN                 ((unsigned long)0x00020000)         /*!< TIM16 Timer clock enable */
#define  RCC_APB2ENR_TIM17EN                 ((unsigned long)0x00040000)         /*!< TIM17 Timer clock enable */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB2ENR_IOPEEN                 ((unsigned long)0x00000040)         /*!< I/O port E clock enable */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_APB2ENR_IOPFEN                 ((unsigned long)0x00000080)         /*!< I/O port F clock enable */
 #define  RCC_APB2ENR_IOPGEN                 ((unsigned long)0x00000100)         /*!< I/O port G clock enable */
 #define  RCC_APB2ENR_TIM8EN                 ((unsigned long)0x00002000)         /*!< TIM8 Timer clock enable */
 #define  RCC_APB2ENR_ADC3EN                 ((unsigned long)0x00008000)         /*!< DMA1 clock enable */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_APB2ENR_IOPFEN                 ((unsigned long)0x00000080)         /*!< I/O port F clock enable */
 #define  RCC_APB2ENR_IOPGEN                 ((unsigned long)0x00000100)         /*!< I/O port G clock enable */
#endif

#ifdef STM32F10X_XL
 #define  RCC_APB2ENR_TIM9EN                 ((unsigned long)0x00080000)         /*!< TIM9 Timer clock enable  */
 #define  RCC_APB2ENR_TIM10EN                ((unsigned long)0x00100000)         /*!< TIM10 Timer clock enable  */
 #define  RCC_APB2ENR_TIM11EN                ((unsigned long)0x00200000)         /*!< TIM11 Timer clock enable */
#endif

/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define  RCC_APB1ENR_TIM2EN                  ((unsigned long)0x00000001)        /*!< Timer 2 clock enabled*/
#define  RCC_APB1ENR_TIM3EN                  ((unsigned long)0x00000002)        /*!< Timer 3 clock enable */
#define  RCC_APB1ENR_WWDGEN                  ((unsigned long)0x00000800)        /*!< Window Watchdog clock enable */
#define  RCC_APB1ENR_USART2EN                ((unsigned long)0x00020000)        /*!< USART 2 clock enable */
#define  RCC_APB1ENR_I2C1EN                  ((unsigned long)0x00200000)        /*!< I2C 1 clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB1ENR_CAN1EN                  ((unsigned long)0x02000000)        /*!< CAN1 clock enable */
#endif

#define  RCC_APB1ENR_BKPEN                   ((unsigned long)0x08000000)        /*!< Backup interface clock enable */
#define  RCC_APB1ENR_PWREN                   ((unsigned long)0x10000000)        /*!< Power interface clock enable */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB1ENR_TIM4EN                 ((unsigned long)0x00000004)        /*!< Timer 4 clock enable */
 #define  RCC_APB1ENR_SPI2EN                 ((unsigned long)0x00004000)        /*!< SPI 2 clock enable */
 #define  RCC_APB1ENR_USART3EN               ((unsigned long)0x00040000)        /*!< USART 3 clock enable */
 #define  RCC_APB1ENR_I2C2EN                 ((unsigned long)0x00400000)        /*!< I2C 2 clock enable */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_MD) || defined  (STM32F10X_LD)
 #define  RCC_APB1ENR_USBEN                  ((unsigned long)0x00800000)        /*!< USB Device clock enable */
#endif

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL)
 #define  RCC_APB1ENR_TIM5EN                 ((unsigned long)0x00000008)        /*!< Timer 5 clock enable */
 #define  RCC_APB1ENR_TIM6EN                 ((unsigned long)0x00000010)        /*!< Timer 6 clock enable */
 #define  RCC_APB1ENR_TIM7EN                 ((unsigned long)0x00000020)        /*!< Timer 7 clock enable */
 #define  RCC_APB1ENR_SPI3EN                 ((unsigned long)0x00008000)        /*!< SPI 3 clock enable */
 #define  RCC_APB1ENR_UART4EN                ((unsigned long)0x00080000)        /*!< UART 4 clock enable */
 #define  RCC_APB1ENR_UART5EN                ((unsigned long)0x00100000)        /*!< UART 5 clock enable */
 #define  RCC_APB1ENR_DACEN                  ((unsigned long)0x20000000)        /*!< DAC interface clock enable */
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || defined  (STM32F10X_HD_VL)
 #define  RCC_APB1ENR_TIM6EN                 ((unsigned long)0x00000010)        /*!< Timer 6 clock enable */
 #define  RCC_APB1ENR_TIM7EN                 ((unsigned long)0x00000020)        /*!< Timer 7 clock enable */
 #define  RCC_APB1ENR_DACEN                  ((unsigned long)0x20000000)        /*!< DAC interface clock enable */
 #define  RCC_APB1ENR_CECEN                  ((unsigned long)0x40000000)        /*!< CEC interface clock enable */
#endif

#ifdef STM32F10X_HD_VL
 #define  RCC_APB1ENR_TIM5EN                 ((unsigned long)0x00000008)        /*!< Timer 5 clock enable */
 #define  RCC_APB1ENR_TIM12EN                ((unsigned long)0x00000040)         /*!< TIM12 Timer clock enable  */
 #define  RCC_APB1ENR_TIM13EN                ((unsigned long)0x00000080)         /*!< TIM13 Timer clock enable  */
 #define  RCC_APB1ENR_TIM14EN                ((unsigned long)0x00000100)         /*!< TIM14 Timer clock enable */
 #define  RCC_APB1ENR_SPI3EN                 ((unsigned long)0x00008000)        /*!< SPI 3 clock enable */
 #define  RCC_APB1ENR_UART4EN                ((unsigned long)0x00080000)        /*!< UART 4 clock enable */
 #define  RCC_APB1ENR_UART5EN                ((unsigned long)0x00100000)        /*!< UART 5 clock enable */
#endif /* STM32F10X_HD_VL */

#ifdef STM32F10X_CL
 #define  RCC_APB1ENR_CAN2EN                  ((unsigned long)0x04000000)        /*!< CAN2 clock enable */
#endif /* STM32F10X_CL */

#ifdef STM32F10X_XL
 #define  RCC_APB1ENR_TIM12EN                ((unsigned long)0x00000040)         /*!< TIM12 Timer clock enable  */
 #define  RCC_APB1ENR_TIM13EN                ((unsigned long)0x00000080)         /*!< TIM13 Timer clock enable  */
 #define  RCC_APB1ENR_TIM14EN                ((unsigned long)0x00000100)         /*!< TIM14 Timer clock enable */
#endif /* STM32F10X_XL */

/*******************  Bit definition for RCC_BDCR register  *******************/
#define  RCC_BDCR_LSEON                      ((unsigned long)0x00000001)        /*!< External Low Speed oscillator enable */
#define  RCC_BDCR_LSERDY                     ((unsigned long)0x00000002)        /*!< External Low Speed oscillator Ready */
#define  RCC_BDCR_LSEBYP                     ((unsigned long)0x00000004)        /*!< External Low Speed oscillator Bypass */

#define  RCC_BDCR_RTCSEL                     ((unsigned long)0x00000300)        /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_BDCR_RTCSEL_0                   ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  RCC_BDCR_RTCSEL_1                   ((unsigned long)0x00000200)        /*!< Bit 1 */

/*!< RTC congiguration */
#define  RCC_BDCR_RTCSEL_NOCLOCK             ((unsigned long)0x00000000)        /*!< No clock */
#define  RCC_BDCR_RTCSEL_LSE                 ((unsigned long)0x00000100)        /*!< LSE oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSI                 ((unsigned long)0x00000200)        /*!< LSI oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_HSE                 ((unsigned long)0x00000300)        /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define  RCC_BDCR_RTCEN                      ((unsigned long)0x00008000)        /*!< RTC clock enable */
#define  RCC_BDCR_BDRST                      ((unsigned long)0x00010000)        /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/
#define  RCC_CSR_LSION                       ((unsigned long)0x00000001)        /*!< Internal Low Speed oscillator enable */
#define  RCC_CSR_LSIRDY                      ((unsigned long)0x00000002)        /*!< Internal Low Speed oscillator Ready */
#define  RCC_CSR_RMVF                        ((unsigned long)0x01000000)        /*!< Remove reset flag */
#define  RCC_CSR_PINRSTF                     ((unsigned long)0x04000000)        /*!< PIN reset flag */
#define  RCC_CSR_PORRSTF                     ((unsigned long)0x08000000)        /*!< POR/PDR reset flag */
#define  RCC_CSR_SFTRSTF                     ((unsigned long)0x10000000)        /*!< Software Reset flag */
#define  RCC_CSR_IWDGRSTF                    ((unsigned long)0x20000000)        /*!< Independent Watchdog reset flag */
#define  RCC_CSR_WWDGRSTF                    ((unsigned long)0x40000000)        /*!< Window watchdog reset flag */
#define  RCC_CSR_LPWRRSTF                    ((unsigned long)0x80000000)        /*!< Low-Power reset flag */

#ifdef STM32F10X_CL
/*******************  Bit definition for RCC_AHBRSTR register  ****************/
 #define  RCC_AHBRSTR_OTGFSRST               ((unsigned long)0x00001000)         /*!< USB OTG FS reset */
 #define  RCC_AHBRSTR_ETHMACRST              ((unsigned long)0x00004000)         /*!< ETHERNET MAC reset */

/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
 #define  RCC_CFGR2_PREDIV1                  ((unsigned long)0x0000000F)        /*!< PREDIV1[3:0] bits */
 #define  RCC_CFGR2_PREDIV1_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV1_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV1_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV1_3                ((unsigned long)0x00000008)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV1_DIV1             ((unsigned long)0x00000000)        /*!< PREDIV1 input clock not divided */
 #define  RCC_CFGR2_PREDIV1_DIV2             ((unsigned long)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV1_DIV3             ((unsigned long)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV1_DIV4             ((unsigned long)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV1_DIV5             ((unsigned long)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV1_DIV6             ((unsigned long)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV1_DIV7             ((unsigned long)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV1_DIV8             ((unsigned long)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV1_DIV9             ((unsigned long)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV1_DIV10            ((unsigned long)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV1_DIV11            ((unsigned long)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV1_DIV12            ((unsigned long)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV1_DIV13            ((unsigned long)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV1_DIV14            ((unsigned long)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV1_DIV15            ((unsigned long)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV1_DIV16            ((unsigned long)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */

/*!< PREDIV2 configuration */
 #define  RCC_CFGR2_PREDIV2                  ((unsigned long)0x000000F0)        /*!< PREDIV2[3:0] bits */
 #define  RCC_CFGR2_PREDIV2_0                ((unsigned long)0x00000010)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV2_1                ((unsigned long)0x00000020)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV2_2                ((unsigned long)0x00000040)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV2_3                ((unsigned long)0x00000080)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV2_DIV1             ((unsigned long)0x00000000)        /*!< PREDIV2 input clock not divided */
 #define  RCC_CFGR2_PREDIV2_DIV2             ((unsigned long)0x00000010)        /*!< PREDIV2 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV2_DIV3             ((unsigned long)0x00000020)        /*!< PREDIV2 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV2_DIV4             ((unsigned long)0x00000030)        /*!< PREDIV2 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV2_DIV5             ((unsigned long)0x00000040)        /*!< PREDIV2 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV2_DIV6             ((unsigned long)0x00000050)        /*!< PREDIV2 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV2_DIV7             ((unsigned long)0x00000060)        /*!< PREDIV2 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV2_DIV8             ((unsigned long)0x00000070)        /*!< PREDIV2 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV2_DIV9             ((unsigned long)0x00000080)        /*!< PREDIV2 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV2_DIV10            ((unsigned long)0x00000090)        /*!< PREDIV2 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV2_DIV11            ((unsigned long)0x000000A0)        /*!< PREDIV2 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV2_DIV12            ((unsigned long)0x000000B0)        /*!< PREDIV2 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV2_DIV13            ((unsigned long)0x000000C0)        /*!< PREDIV2 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV2_DIV14            ((unsigned long)0x000000D0)        /*!< PREDIV2 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV2_DIV15            ((unsigned long)0x000000E0)        /*!< PREDIV2 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV2_DIV16            ((unsigned long)0x000000F0)        /*!< PREDIV2 input clock divided by 16 */

/*!< PLL2MUL configuration */
 #define  RCC_CFGR2_PLL2MUL                  ((unsigned long)0x00000F00)        /*!< PLL2MUL[3:0] bits */
 #define  RCC_CFGR2_PLL2MUL_0                ((unsigned long)0x00000100)        /*!< Bit 0 */
 #define  RCC_CFGR2_PLL2MUL_1                ((unsigned long)0x00000200)        /*!< Bit 1 */
 #define  RCC_CFGR2_PLL2MUL_2                ((unsigned long)0x00000400)        /*!< Bit 2 */
 #define  RCC_CFGR2_PLL2MUL_3                ((unsigned long)0x00000800)        /*!< Bit 3 */

 #define  RCC_CFGR2_PLL2MUL8                 ((unsigned long)0x00000600)        /*!< PLL2 input clock * 8 */
 #define  RCC_CFGR2_PLL2MUL9                 ((unsigned long)0x00000700)        /*!< PLL2 input clock * 9 */
 #define  RCC_CFGR2_PLL2MUL10                ((unsigned long)0x00000800)        /*!< PLL2 input clock * 10 */
 #define  RCC_CFGR2_PLL2MUL11                ((unsigned long)0x00000900)        /*!< PLL2 input clock * 11 */
 #define  RCC_CFGR2_PLL2MUL12                ((unsigned long)0x00000A00)        /*!< PLL2 input clock * 12 */
 #define  RCC_CFGR2_PLL2MUL13                ((unsigned long)0x00000B00)        /*!< PLL2 input clock * 13 */
 #define  RCC_CFGR2_PLL2MUL14                ((unsigned long)0x00000C00)        /*!< PLL2 input clock * 14 */
 #define  RCC_CFGR2_PLL2MUL16                ((unsigned long)0x00000E00)        /*!< PLL2 input clock * 16 */
 #define  RCC_CFGR2_PLL2MUL20                ((unsigned long)0x00000F00)        /*!< PLL2 input clock * 20 */

/*!< PLL3MUL configuration */
 #define  RCC_CFGR2_PLL3MUL                  ((unsigned long)0x0000F000)        /*!< PLL3MUL[3:0] bits */
 #define  RCC_CFGR2_PLL3MUL_0                ((unsigned long)0x00001000)        /*!< Bit 0 */
 #define  RCC_CFGR2_PLL3MUL_1                ((unsigned long)0x00002000)        /*!< Bit 1 */
 #define  RCC_CFGR2_PLL3MUL_2                ((unsigned long)0x00004000)        /*!< Bit 2 */
 #define  RCC_CFGR2_PLL3MUL_3                ((unsigned long)0x00008000)        /*!< Bit 3 */

 #define  RCC_CFGR2_PLL3MUL8                 ((unsigned long)0x00006000)        /*!< PLL3 input clock * 8 */
 #define  RCC_CFGR2_PLL3MUL9                 ((unsigned long)0x00007000)        /*!< PLL3 input clock * 9 */
 #define  RCC_CFGR2_PLL3MUL10                ((unsigned long)0x00008000)        /*!< PLL3 input clock * 10 */
 #define  RCC_CFGR2_PLL3MUL11                ((unsigned long)0x00009000)        /*!< PLL3 input clock * 11 */
 #define  RCC_CFGR2_PLL3MUL12                ((unsigned long)0x0000A000)        /*!< PLL3 input clock * 12 */
 #define  RCC_CFGR2_PLL3MUL13                ((unsigned long)0x0000B000)        /*!< PLL3 input clock * 13 */
 #define  RCC_CFGR2_PLL3MUL14                ((unsigned long)0x0000C000)        /*!< PLL3 input clock * 14 */
 #define  RCC_CFGR2_PLL3MUL16                ((unsigned long)0x0000E000)        /*!< PLL3 input clock * 16 */
 #define  RCC_CFGR2_PLL3MUL20                ((unsigned long)0x0000F000)        /*!< PLL3 input clock * 20 */

 #define  RCC_CFGR2_PREDIV1SRC               ((unsigned long)0x00010000)        /*!< PREDIV1 entry clock source */
 #define  RCC_CFGR2_PREDIV1SRC_PLL2          ((unsigned long)0x00010000)        /*!< PLL2 selected as PREDIV1 entry clock source */
 #define  RCC_CFGR2_PREDIV1SRC_HSE           ((unsigned long)0x00000000)        /*!< HSE selected as PREDIV1 entry clock source */
 #define  RCC_CFGR2_I2S2SRC                  ((unsigned long)0x00020000)        /*!< I2S2 entry clock source */
 #define  RCC_CFGR2_I2S3SRC                  ((unsigned long)0x00040000)        /*!< I2S3 clock source */
#endif /* STM32F10X_CL */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
 #define  RCC_CFGR2_PREDIV1                  ((unsigned long)0x0000000F)        /*!< PREDIV1[3:0] bits */
 #define  RCC_CFGR2_PREDIV1_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV1_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV1_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV1_3                ((unsigned long)0x00000008)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV1_DIV1             ((unsigned long)0x00000000)        /*!< PREDIV1 input clock not divided */
 #define  RCC_CFGR2_PREDIV1_DIV2             ((unsigned long)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV1_DIV3             ((unsigned long)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV1_DIV4             ((unsigned long)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV1_DIV5             ((unsigned long)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV1_DIV6             ((unsigned long)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV1_DIV7             ((unsigned long)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV1_DIV8             ((unsigned long)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV1_DIV9             ((unsigned long)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV1_DIV10            ((unsigned long)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV1_DIV11            ((unsigned long)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV1_DIV12            ((unsigned long)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV1_DIV13            ((unsigned long)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV1_DIV14            ((unsigned long)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV1_DIV15            ((unsigned long)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV1_DIV16            ((unsigned long)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */
#endif

/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CRL register  *******************/
#define  GPIO_CRL_MODE                       ((unsigned long)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CRL_MODE0                      ((unsigned long)0x00000003)        /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CRL_MODE0_0                    ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRL_MODE0_1                    ((unsigned long)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRL_MODE1                      ((unsigned long)0x00000030)        /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CRL_MODE1_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  GPIO_CRL_MODE1_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  GPIO_CRL_MODE2                      ((unsigned long)0x00000300)        /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CRL_MODE2_0                    ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  GPIO_CRL_MODE2_1                    ((unsigned long)0x00000200)        /*!< Bit 1 */

#define  GPIO_CRL_MODE3                      ((unsigned long)0x00003000)        /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CRL_MODE3_0                    ((unsigned long)0x00001000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE3_1                    ((unsigned long)0x00002000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE4                      ((unsigned long)0x00030000)        /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CRL_MODE4_0                    ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE4_1                    ((unsigned long)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE5                      ((unsigned long)0x00300000)        /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CRL_MODE5_0                    ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE5_1                    ((unsigned long)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE6                      ((unsigned long)0x03000000)        /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CRL_MODE6_0                    ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE6_1                    ((unsigned long)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE7                      ((unsigned long)0x30000000)        /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CRL_MODE7_0                    ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE7_1                    ((unsigned long)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF                        ((unsigned long)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CRL_CNF0                       ((unsigned long)0x0000000C)        /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CRL_CNF0_0                     ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRL_CNF0_1                     ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRL_CNF1                       ((unsigned long)0x000000C0)        /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CRL_CNF1_0                     ((unsigned long)0x00000040)        /*!< Bit 0 */
#define  GPIO_CRL_CNF1_1                     ((unsigned long)0x00000080)        /*!< Bit 1 */

#define  GPIO_CRL_CNF2                       ((unsigned long)0x00000C00)        /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CRL_CNF2_0                     ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  GPIO_CRL_CNF2_1                     ((unsigned long)0x00000800)        /*!< Bit 1 */

#define  GPIO_CRL_CNF3                       ((unsigned long)0x0000C000)        /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CRL_CNF3_0                     ((unsigned long)0x00004000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF3_1                     ((unsigned long)0x00008000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF4                       ((unsigned long)0x000C0000)        /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CRL_CNF4_0                     ((unsigned long)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF4_1                     ((unsigned long)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF5                       ((unsigned long)0x00C00000)        /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CRL_CNF5_0                     ((unsigned long)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF5_1                     ((unsigned long)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF6                       ((unsigned long)0x0C000000)        /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CRL_CNF6_0                     ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF6_1                     ((unsigned long)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF7                       ((unsigned long)0xC0000000)        /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CRL_CNF7_0                     ((unsigned long)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF7_1                     ((unsigned long)0x80000000)        /*!< Bit 1 */

/*******************  Bit definition for GPIO_CRH register  *******************/
#define  GPIO_CRH_MODE                       ((unsigned long)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CRH_MODE8                      ((unsigned long)0x00000003)        /*!< MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CRH_MODE8_0                    ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRH_MODE8_1                    ((unsigned long)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRH_MODE9                      ((unsigned long)0x00000030)        /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CRH_MODE9_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  GPIO_CRH_MODE9_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  GPIO_CRH_MODE10                     ((unsigned long)0x00000300)        /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CRH_MODE10_0                   ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  GPIO_CRH_MODE10_1                   ((unsigned long)0x00000200)        /*!< Bit 1 */

#define  GPIO_CRH_MODE11                     ((unsigned long)0x00003000)        /*!< MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CRH_MODE11_0                   ((unsigned long)0x00001000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE11_1                   ((unsigned long)0x00002000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE12                     ((unsigned long)0x00030000)        /*!< MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CRH_MODE12_0                   ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE12_1                   ((unsigned long)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE13                     ((unsigned long)0x00300000)        /*!< MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CRH_MODE13_0                   ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE13_1                   ((unsigned long)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE14                     ((unsigned long)0x03000000)        /*!< MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CRH_MODE14_0                   ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE14_1                   ((unsigned long)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE15                     ((unsigned long)0x30000000)        /*!< MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CRH_MODE15_0                   ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE15_1                   ((unsigned long)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF                        ((unsigned long)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CRH_CNF8                       ((unsigned long)0x0000000C)        /*!< CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CRH_CNF8_0                     ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRH_CNF8_1                     ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRH_CNF9                       ((unsigned long)0x000000C0)        /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CRH_CNF9_0                     ((unsigned long)0x00000040)        /*!< Bit 0 */
#define  GPIO_CRH_CNF9_1                     ((unsigned long)0x00000080)        /*!< Bit 1 */

#define  GPIO_CRH_CNF10                      ((unsigned long)0x00000C00)        /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CRH_CNF10_0                    ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  GPIO_CRH_CNF10_1                    ((unsigned long)0x00000800)        /*!< Bit 1 */

#define  GPIO_CRH_CNF11                      ((unsigned long)0x0000C000)        /*!< CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CRH_CNF11_0                    ((unsigned long)0x00004000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF11_1                    ((unsigned long)0x00008000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF12                      ((unsigned long)0x000C0000)        /*!< CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CRH_CNF12_0                    ((unsigned long)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF12_1                    ((unsigned long)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF13                      ((unsigned long)0x00C00000)        /*!< CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CRH_CNF13_0                    ((unsigned long)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF13_1                    ((unsigned long)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF14                      ((unsigned long)0x0C000000)        /*!< CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CRH_CNF14_0                    ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF14_1                    ((unsigned long)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF15                      ((unsigned long)0xC0000000)        /*!< CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CRH_CNF15_0                    ((unsigned long)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF15_1                    ((unsigned long)0x80000000)        /*!< Bit 1 */

/*!<******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR0                        ((unsigned short)0x0001)            /*!< Port input data, bit 0 */
#define GPIO_IDR_IDR1                        ((unsigned short)0x0002)            /*!< Port input data, bit 1 */
#define GPIO_IDR_IDR2                        ((unsigned short)0x0004)            /*!< Port input data, bit 2 */
#define GPIO_IDR_IDR3                        ((unsigned short)0x0008)            /*!< Port input data, bit 3 */
#define GPIO_IDR_IDR4                        ((unsigned short)0x0010)            /*!< Port input data, bit 4 */
#define GPIO_IDR_IDR5                        ((unsigned short)0x0020)            /*!< Port input data, bit 5 */
#define GPIO_IDR_IDR6                        ((unsigned short)0x0040)            /*!< Port input data, bit 6 */
#define GPIO_IDR_IDR7                        ((unsigned short)0x0080)            /*!< Port input data, bit 7 */
#define GPIO_IDR_IDR8                        ((unsigned short)0x0100)            /*!< Port input data, bit 8 */
#define GPIO_IDR_IDR9                        ((unsigned short)0x0200)            /*!< Port input data, bit 9 */
#define GPIO_IDR_IDR10                       ((unsigned short)0x0400)            /*!< Port input data, bit 10 */
#define GPIO_IDR_IDR11                       ((unsigned short)0x0800)            /*!< Port input data, bit 11 */
#define GPIO_IDR_IDR12                       ((unsigned short)0x1000)            /*!< Port input data, bit 12 */
#define GPIO_IDR_IDR13                       ((unsigned short)0x2000)            /*!< Port input data, bit 13 */
#define GPIO_IDR_IDR14                       ((unsigned short)0x4000)            /*!< Port input data, bit 14 */
#define GPIO_IDR_IDR15                       ((unsigned short)0x8000)            /*!< Port input data, bit 15 */

/*******************  Bit definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR0                        ((unsigned short)0x0001)            /*!< Port output data, bit 0 */
#define GPIO_ODR_ODR1                        ((unsigned short)0x0002)            /*!< Port output data, bit 1 */
#define GPIO_ODR_ODR2                        ((unsigned short)0x0004)            /*!< Port output data, bit 2 */
#define GPIO_ODR_ODR3                        ((unsigned short)0x0008)            /*!< Port output data, bit 3 */
#define GPIO_ODR_ODR4                        ((unsigned short)0x0010)            /*!< Port output data, bit 4 */
#define GPIO_ODR_ODR5                        ((unsigned short)0x0020)            /*!< Port output data, bit 5 */
#define GPIO_ODR_ODR6                        ((unsigned short)0x0040)            /*!< Port output data, bit 6 */
#define GPIO_ODR_ODR7                        ((unsigned short)0x0080)            /*!< Port output data, bit 7 */
#define GPIO_ODR_ODR8                        ((unsigned short)0x0100)            /*!< Port output data, bit 8 */
#define GPIO_ODR_ODR9                        ((unsigned short)0x0200)            /*!< Port output data, bit 9 */
#define GPIO_ODR_ODR10                       ((unsigned short)0x0400)            /*!< Port output data, bit 10 */
#define GPIO_ODR_ODR11                       ((unsigned short)0x0800)            /*!< Port output data, bit 11 */
#define GPIO_ODR_ODR12                       ((unsigned short)0x1000)            /*!< Port output data, bit 12 */
#define GPIO_ODR_ODR13                       ((unsigned short)0x2000)            /*!< Port output data, bit 13 */
#define GPIO_ODR_ODR14                       ((unsigned short)0x4000)            /*!< Port output data, bit 14 */
#define GPIO_ODR_ODR15                       ((unsigned short)0x8000)            /*!< Port output data, bit 15 */

/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0                        ((unsigned long)0x00000001)        /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1                        ((unsigned long)0x00000002)        /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2                        ((unsigned long)0x00000004)        /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3                        ((unsigned long)0x00000008)        /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4                        ((unsigned long)0x00000010)        /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5                        ((unsigned long)0x00000020)        /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6                        ((unsigned long)0x00000040)        /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7                        ((unsigned long)0x00000080)        /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8                        ((unsigned long)0x00000100)        /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9                        ((unsigned long)0x00000200)        /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10                       ((unsigned long)0x00000400)        /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11                       ((unsigned long)0x00000800)        /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12                       ((unsigned long)0x00001000)        /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13                       ((unsigned long)0x00002000)        /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14                       ((unsigned long)0x00004000)        /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15                       ((unsigned long)0x00008000)        /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0                        ((unsigned long)0x00010000)        /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1                        ((unsigned long)0x00020000)        /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2                        ((unsigned long)0x00040000)        /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3                        ((unsigned long)0x00080000)        /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4                        ((unsigned long)0x00100000)        /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5                        ((unsigned long)0x00200000)        /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6                        ((unsigned long)0x00400000)        /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7                        ((unsigned long)0x00800000)        /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8                        ((unsigned long)0x01000000)        /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9                        ((unsigned long)0x02000000)        /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10                       ((unsigned long)0x04000000)        /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11                       ((unsigned long)0x08000000)        /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12                       ((unsigned long)0x10000000)        /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13                       ((unsigned long)0x20000000)        /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14                       ((unsigned long)0x40000000)        /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15                       ((unsigned long)0x80000000)        /*!< Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BRR register  *******************/
#define GPIO_BRR_BR0                         ((unsigned short)0x0001)            /*!< Port x Reset bit 0 */
#define GPIO_BRR_BR1                         ((unsigned short)0x0002)            /*!< Port x Reset bit 1 */
#define GPIO_BRR_BR2                         ((unsigned short)0x0004)            /*!< Port x Reset bit 2 */
#define GPIO_BRR_BR3                         ((unsigned short)0x0008)            /*!< Port x Reset bit 3 */
#define GPIO_BRR_BR4                         ((unsigned short)0x0010)            /*!< Port x Reset bit 4 */
#define GPIO_BRR_BR5                         ((unsigned short)0x0020)            /*!< Port x Reset bit 5 */
#define GPIO_BRR_BR6                         ((unsigned short)0x0040)            /*!< Port x Reset bit 6 */
#define GPIO_BRR_BR7                         ((unsigned short)0x0080)            /*!< Port x Reset bit 7 */
#define GPIO_BRR_BR8                         ((unsigned short)0x0100)            /*!< Port x Reset bit 8 */
#define GPIO_BRR_BR9                         ((unsigned short)0x0200)            /*!< Port x Reset bit 9 */
#define GPIO_BRR_BR10                        ((unsigned short)0x0400)            /*!< Port x Reset bit 10 */
#define GPIO_BRR_BR11                        ((unsigned short)0x0800)            /*!< Port x Reset bit 11 */
#define GPIO_BRR_BR12                        ((unsigned short)0x1000)            /*!< Port x Reset bit 12 */
#define GPIO_BRR_BR13                        ((unsigned short)0x2000)            /*!< Port x Reset bit 13 */
#define GPIO_BRR_BR14                        ((unsigned short)0x4000)            /*!< Port x Reset bit 14 */
#define GPIO_BRR_BR15                        ((unsigned short)0x8000)            /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCKR_LCK0                       ((unsigned long)0x00000001)        /*!< Port x Lock bit 0 */
#define GPIO_LCKR_LCK1                       ((unsigned long)0x00000002)        /*!< Port x Lock bit 1 */
#define GPIO_LCKR_LCK2                       ((unsigned long)0x00000004)        /*!< Port x Lock bit 2 */
#define GPIO_LCKR_LCK3                       ((unsigned long)0x00000008)        /*!< Port x Lock bit 3 */
#define GPIO_LCKR_LCK4                       ((unsigned long)0x00000010)        /*!< Port x Lock bit 4 */
#define GPIO_LCKR_LCK5                       ((unsigned long)0x00000020)        /*!< Port x Lock bit 5 */
#define GPIO_LCKR_LCK6                       ((unsigned long)0x00000040)        /*!< Port x Lock bit 6 */
#define GPIO_LCKR_LCK7                       ((unsigned long)0x00000080)        /*!< Port x Lock bit 7 */
#define GPIO_LCKR_LCK8                       ((unsigned long)0x00000100)        /*!< Port x Lock bit 8 */
#define GPIO_LCKR_LCK9                       ((unsigned long)0x00000200)        /*!< Port x Lock bit 9 */
#define GPIO_LCKR_LCK10                      ((unsigned long)0x00000400)        /*!< Port x Lock bit 10 */
#define GPIO_LCKR_LCK11                      ((unsigned long)0x00000800)        /*!< Port x Lock bit 11 */
#define GPIO_LCKR_LCK12                      ((unsigned long)0x00001000)        /*!< Port x Lock bit 12 */
#define GPIO_LCKR_LCK13                      ((unsigned long)0x00002000)        /*!< Port x Lock bit 13 */
#define GPIO_LCKR_LCK14                      ((unsigned long)0x00004000)        /*!< Port x Lock bit 14 */
#define GPIO_LCKR_LCK15                      ((unsigned long)0x00008000)        /*!< Port x Lock bit 15 */
#define GPIO_LCKR_LCKK                       ((unsigned long)0x00010000)        /*!< Lock key */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN                        ((unsigned char)0x0F)               /*!< PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_0                      ((unsigned char)0x01)               /*!< Bit 0 */
#define AFIO_EVCR_PIN_1                      ((unsigned char)0x02)               /*!< Bit 1 */
#define AFIO_EVCR_PIN_2                      ((unsigned char)0x04)               /*!< Bit 2 */
#define AFIO_EVCR_PIN_3                      ((unsigned char)0x08)               /*!< Bit 3 */

/*!< PIN configuration */
#define AFIO_EVCR_PIN_PX0                    ((unsigned char)0x00)               /*!< Pin 0 selected */
#define AFIO_EVCR_PIN_PX1                    ((unsigned char)0x01)               /*!< Pin 1 selected */
#define AFIO_EVCR_PIN_PX2                    ((unsigned char)0x02)               /*!< Pin 2 selected */
#define AFIO_EVCR_PIN_PX3                    ((unsigned char)0x03)               /*!< Pin 3 selected */
#define AFIO_EVCR_PIN_PX4                    ((unsigned char)0x04)               /*!< Pin 4 selected */
#define AFIO_EVCR_PIN_PX5                    ((unsigned char)0x05)               /*!< Pin 5 selected */
#define AFIO_EVCR_PIN_PX6                    ((unsigned char)0x06)               /*!< Pin 6 selected */
#define AFIO_EVCR_PIN_PX7                    ((unsigned char)0x07)               /*!< Pin 7 selected */
#define AFIO_EVCR_PIN_PX8                    ((unsigned char)0x08)               /*!< Pin 8 selected */
#define AFIO_EVCR_PIN_PX9                    ((unsigned char)0x09)               /*!< Pin 9 selected */
#define AFIO_EVCR_PIN_PX10                   ((unsigned char)0x0A)               /*!< Pin 10 selected */
#define AFIO_EVCR_PIN_PX11                   ((unsigned char)0x0B)               /*!< Pin 11 selected */
#define AFIO_EVCR_PIN_PX12                   ((unsigned char)0x0C)               /*!< Pin 12 selected */
#define AFIO_EVCR_PIN_PX13                   ((unsigned char)0x0D)               /*!< Pin 13 selected */
#define AFIO_EVCR_PIN_PX14                   ((unsigned char)0x0E)               /*!< Pin 14 selected */
#define AFIO_EVCR_PIN_PX15                   ((unsigned char)0x0F)               /*!< Pin 15 selected */

#define AFIO_EVCR_PORT                       ((unsigned char)0x70)               /*!< PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_0                     ((unsigned char)0x10)               /*!< Bit 0 */
#define AFIO_EVCR_PORT_1                     ((unsigned char)0x20)               /*!< Bit 1 */
#define AFIO_EVCR_PORT_2                     ((unsigned char)0x40)               /*!< Bit 2 */

/*!< PORT configuration */
#define AFIO_EVCR_PORT_PA                    ((unsigned char)0x00)               /*!< Port A selected */
#define AFIO_EVCR_PORT_PB                    ((unsigned char)0x10)               /*!< Port B selected */
#define AFIO_EVCR_PORT_PC                    ((unsigned char)0x20)               /*!< Port C selected */
#define AFIO_EVCR_PORT_PD                    ((unsigned char)0x30)               /*!< Port D selected */
#define AFIO_EVCR_PORT_PE                    ((unsigned char)0x40)               /*!< Port E selected */

#define AFIO_EVCR_EVOE                       ((unsigned char)0x80)               /*!< Event Output Enable */

/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1_REMAP                 ((unsigned long)0x00000001)        /*!< SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP                 ((unsigned long)0x00000002)        /*!< I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP               ((unsigned long)0x00000004)        /*!< USART1 remapping */
#define AFIO_MAPR_USART2_REMAP               ((unsigned long)0x00000008)        /*!< USART2 remapping */

#define AFIO_MAPR_USART3_REMAP               ((unsigned long)0x00000030)        /*!< USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_0             ((unsigned long)0x00000010)        /*!< Bit 0 */
#define AFIO_MAPR_USART3_REMAP_1             ((unsigned long)0x00000020)        /*!< Bit 1 */

/* USART3_REMAP configuration */
#define AFIO_MAPR_USART3_REMAP_NOREMAP       ((unsigned long)0x00000000)        /*!< No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  ((unsigned long)0x00000010)        /*!< Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     ((unsigned long)0x00000030)        /*!< Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP                 ((unsigned long)0x000000C0)        /*!< TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_0               ((unsigned long)0x00000040)        /*!< Bit 0 */
#define AFIO_MAPR_TIM1_REMAP_1               ((unsigned long)0x00000080)        /*!< Bit 1 */

/*!< TIM1_REMAP configuration */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         ((unsigned long)0x00000000)        /*!< No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    ((unsigned long)0x00000040)        /*!< Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       ((unsigned long)0x000000C0)        /*!< Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP                 ((unsigned long)0x00000300)        /*!< TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define AFIO_MAPR_TIM2_REMAP_1               ((unsigned long)0x00000200)        /*!< Bit 1 */

/*!< TIM2_REMAP configuration */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         ((unsigned long)0x00000000)        /*!< No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   ((unsigned long)0x00000100)        /*!< Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   ((unsigned long)0x00000200)        /*!< Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       ((unsigned long)0x00000300)        /*!< Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP                 ((unsigned long)0x00000C00)        /*!< TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_0               ((unsigned long)0x00000400)        /*!< Bit 0 */
#define AFIO_MAPR_TIM3_REMAP_1               ((unsigned long)0x00000800)        /*!< Bit 1 */

/*!< TIM3_REMAP configuration */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         ((unsigned long)0x00000000)        /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    ((unsigned long)0x00000800)        /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       ((unsigned long)0x00000C00)        /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP                 ((unsigned long)0x00001000)        /*!< TIM4_REMAP bit (TIM4 remapping) */

#define AFIO_MAPR_CAN_REMAP                  ((unsigned long)0x00006000)        /*!< CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_0                ((unsigned long)0x00002000)        /*!< Bit 0 */
#define AFIO_MAPR_CAN_REMAP_1                ((unsigned long)0x00004000)        /*!< Bit 1 */

/*!< CAN_REMAP configuration */
#define AFIO_MAPR_CAN_REMAP_REMAP1           ((unsigned long)0x00000000)        /*!< CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2           ((unsigned long)0x00004000)        /*!< CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3           ((unsigned long)0x00006000)        /*!< CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP                 ((unsigned long)0x00008000)        /*!< Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP             ((unsigned long)0x00010000)        /*!< TIM5 Channel4 Internal Remap */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP         ((unsigned long)0x00020000)        /*!< ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP         ((unsigned long)0x00040000)        /*!< ADC 1 External Trigger Regular Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP         ((unsigned long)0x00080000)        /*!< ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP         ((unsigned long)0x00100000)        /*!< ADC 2 External Trigger Regular Conversion remapping */

/*!< SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG                    ((unsigned long)0x07000000)        /*!< SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_0                  ((unsigned long)0x01000000)        /*!< Bit 0 */
#define AFIO_MAPR_SWJ_CFG_1                  ((unsigned long)0x02000000)        /*!< Bit 1 */
#define AFIO_MAPR_SWJ_CFG_2                  ((unsigned long)0x04000000)        /*!< Bit 2 */

#define AFIO_MAPR_SWJ_CFG_RESET              ((unsigned long)0x00000000)        /*!< Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           ((unsigned long)0x01000000)        /*!< Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        ((unsigned long)0x02000000)        /*!< JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE            ((unsigned long)0x04000000)        /*!< JTAG-DP Disabled and SW-DP Disabled */

#ifdef STM32F10X_CL
/*!< ETH_REMAP configuration */
 #define AFIO_MAPR_ETH_REMAP                  ((unsigned long)0x00200000)        /*!< SPI3_REMAP bit (Ethernet MAC I/O remapping) */

/*!< CAN2_REMAP configuration */
 #define AFIO_MAPR_CAN2_REMAP                 ((unsigned long)0x00400000)        /*!< CAN2_REMAP bit (CAN2 I/O remapping) */

/*!< MII_RMII_SEL configuration */
 #define AFIO_MAPR_MII_RMII_SEL               ((unsigned long)0x00800000)        /*!< MII_RMII_SEL bit (Ethernet MII or RMII selection) */

/*!< SPI3_REMAP configuration */
 #define AFIO_MAPR_SPI3_REMAP                 ((unsigned long)0x10000000)        /*!< SPI3_REMAP bit (SPI3 remapping) */

/*!< TIM2ITR1_IREMAP configuration */
 #define AFIO_MAPR_TIM2ITR1_IREMAP            ((unsigned long)0x20000000)        /*!< TIM2ITR1_IREMAP bit (TIM2 internal trigger 1 remapping) */

/*!< PTP_PPS_REMAP configuration */
 #define AFIO_MAPR_PTP_PPS_REMAP              ((unsigned long)0x40000000)        /*!< PTP_PPS_REMAP bit (Ethernet PTP PPS remapping) */
#endif

/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0                   ((unsigned short)0x000F)            /*!< EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1                   ((unsigned short)0x00F0)            /*!< EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2                   ((unsigned short)0x0F00)            /*!< EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3                   ((unsigned short)0xF000)            /*!< EXTI 3 configuration */

/*!< EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                ((unsigned short)0x0000)            /*!< PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                ((unsigned short)0x0001)            /*!< PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                ((unsigned short)0x0002)            /*!< PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD                ((unsigned short)0x0003)            /*!< PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE                ((unsigned short)0x0004)            /*!< PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF                ((unsigned short)0x0005)            /*!< PF[0] pin */
#define AFIO_EXTICR1_EXTI0_PG                ((unsigned short)0x0006)            /*!< PG[0] pin */

/*!< EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                ((unsigned short)0x0000)            /*!< PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                ((unsigned short)0x0010)            /*!< PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                ((unsigned short)0x0020)            /*!< PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD                ((unsigned short)0x0030)            /*!< PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE                ((unsigned short)0x0040)            /*!< PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF                ((unsigned short)0x0050)            /*!< PF[1] pin */
#define AFIO_EXTICR1_EXTI1_PG                ((unsigned short)0x0060)            /*!< PG[1] pin */

/*!< EXTI2 configuration */
#define AFIO_EXTICR1_EXTI2_PA                ((unsigned short)0x0000)            /*!< PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                ((unsigned short)0x0100)            /*!< PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                ((unsigned short)0x0200)            /*!< PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD                ((unsigned short)0x0300)            /*!< PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE                ((unsigned short)0x0400)            /*!< PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF                ((unsigned short)0x0500)            /*!< PF[2] pin */
#define AFIO_EXTICR1_EXTI2_PG                ((unsigned short)0x0600)            /*!< PG[2] pin */

/*!< EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                ((unsigned short)0x0000)            /*!< PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                ((unsigned short)0x1000)            /*!< PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                ((unsigned short)0x2000)            /*!< PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD                ((unsigned short)0x3000)            /*!< PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE                ((unsigned short)0x4000)            /*!< PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF                ((unsigned short)0x5000)            /*!< PF[3] pin */
#define AFIO_EXTICR1_EXTI3_PG                ((unsigned short)0x6000)            /*!< PG[3] pin */

/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI4                   ((unsigned short)0x000F)            /*!< EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5                   ((unsigned short)0x00F0)            /*!< EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6                   ((unsigned short)0x0F00)            /*!< EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7                   ((unsigned short)0xF000)            /*!< EXTI 7 configuration */

/*!< EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                ((unsigned short)0x0000)            /*!< PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB                ((unsigned short)0x0001)            /*!< PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC                ((unsigned short)0x0002)            /*!< PC[4] pin */
#define AFIO_EXTICR2_EXTI4_PD                ((unsigned short)0x0003)            /*!< PD[4] pin */
#define AFIO_EXTICR2_EXTI4_PE                ((unsigned short)0x0004)            /*!< PE[4] pin */
#define AFIO_EXTICR2_EXTI4_PF                ((unsigned short)0x0005)            /*!< PF[4] pin */
#define AFIO_EXTICR2_EXTI4_PG                ((unsigned short)0x0006)            /*!< PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                ((unsigned short)0x0000)            /*!< PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB                ((unsigned short)0x0010)            /*!< PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC                ((unsigned short)0x0020)            /*!< PC[5] pin */
#define AFIO_EXTICR2_EXTI5_PD                ((unsigned short)0x0030)            /*!< PD[5] pin */
#define AFIO_EXTICR2_EXTI5_PE                ((unsigned short)0x0040)            /*!< PE[5] pin */
#define AFIO_EXTICR2_EXTI5_PF                ((unsigned short)0x0050)            /*!< PF[5] pin */
#define AFIO_EXTICR2_EXTI5_PG                ((unsigned short)0x0060)            /*!< PG[5] pin */

/*!< EXTI6 configuration */
#define AFIO_EXTICR2_EXTI6_PA                ((unsigned short)0x0000)            /*!< PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB                ((unsigned short)0x0100)            /*!< PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC                ((unsigned short)0x0200)            /*!< PC[6] pin */
#define AFIO_EXTICR2_EXTI6_PD                ((unsigned short)0x0300)            /*!< PD[6] pin */
#define AFIO_EXTICR2_EXTI6_PE                ((unsigned short)0x0400)            /*!< PE[6] pin */
#define AFIO_EXTICR2_EXTI6_PF                ((unsigned short)0x0500)            /*!< PF[6] pin */
#define AFIO_EXTICR2_EXTI6_PG                ((unsigned short)0x0600)            /*!< PG[6] pin */

/*!< EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                ((unsigned short)0x0000)            /*!< PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB                ((unsigned short)0x1000)            /*!< PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC                ((unsigned short)0x2000)            /*!< PC[7] pin */
#define AFIO_EXTICR2_EXTI7_PD                ((unsigned short)0x3000)            /*!< PD[7] pin */
#define AFIO_EXTICR2_EXTI7_PE                ((unsigned short)0x4000)            /*!< PE[7] pin */
#define AFIO_EXTICR2_EXTI7_PF                ((unsigned short)0x5000)            /*!< PF[7] pin */
#define AFIO_EXTICR2_EXTI7_PG                ((unsigned short)0x6000)            /*!< PG[7] pin */

/*****************  Bit definition for AFIO_EXTICR3 register  *****************/
#define AFIO_EXTICR3_EXTI8                   ((unsigned short)0x000F)            /*!< EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9                   ((unsigned short)0x00F0)            /*!< EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10                  ((unsigned short)0x0F00)            /*!< EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11                  ((unsigned short)0xF000)            /*!< EXTI 11 configuration */

/*!< EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                ((unsigned short)0x0000)            /*!< PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB                ((unsigned short)0x0001)            /*!< PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC                ((unsigned short)0x0002)            /*!< PC[8] pin */
#define AFIO_EXTICR3_EXTI8_PD                ((unsigned short)0x0003)            /*!< PD[8] pin */
#define AFIO_EXTICR3_EXTI8_PE                ((unsigned short)0x0004)            /*!< PE[8] pin */
#define AFIO_EXTICR3_EXTI8_PF                ((unsigned short)0x0005)            /*!< PF[8] pin */
#define AFIO_EXTICR3_EXTI8_PG                ((unsigned short)0x0006)            /*!< PG[8] pin */

/*!< EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                ((unsigned short)0x0000)            /*!< PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB                ((unsigned short)0x0010)            /*!< PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC                ((unsigned short)0x0020)            /*!< PC[9] pin */
#define AFIO_EXTICR3_EXTI9_PD                ((unsigned short)0x0030)            /*!< PD[9] pin */
#define AFIO_EXTICR3_EXTI9_PE                ((unsigned short)0x0040)            /*!< PE[9] pin */
#define AFIO_EXTICR3_EXTI9_PF                ((unsigned short)0x0050)            /*!< PF[9] pin */
#define AFIO_EXTICR3_EXTI9_PG                ((unsigned short)0x0060)            /*!< PG[9] pin */

/*!< EXTI10 configuration */
#define AFIO_EXTICR3_EXTI10_PA               ((unsigned short)0x0000)            /*!< PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB               ((unsigned short)0x0100)            /*!< PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC               ((unsigned short)0x0200)            /*!< PC[10] pin */
#define AFIO_EXTICR3_EXTI10_PD               ((unsigned short)0x0300)            /*!< PD[10] pin */
#define AFIO_EXTICR3_EXTI10_PE               ((unsigned short)0x0400)            /*!< PE[10] pin */
#define AFIO_EXTICR3_EXTI10_PF               ((unsigned short)0x0500)            /*!< PF[10] pin */
#define AFIO_EXTICR3_EXTI10_PG               ((unsigned short)0x0600)            /*!< PG[10] pin */

/*!< EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               ((unsigned short)0x0000)            /*!< PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB               ((unsigned short)0x1000)            /*!< PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC               ((unsigned short)0x2000)            /*!< PC[11] pin */
#define AFIO_EXTICR3_EXTI11_PD               ((unsigned short)0x3000)            /*!< PD[11] pin */
#define AFIO_EXTICR3_EXTI11_PE               ((unsigned short)0x4000)            /*!< PE[11] pin */
#define AFIO_EXTICR3_EXTI11_PF               ((unsigned short)0x5000)            /*!< PF[11] pin */
#define AFIO_EXTICR3_EXTI11_PG               ((unsigned short)0x6000)            /*!< PG[11] pin */

/*****************  Bit definition for AFIO_EXTICR4 register  *****************/
#define AFIO_EXTICR4_EXTI12                  ((unsigned short)0x000F)            /*!< EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13                  ((unsigned short)0x00F0)            /*!< EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14                  ((unsigned short)0x0F00)            /*!< EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15                  ((unsigned short)0xF000)            /*!< EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               ((unsigned short)0x0000)            /*!< PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB               ((unsigned short)0x0001)            /*!< PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC               ((unsigned short)0x0002)            /*!< PC[12] pin */
#define AFIO_EXTICR4_EXTI12_PD               ((unsigned short)0x0003)            /*!< PD[12] pin */
#define AFIO_EXTICR4_EXTI12_PE               ((unsigned short)0x0004)            /*!< PE[12] pin */
#define AFIO_EXTICR4_EXTI12_PF               ((unsigned short)0x0005)            /*!< PF[12] pin */
#define AFIO_EXTICR4_EXTI12_PG               ((unsigned short)0x0006)            /*!< PG[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               ((unsigned short)0x0000)            /*!< PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB               ((unsigned short)0x0010)            /*!< PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC               ((unsigned short)0x0020)            /*!< PC[13] pin */
#define AFIO_EXTICR4_EXTI13_PD               ((unsigned short)0x0030)            /*!< PD[13] pin */
#define AFIO_EXTICR4_EXTI13_PE               ((unsigned short)0x0040)            /*!< PE[13] pin */
#define AFIO_EXTICR4_EXTI13_PF               ((unsigned short)0x0050)            /*!< PF[13] pin */
#define AFIO_EXTICR4_EXTI13_PG               ((unsigned short)0x0060)            /*!< PG[13] pin */

/*!< EXTI14 configuration */
#define AFIO_EXTICR4_EXTI14_PA               ((unsigned short)0x0000)            /*!< PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB               ((unsigned short)0x0100)            /*!< PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC               ((unsigned short)0x0200)            /*!< PC[14] pin */
#define AFIO_EXTICR4_EXTI14_PD               ((unsigned short)0x0300)            /*!< PD[14] pin */
#define AFIO_EXTICR4_EXTI14_PE               ((unsigned short)0x0400)            /*!< PE[14] pin */
#define AFIO_EXTICR4_EXTI14_PF               ((unsigned short)0x0500)            /*!< PF[14] pin */
#define AFIO_EXTICR4_EXTI14_PG               ((unsigned short)0x0600)            /*!< PG[14] pin */

/*!< EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               ((unsigned short)0x0000)            /*!< PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB               ((unsigned short)0x1000)            /*!< PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC               ((unsigned short)0x2000)            /*!< PC[15] pin */
#define AFIO_EXTICR4_EXTI15_PD               ((unsigned short)0x3000)            /*!< PD[15] pin */
#define AFIO_EXTICR4_EXTI15_PE               ((unsigned short)0x4000)            /*!< PE[15] pin */
#define AFIO_EXTICR4_EXTI15_PF               ((unsigned short)0x5000)            /*!< PF[15] pin */
#define AFIO_EXTICR4_EXTI15_PG               ((unsigned short)0x6000)            /*!< PG[15] pin */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/******************  Bit definition for AFIO_MAPR2 register  ******************/
#define AFIO_MAPR2_TIM15_REMAP               ((unsigned long)0x00000001)        /*!< TIM15 remapping */
#define AFIO_MAPR2_TIM16_REMAP               ((unsigned long)0x00000002)        /*!< TIM16 remapping */
#define AFIO_MAPR2_TIM17_REMAP               ((unsigned long)0x00000004)        /*!< TIM17 remapping */
#define AFIO_MAPR2_CEC_REMAP                 ((unsigned long)0x00000008)        /*!< CEC remapping */
#define AFIO_MAPR2_TIM1_DMA_REMAP            ((unsigned long)0x00000010)        /*!< TIM1_DMA remapping */
#endif

#ifdef STM32F10X_HD_VL
#define AFIO_MAPR2_TIM13_REMAP               ((unsigned long)0x00000100)        /*!< TIM13 remapping */
#define AFIO_MAPR2_TIM14_REMAP               ((unsigned long)0x00000200)        /*!< TIM14 remapping */
#define AFIO_MAPR2_FSMC_NADV_REMAP           ((unsigned long)0x00000400)        /*!< FSMC NADV remapping */
#define AFIO_MAPR2_TIM67_DAC_DMA_REMAP       ((unsigned long)0x00000800)        /*!< TIM6/TIM7 and DAC DMA remapping */
#define AFIO_MAPR2_TIM12_REMAP               ((unsigned long)0x00001000)        /*!< TIM12 remapping */
#define AFIO_MAPR2_MISC_REMAP                ((unsigned long)0x00002000)        /*!< Miscellaneous remapping */
#endif

#ifdef STM32F10X_XL
/******************  Bit definition for AFIO_MAPR2 register  ******************/
#define AFIO_MAPR2_TIM9_REMAP                ((unsigned long)0x00000020)        /*!< TIM9 remapping */
#define AFIO_MAPR2_TIM10_REMAP               ((unsigned long)0x00000040)        /*!< TIM10 remapping */
#define AFIO_MAPR2_TIM11_REMAP               ((unsigned long)0x00000080)        /*!< TIM11 remapping */
#define AFIO_MAPR2_TIM13_REMAP               ((unsigned long)0x00000100)        /*!< TIM13 remapping */
#define AFIO_MAPR2_TIM14_REMAP               ((unsigned long)0x00000200)        /*!< TIM14 remapping */
#define AFIO_MAPR2_FSMC_NADV_REMAP           ((unsigned long)0x00000400)        /*!< FSMC NADV remapping */
#endif

/******************************************************************************/
/*                                                                            */
/*                               SystemTick                                   */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SysTick_CTRL register  *****************/
#define  SysTick_CTRL_ENABLE                 ((unsigned long)0x00000001)        /*!< Counter enable */
#define  SysTick_CTRL_TICKINT                ((unsigned long)0x00000002)        /*!< Counting down to 0 pends the SysTick handler */
#define  SysTick_CTRL_CLKSOURCE              ((unsigned long)0x00000004)        /*!< Clock source */
#define  SysTick_CTRL_COUNTFLAG              ((unsigned long)0x00010000)        /*!< Count Flag */

/*****************  Bit definition for SysTick_LOAD register  *****************/
#define  SysTick_LOAD_RELOAD                 ((unsigned long)0x00FFFFFF)        /*!< Value to load into the SysTick Current Value Register when the counter reaches 0 */

/*****************  Bit definition for SysTick_VAL register  ******************/
#define  SysTick_VAL_CURRENT                 ((unsigned long)0x00FFFFFF)        /*!< Current value at the time the register is accessed */

/*****************  Bit definition for SysTick_CALIB register  ****************/
#define  SysTick_CALIB_TENMS                 ((unsigned long)0x00FFFFFF)        /*!< Reload value to use for 10ms timing */
#define  SysTick_CALIB_SKEW                  ((unsigned long)0x40000000)        /*!< Calibration value is not exactly 10 ms */
#define  SysTick_CALIB_NOREF                 ((unsigned long)0x80000000)        /*!< The reference clock is not provided */

/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for NVIC_ISER register  *******************/
#define  NVIC_ISER_SETENA                    ((unsigned long)0xFFFFFFFF)        /*!< Interrupt set enable bits */
#define  NVIC_ISER_SETENA_0                  ((unsigned long)0x00000001)        /*!< bit 0 */
#define  NVIC_ISER_SETENA_1                  ((unsigned long)0x00000002)        /*!< bit 1 */
#define  NVIC_ISER_SETENA_2                  ((unsigned long)0x00000004)        /*!< bit 2 */
#define  NVIC_ISER_SETENA_3                  ((unsigned long)0x00000008)        /*!< bit 3 */
#define  NVIC_ISER_SETENA_4                  ((unsigned long)0x00000010)        /*!< bit 4 */
#define  NVIC_ISER_SETENA_5                  ((unsigned long)0x00000020)        /*!< bit 5 */
#define  NVIC_ISER_SETENA_6                  ((unsigned long)0x00000040)        /*!< bit 6 */
#define  NVIC_ISER_SETENA_7                  ((unsigned long)0x00000080)        /*!< bit 7 */
#define  NVIC_ISER_SETENA_8                  ((unsigned long)0x00000100)        /*!< bit 8 */
#define  NVIC_ISER_SETENA_9                  ((unsigned long)0x00000200)        /*!< bit 9 */
#define  NVIC_ISER_SETENA_10                 ((unsigned long)0x00000400)        /*!< bit 10 */
#define  NVIC_ISER_SETENA_11                 ((unsigned long)0x00000800)        /*!< bit 11 */
#define  NVIC_ISER_SETENA_12                 ((unsigned long)0x00001000)        /*!< bit 12 */
#define  NVIC_ISER_SETENA_13                 ((unsigned long)0x00002000)        /*!< bit 13 */
#define  NVIC_ISER_SETENA_14                 ((unsigned long)0x00004000)        /*!< bit 14 */
#define  NVIC_ISER_SETENA_15                 ((unsigned long)0x00008000)        /*!< bit 15 */
#define  NVIC_ISER_SETENA_16                 ((unsigned long)0x00010000)        /*!< bit 16 */
#define  NVIC_ISER_SETENA_17                 ((unsigned long)0x00020000)        /*!< bit 17 */
#define  NVIC_ISER_SETENA_18                 ((unsigned long)0x00040000)        /*!< bit 18 */
#define  NVIC_ISER_SETENA_19                 ((unsigned long)0x00080000)        /*!< bit 19 */
#define  NVIC_ISER_SETENA_20                 ((unsigned long)0x00100000)        /*!< bit 20 */
#define  NVIC_ISER_SETENA_21                 ((unsigned long)0x00200000)        /*!< bit 21 */
#define  NVIC_ISER_SETENA_22                 ((unsigned long)0x00400000)        /*!< bit 22 */
#define  NVIC_ISER_SETENA_23                 ((unsigned long)0x00800000)        /*!< bit 23 */
#define  NVIC_ISER_SETENA_24                 ((unsigned long)0x01000000)        /*!< bit 24 */
#define  NVIC_ISER_SETENA_25                 ((unsigned long)0x02000000)        /*!< bit 25 */
#define  NVIC_ISER_SETENA_26                 ((unsigned long)0x04000000)        /*!< bit 26 */
#define  NVIC_ISER_SETENA_27                 ((unsigned long)0x08000000)        /*!< bit 27 */
#define  NVIC_ISER_SETENA_28                 ((unsigned long)0x10000000)        /*!< bit 28 */
#define  NVIC_ISER_SETENA_29                 ((unsigned long)0x20000000)        /*!< bit 29 */
#define  NVIC_ISER_SETENA_30                 ((unsigned long)0x40000000)        /*!< bit 30 */
#define  NVIC_ISER_SETENA_31                 ((unsigned long)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICER register  *******************/
#define  NVIC_ICER_CLRENA                   ((unsigned long)0xFFFFFFFF)        /*!< Interrupt clear-enable bits */
#define  NVIC_ICER_CLRENA_0                  ((unsigned long)0x00000001)        /*!< bit 0 */
#define  NVIC_ICER_CLRENA_1                  ((unsigned long)0x00000002)        /*!< bit 1 */
#define  NVIC_ICER_CLRENA_2                  ((unsigned long)0x00000004)        /*!< bit 2 */
#define  NVIC_ICER_CLRENA_3                  ((unsigned long)0x00000008)        /*!< bit 3 */
#define  NVIC_ICER_CLRENA_4                  ((unsigned long)0x00000010)        /*!< bit 4 */
#define  NVIC_ICER_CLRENA_5                  ((unsigned long)0x00000020)        /*!< bit 5 */
#define  NVIC_ICER_CLRENA_6                  ((unsigned long)0x00000040)        /*!< bit 6 */
#define  NVIC_ICER_CLRENA_7                  ((unsigned long)0x00000080)        /*!< bit 7 */
#define  NVIC_ICER_CLRENA_8                  ((unsigned long)0x00000100)        /*!< bit 8 */
#define  NVIC_ICER_CLRENA_9                  ((unsigned long)0x00000200)        /*!< bit 9 */
#define  NVIC_ICER_CLRENA_10                 ((unsigned long)0x00000400)        /*!< bit 10 */
#define  NVIC_ICER_CLRENA_11                 ((unsigned long)0x00000800)        /*!< bit 11 */
#define  NVIC_ICER_CLRENA_12                 ((unsigned long)0x00001000)        /*!< bit 12 */
#define  NVIC_ICER_CLRENA_13                 ((unsigned long)0x00002000)        /*!< bit 13 */
#define  NVIC_ICER_CLRENA_14                 ((unsigned long)0x00004000)        /*!< bit 14 */
#define  NVIC_ICER_CLRENA_15                 ((unsigned long)0x00008000)        /*!< bit 15 */
#define  NVIC_ICER_CLRENA_16                 ((unsigned long)0x00010000)        /*!< bit 16 */
#define  NVIC_ICER_CLRENA_17                 ((unsigned long)0x00020000)        /*!< bit 17 */
#define  NVIC_ICER_CLRENA_18                 ((unsigned long)0x00040000)        /*!< bit 18 */
#define  NVIC_ICER_CLRENA_19                 ((unsigned long)0x00080000)        /*!< bit 19 */
#define  NVIC_ICER_CLRENA_20                 ((unsigned long)0x00100000)        /*!< bit 20 */
#define  NVIC_ICER_CLRENA_21                 ((unsigned long)0x00200000)        /*!< bit 21 */
#define  NVIC_ICER_CLRENA_22                 ((unsigned long)0x00400000)        /*!< bit 22 */
#define  NVIC_ICER_CLRENA_23                 ((unsigned long)0x00800000)        /*!< bit 23 */
#define  NVIC_ICER_CLRENA_24                 ((unsigned long)0x01000000)        /*!< bit 24 */
#define  NVIC_ICER_CLRENA_25                 ((unsigned long)0x02000000)        /*!< bit 25 */
#define  NVIC_ICER_CLRENA_26                 ((unsigned long)0x04000000)        /*!< bit 26 */
#define  NVIC_ICER_CLRENA_27                 ((unsigned long)0x08000000)        /*!< bit 27 */
#define  NVIC_ICER_CLRENA_28                 ((unsigned long)0x10000000)        /*!< bit 28 */
#define  NVIC_ICER_CLRENA_29                 ((unsigned long)0x20000000)        /*!< bit 29 */
#define  NVIC_ICER_CLRENA_30                 ((unsigned long)0x40000000)        /*!< bit 30 */
#define  NVIC_ICER_CLRENA_31                 ((unsigned long)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ISPR register  *******************/
#define  NVIC_ISPR_SETPEND                   ((unsigned long)0xFFFFFFFF)        /*!< Interrupt set-pending bits */
#define  NVIC_ISPR_SETPEND_0                 ((unsigned long)0x00000001)        /*!< bit 0 */
#define  NVIC_ISPR_SETPEND_1                 ((unsigned long)0x00000002)        /*!< bit 1 */
#define  NVIC_ISPR_SETPEND_2                 ((unsigned long)0x00000004)        /*!< bit 2 */
#define  NVIC_ISPR_SETPEND_3                 ((unsigned long)0x00000008)        /*!< bit 3 */
#define  NVIC_ISPR_SETPEND_4                 ((unsigned long)0x00000010)        /*!< bit 4 */
#define  NVIC_ISPR_SETPEND_5                 ((unsigned long)0x00000020)        /*!< bit 5 */
#define  NVIC_ISPR_SETPEND_6                 ((unsigned long)0x00000040)        /*!< bit 6 */
#define  NVIC_ISPR_SETPEND_7                 ((unsigned long)0x00000080)        /*!< bit 7 */
#define  NVIC_ISPR_SETPEND_8                 ((unsigned long)0x00000100)        /*!< bit 8 */
#define  NVIC_ISPR_SETPEND_9                 ((unsigned long)0x00000200)        /*!< bit 9 */
#define  NVIC_ISPR_SETPEND_10                ((unsigned long)0x00000400)        /*!< bit 10 */
#define  NVIC_ISPR_SETPEND_11                ((unsigned long)0x00000800)        /*!< bit 11 */
#define  NVIC_ISPR_SETPEND_12                ((unsigned long)0x00001000)        /*!< bit 12 */
#define  NVIC_ISPR_SETPEND_13                ((unsigned long)0x00002000)        /*!< bit 13 */
#define  NVIC_ISPR_SETPEND_14                ((unsigned long)0x00004000)        /*!< bit 14 */
#define  NVIC_ISPR_SETPEND_15                ((unsigned long)0x00008000)        /*!< bit 15 */
#define  NVIC_ISPR_SETPEND_16                ((unsigned long)0x00010000)        /*!< bit 16 */
#define  NVIC_ISPR_SETPEND_17                ((unsigned long)0x00020000)        /*!< bit 17 */
#define  NVIC_ISPR_SETPEND_18                ((unsigned long)0x00040000)        /*!< bit 18 */
#define  NVIC_ISPR_SETPEND_19                ((unsigned long)0x00080000)        /*!< bit 19 */
#define  NVIC_ISPR_SETPEND_20                ((unsigned long)0x00100000)        /*!< bit 20 */
#define  NVIC_ISPR_SETPEND_21                ((unsigned long)0x00200000)        /*!< bit 21 */
#define  NVIC_ISPR_SETPEND_22                ((unsigned long)0x00400000)        /*!< bit 22 */
#define  NVIC_ISPR_SETPEND_23                ((unsigned long)0x00800000)        /*!< bit 23 */
#define  NVIC_ISPR_SETPEND_24                ((unsigned long)0x01000000)        /*!< bit 24 */
#define  NVIC_ISPR_SETPEND_25                ((unsigned long)0x02000000)        /*!< bit 25 */
#define  NVIC_ISPR_SETPEND_26                ((unsigned long)0x04000000)        /*!< bit 26 */
#define  NVIC_ISPR_SETPEND_27                ((unsigned long)0x08000000)        /*!< bit 27 */
#define  NVIC_ISPR_SETPEND_28                ((unsigned long)0x10000000)        /*!< bit 28 */
#define  NVIC_ISPR_SETPEND_29                ((unsigned long)0x20000000)        /*!< bit 29 */
#define  NVIC_ISPR_SETPEND_30                ((unsigned long)0x40000000)        /*!< bit 30 */
#define  NVIC_ISPR_SETPEND_31                ((unsigned long)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICPR register  *******************/
#define  NVIC_ICPR_CLRPEND                   ((unsigned long)0xFFFFFFFF)        /*!< Interrupt clear-pending bits */
#define  NVIC_ICPR_CLRPEND_0                 ((unsigned long)0x00000001)        /*!< bit 0 */
#define  NVIC_ICPR_CLRPEND_1                 ((unsigned long)0x00000002)        /*!< bit 1 */
#define  NVIC_ICPR_CLRPEND_2                 ((unsigned long)0x00000004)        /*!< bit 2 */
#define  NVIC_ICPR_CLRPEND_3                 ((unsigned long)0x00000008)        /*!< bit 3 */
#define  NVIC_ICPR_CLRPEND_4                 ((unsigned long)0x00000010)        /*!< bit 4 */
#define  NVIC_ICPR_CLRPEND_5                 ((unsigned long)0x00000020)        /*!< bit 5 */
#define  NVIC_ICPR_CLRPEND_6                 ((unsigned long)0x00000040)        /*!< bit 6 */
#define  NVIC_ICPR_CLRPEND_7                 ((unsigned long)0x00000080)        /*!< bit 7 */
#define  NVIC_ICPR_CLRPEND_8                 ((unsigned long)0x00000100)        /*!< bit 8 */
#define  NVIC_ICPR_CLRPEND_9                 ((unsigned long)0x00000200)        /*!< bit 9 */
#define  NVIC_ICPR_CLRPEND_10                ((unsigned long)0x00000400)        /*!< bit 10 */
#define  NVIC_ICPR_CLRPEND_11                ((unsigned long)0x00000800)        /*!< bit 11 */
#define  NVIC_ICPR_CLRPEND_12                ((unsigned long)0x00001000)        /*!< bit 12 */
#define  NVIC_ICPR_CLRPEND_13                ((unsigned long)0x00002000)        /*!< bit 13 */
#define  NVIC_ICPR_CLRPEND_14                ((unsigned long)0x00004000)        /*!< bit 14 */
#define  NVIC_ICPR_CLRPEND_15                ((unsigned long)0x00008000)        /*!< bit 15 */
#define  NVIC_ICPR_CLRPEND_16                ((unsigned long)0x00010000)        /*!< bit 16 */
#define  NVIC_ICPR_CLRPEND_17                ((unsigned long)0x00020000)        /*!< bit 17 */
#define  NVIC_ICPR_CLRPEND_18                ((unsigned long)0x00040000)        /*!< bit 18 */
#define  NVIC_ICPR_CLRPEND_19                ((unsigned long)0x00080000)        /*!< bit 19 */
#define  NVIC_ICPR_CLRPEND_20                ((unsigned long)0x00100000)        /*!< bit 20 */
#define  NVIC_ICPR_CLRPEND_21                ((unsigned long)0x00200000)        /*!< bit 21 */
#define  NVIC_ICPR_CLRPEND_22                ((unsigned long)0x00400000)        /*!< bit 22 */
#define  NVIC_ICPR_CLRPEND_23                ((unsigned long)0x00800000)        /*!< bit 23 */
#define  NVIC_ICPR_CLRPEND_24                ((unsigned long)0x01000000)        /*!< bit 24 */
#define  NVIC_ICPR_CLRPEND_25                ((unsigned long)0x02000000)        /*!< bit 25 */
#define  NVIC_ICPR_CLRPEND_26                ((unsigned long)0x04000000)        /*!< bit 26 */
#define  NVIC_ICPR_CLRPEND_27                ((unsigned long)0x08000000)        /*!< bit 27 */
#define  NVIC_ICPR_CLRPEND_28                ((unsigned long)0x10000000)        /*!< bit 28 */
#define  NVIC_ICPR_CLRPEND_29                ((unsigned long)0x20000000)        /*!< bit 29 */
#define  NVIC_ICPR_CLRPEND_30                ((unsigned long)0x40000000)        /*!< bit 30 */
#define  NVIC_ICPR_CLRPEND_31                ((unsigned long)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_IABR register  *******************/
#define  NVIC_IABR_ACTIVE                    ((unsigned long)0xFFFFFFFF)        /*!< Interrupt active flags */
#define  NVIC_IABR_ACTIVE_0                  ((unsigned long)0x00000001)        /*!< bit 0 */
#define  NVIC_IABR_ACTIVE_1                  ((unsigned long)0x00000002)        /*!< bit 1 */
#define  NVIC_IABR_ACTIVE_2                  ((unsigned long)0x00000004)        /*!< bit 2 */
#define  NVIC_IABR_ACTIVE_3                  ((unsigned long)0x00000008)        /*!< bit 3 */
#define  NVIC_IABR_ACTIVE_4                  ((unsigned long)0x00000010)        /*!< bit 4 */
#define  NVIC_IABR_ACTIVE_5                  ((unsigned long)0x00000020)        /*!< bit 5 */
#define  NVIC_IABR_ACTIVE_6                  ((unsigned long)0x00000040)        /*!< bit 6 */
#define  NVIC_IABR_ACTIVE_7                  ((unsigned long)0x00000080)        /*!< bit 7 */
#define  NVIC_IABR_ACTIVE_8                  ((unsigned long)0x00000100)        /*!< bit 8 */
#define  NVIC_IABR_ACTIVE_9                  ((unsigned long)0x00000200)        /*!< bit 9 */
#define  NVIC_IABR_ACTIVE_10                 ((unsigned long)0x00000400)        /*!< bit 10 */
#define  NVIC_IABR_ACTIVE_11                 ((unsigned long)0x00000800)        /*!< bit 11 */
#define  NVIC_IABR_ACTIVE_12                 ((unsigned long)0x00001000)        /*!< bit 12 */
#define  NVIC_IABR_ACTIVE_13                 ((unsigned long)0x00002000)        /*!< bit 13 */
#define  NVIC_IABR_ACTIVE_14                 ((unsigned long)0x00004000)        /*!< bit 14 */
#define  NVIC_IABR_ACTIVE_15                 ((unsigned long)0x00008000)        /*!< bit 15 */
#define  NVIC_IABR_ACTIVE_16                 ((unsigned long)0x00010000)        /*!< bit 16 */
#define  NVIC_IABR_ACTIVE_17                 ((unsigned long)0x00020000)        /*!< bit 17 */
#define  NVIC_IABR_ACTIVE_18                 ((unsigned long)0x00040000)        /*!< bit 18 */
#define  NVIC_IABR_ACTIVE_19                 ((unsigned long)0x00080000)        /*!< bit 19 */
#define  NVIC_IABR_ACTIVE_20                 ((unsigned long)0x00100000)        /*!< bit 20 */
#define  NVIC_IABR_ACTIVE_21                 ((unsigned long)0x00200000)        /*!< bit 21 */
#define  NVIC_IABR_ACTIVE_22                 ((unsigned long)0x00400000)        /*!< bit 22 */
#define  NVIC_IABR_ACTIVE_23                 ((unsigned long)0x00800000)        /*!< bit 23 */
#define  NVIC_IABR_ACTIVE_24                 ((unsigned long)0x01000000)        /*!< bit 24 */
#define  NVIC_IABR_ACTIVE_25                 ((unsigned long)0x02000000)        /*!< bit 25 */
#define  NVIC_IABR_ACTIVE_26                 ((unsigned long)0x04000000)        /*!< bit 26 */
#define  NVIC_IABR_ACTIVE_27                 ((unsigned long)0x08000000)        /*!< bit 27 */
#define  NVIC_IABR_ACTIVE_28                 ((unsigned long)0x10000000)        /*!< bit 28 */
#define  NVIC_IABR_ACTIVE_29                 ((unsigned long)0x20000000)        /*!< bit 29 */
#define  NVIC_IABR_ACTIVE_30                 ((unsigned long)0x40000000)        /*!< bit 30 */
#define  NVIC_IABR_ACTIVE_31                 ((unsigned long)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_PRI0 register  *******************/
#define  NVIC_IPR0_PRI_0                     ((unsigned long)0x000000FF)        /*!< Priority of interrupt 0 */
#define  NVIC_IPR0_PRI_1                     ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 1 */
#define  NVIC_IPR0_PRI_2                     ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 2 */
#define  NVIC_IPR0_PRI_3                     ((unsigned long)0xFF000000)        /*!< Priority of interrupt 3 */

/******************  Bit definition for NVIC_PRI1 register  *******************/
#define  NVIC_IPR1_PRI_4                     ((unsigned long)0x000000FF)        /*!< Priority of interrupt 4 */
#define  NVIC_IPR1_PRI_5                     ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 5 */
#define  NVIC_IPR1_PRI_6                     ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 6 */
#define  NVIC_IPR1_PRI_7                     ((unsigned long)0xFF000000)        /*!< Priority of interrupt 7 */

/******************  Bit definition for NVIC_PRI2 register  *******************/
#define  NVIC_IPR2_PRI_8                     ((unsigned long)0x000000FF)        /*!< Priority of interrupt 8 */
#define  NVIC_IPR2_PRI_9                     ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 9 */
#define  NVIC_IPR2_PRI_10                    ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 10 */
#define  NVIC_IPR2_PRI_11                    ((unsigned long)0xFF000000)        /*!< Priority of interrupt 11 */

/******************  Bit definition for NVIC_PRI3 register  *******************/
#define  NVIC_IPR3_PRI_12                    ((unsigned long)0x000000FF)        /*!< Priority of interrupt 12 */
#define  NVIC_IPR3_PRI_13                    ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 13 */
#define  NVIC_IPR3_PRI_14                    ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 14 */
#define  NVIC_IPR3_PRI_15                    ((unsigned long)0xFF000000)        /*!< Priority of interrupt 15 */

/******************  Bit definition for NVIC_PRI4 register  *******************/
#define  NVIC_IPR4_PRI_16                    ((unsigned long)0x000000FF)        /*!< Priority of interrupt 16 */
#define  NVIC_IPR4_PRI_17                    ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 17 */
#define  NVIC_IPR4_PRI_18                    ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 18 */
#define  NVIC_IPR4_PRI_19                    ((unsigned long)0xFF000000)        /*!< Priority of interrupt 19 */

/******************  Bit definition for NVIC_PRI5 register  *******************/
#define  NVIC_IPR5_PRI_20                    ((unsigned long)0x000000FF)        /*!< Priority of interrupt 20 */
#define  NVIC_IPR5_PRI_21                    ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 21 */
#define  NVIC_IPR5_PRI_22                    ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 22 */
#define  NVIC_IPR5_PRI_23                    ((unsigned long)0xFF000000)        /*!< Priority of interrupt 23 */

/******************  Bit definition for NVIC_PRI6 register  *******************/
#define  NVIC_IPR6_PRI_24                    ((unsigned long)0x000000FF)        /*!< Priority of interrupt 24 */
#define  NVIC_IPR6_PRI_25                    ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 25 */
#define  NVIC_IPR6_PRI_26                    ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 26 */
#define  NVIC_IPR6_PRI_27                    ((unsigned long)0xFF000000)        /*!< Priority of interrupt 27 */

/******************  Bit definition for NVIC_PRI7 register  *******************/
#define  NVIC_IPR7_PRI_28                    ((unsigned long)0x000000FF)        /*!< Priority of interrupt 28 */
#define  NVIC_IPR7_PRI_29                    ((unsigned long)0x0000FF00)        /*!< Priority of interrupt 29 */
#define  NVIC_IPR7_PRI_30                    ((unsigned long)0x00FF0000)        /*!< Priority of interrupt 30 */
#define  NVIC_IPR7_PRI_31                    ((unsigned long)0xFF000000)        /*!< Priority of interrupt 31 */

/******************  Bit definition for SCB_CPUID register  *******************/
#define  SCB_CPUID_REVISION                  ((unsigned long)0x0000000F)        /*!< Implementation defined revision number */
#define  SCB_CPUID_PARTNO                    ((unsigned long)0x0000FFF0)        /*!< Number of processor within family */
#define  SCB_CPUID_Constant                  ((unsigned long)0x000F0000)        /*!< Reads as 0x0F */
#define  SCB_CPUID_VARIANT                   ((unsigned long)0x00F00000)        /*!< Implementation defined variant number */
#define  SCB_CPUID_IMPLEMENTER               ((unsigned long)0xFF000000)        /*!< Implementer code. ARM is 0x41 */

/*******************  Bit definition for SCB_ICSR register  *******************/
#define  SCB_ICSR_VECTACTIVE                 ((unsigned long)0x000001FF)        /*!< Active ISR number field */
#define  SCB_ICSR_RETTOBASE                  ((unsigned long)0x00000800)        /*!< All active exceptions minus the IPSR_current_exception yields the empty set */
#define  SCB_ICSR_VECTPENDING                ((unsigned long)0x003FF000)        /*!< Pending ISR number field */
#define  SCB_ICSR_ISRPENDING                 ((unsigned long)0x00400000)        /*!< Interrupt pending flag */
#define  SCB_ICSR_ISRPREEMPT                 ((unsigned long)0x00800000)        /*!< It indicates that a pending interrupt becomes active in the next running cycle */
#define  SCB_ICSR_PENDSTCLR                  ((unsigned long)0x02000000)        /*!< Clear pending SysTick bit */
#define  SCB_ICSR_PENDSTSET                  ((unsigned long)0x04000000)        /*!< Set pending SysTick bit */
#define  SCB_ICSR_PENDSVCLR                  ((unsigned long)0x08000000)        /*!< Clear pending pendSV bit */
#define  SCB_ICSR_PENDSVSET                  ((unsigned long)0x10000000)        /*!< Set pending pendSV bit */
#define  SCB_ICSR_NMIPENDSET                 ((unsigned long)0x80000000)        /*!< Set pending NMI bit */

/*******************  Bit definition for SCB_VTOR register  *******************/
#define  SCB_VTOR_TBLOFF                     ((unsigned long)0x1FFFFF80)        /*!< Vector table base offset field */
#define  SCB_VTOR_TBLBASE                    ((unsigned long)0x20000000)        /*!< Table base in code(0) or RAM(1) */

/*!<*****************  Bit definition for SCB_AIRCR register  *******************/
#define  SCB_AIRCR_VECTRESET                 ((unsigned long)0x00000001)        /*!< System Reset bit */
#define  SCB_AIRCR_VECTCLRACTIVE             ((unsigned long)0x00000002)        /*!< Clear active vector bit */
#define  SCB_AIRCR_SYSRESETREQ               ((unsigned long)0x00000004)        /*!< Requests chip control logic to generate a reset */

#define  SCB_AIRCR_PRIGROUP                  ((unsigned long)0x00000700)        /*!< PRIGROUP[2:0] bits (Priority group) */
#define  SCB_AIRCR_PRIGROUP_0                ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  SCB_AIRCR_PRIGROUP_1                ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  SCB_AIRCR_PRIGROUP_2                ((unsigned long)0x00000400)        /*!< Bit 2  */

/* prority group configuration */
#define  SCB_AIRCR_PRIGROUP0                 ((unsigned long)0x00000000)        /*!< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority) */
#define  SCB_AIRCR_PRIGROUP1                 ((unsigned long)0x00000100)        /*!< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP2                 ((unsigned long)0x00000200)        /*!< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP3                 ((unsigned long)0x00000300)        /*!< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP4                 ((unsigned long)0x00000400)        /*!< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP5                 ((unsigned long)0x00000500)        /*!< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP6                 ((unsigned long)0x00000600)        /*!< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP7                 ((unsigned long)0x00000700)        /*!< Priority group=7 (no pre-emption priority, 8 bits of subpriority) */

#define  SCB_AIRCR_ENDIANESS                 ((unsigned long)0x00008000)        /*!< Data endianness bit */
#define  SCB_AIRCR_VECTKEY                   ((unsigned long)0xFFFF0000)        /*!< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) */

/*******************  Bit definition for SCB_SCR register  ********************/
#define  SCB_SCR_SLEEPONEXIT                 ((unsigned char)0x02)               /*!< Sleep on exit bit */
#define  SCB_SCR_SLEEPDEEP                   ((unsigned char)0x04)               /*!< Sleep deep bit */
#define  SCB_SCR_SEVONPEND                   ((unsigned char)0x10)               /*!< Wake up from WFE */

/********************  Bit definition for SCB_CCR register  *******************/
#define  SCB_CCR_NONBASETHRDENA              ((unsigned short)0x0001)            /*!< Thread mode can be entered from any level in Handler mode by controlled return value */
#define  SCB_CCR_USERSETMPEND                ((unsigned short)0x0002)            /*!< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception */
#define  SCB_CCR_UNALIGN_TRP                 ((unsigned short)0x0008)            /*!< Trap for unaligned access */
#define  SCB_CCR_DIV_0_TRP                   ((unsigned short)0x0010)            /*!< Trap on Divide by 0 */
#define  SCB_CCR_BFHFNMIGN                   ((unsigned short)0x0100)            /*!< Handlers running at priority -1 and -2 */
#define  SCB_CCR_STKALIGN                    ((unsigned short)0x0200)            /*!< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */

/*******************  Bit definition for SCB_SHPR register ********************/
#define  SCB_SHPR_PRI_N                      ((unsigned long)0x000000FF)        /*!< Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
#define  SCB_SHPR_PRI_N1                     ((unsigned long)0x0000FF00)        /*!< Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
#define  SCB_SHPR_PRI_N2                     ((unsigned long)0x00FF0000)        /*!< Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
#define  SCB_SHPR_PRI_N3                     ((unsigned long)0xFF000000)        /*!< Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick */

/******************  Bit definition for SCB_SHCSR register  *******************/
#define  SCB_SHCSR_MEMFAULTACT               ((unsigned long)0x00000001)        /*!< MemManage is active */
#define  SCB_SHCSR_BUSFAULTACT               ((unsigned long)0x00000002)        /*!< BusFault is active */
#define  SCB_SHCSR_USGFAULTACT               ((unsigned long)0x00000008)        /*!< UsageFault is active */
#define  SCB_SHCSR_SVCALLACT                 ((unsigned long)0x00000080)        /*!< SVCall is active */
#define  SCB_SHCSR_MONITORACT                ((unsigned long)0x00000100)        /*!< Monitor is active */
#define  SCB_SHCSR_PENDSVACT                 ((unsigned long)0x00000400)        /*!< PendSV is active */
#define  SCB_SHCSR_SYSTICKACT                ((unsigned long)0x00000800)        /*!< SysTick is active */
#define  SCB_SHCSR_USGFAULTPENDED            ((unsigned long)0x00001000)        /*!< Usage Fault is pended */
#define  SCB_SHCSR_MEMFAULTPENDED            ((unsigned long)0x00002000)        /*!< MemManage is pended */
#define  SCB_SHCSR_BUSFAULTPENDED            ((unsigned long)0x00004000)        /*!< Bus Fault is pended */
#define  SCB_SHCSR_SVCALLPENDED              ((unsigned long)0x00008000)        /*!< SVCall is pended */
#define  SCB_SHCSR_MEMFAULTENA               ((unsigned long)0x00010000)        /*!< MemManage enable */
#define  SCB_SHCSR_BUSFAULTENA               ((unsigned long)0x00020000)        /*!< Bus Fault enable */
#define  SCB_SHCSR_USGFAULTENA               ((unsigned long)0x00040000)        /*!< UsageFault enable */

/*******************  Bit definition for SCB_CFSR register  *******************/
/*!< MFSR */
#define  SCB_CFSR_IACCVIOL                   ((unsigned long)0x00000001)        /*!< Instruction access violation */
#define  SCB_CFSR_DACCVIOL                   ((unsigned long)0x00000002)        /*!< Data access violation */
#define  SCB_CFSR_MUNSTKERR                  ((unsigned long)0x00000008)        /*!< Unstacking error */
#define  SCB_CFSR_MSTKERR                    ((unsigned long)0x00000010)        /*!< Stacking error */
#define  SCB_CFSR_MMARVALID                  ((unsigned long)0x00000080)        /*!< Memory Manage Address Register address valid flag */
/*!< BFSR */
#define  SCB_CFSR_IBUSERR                    ((unsigned long)0x00000100)        /*!< Instruction bus error flag */
#define  SCB_CFSR_PRECISERR                  ((unsigned long)0x00000200)        /*!< Precise data bus error */
#define  SCB_CFSR_IMPRECISERR                ((unsigned long)0x00000400)        /*!< Imprecise data bus error */
#define  SCB_CFSR_UNSTKERR                   ((unsigned long)0x00000800)        /*!< Unstacking error */
#define  SCB_CFSR_STKERR                     ((unsigned long)0x00001000)        /*!< Stacking error */
#define  SCB_CFSR_BFARVALID                  ((unsigned long)0x00008000)        /*!< Bus Fault Address Register address valid flag */
/*!< UFSR */
#define  SCB_CFSR_UNDEFINSTR                 ((unsigned long)0x00010000)        /*!< The processor attempt to execute an undefined instruction */
#define  SCB_CFSR_INVSTATE                   ((unsigned long)0x00020000)        /*!< Invalid combination of EPSR and instruction */
#define  SCB_CFSR_INVPC                      ((unsigned long)0x00040000)        /*!< Attempt to load EXC_RETURN into pc illegally */
#define  SCB_CFSR_NOCP                       ((unsigned long)0x00080000)        /*!< Attempt to use a coprocessor instruction */
#define  SCB_CFSR_UNALIGNED                  ((unsigned long)0x01000000)        /*!< Fault occurs when there is an attempt to make an unaligned memory access */
#define  SCB_CFSR_DIVBYZERO                  ((unsigned long)0x02000000)        /*!< Fault occurs when SDIV or DIV instruction is used with a divisor of 0 */

/*******************  Bit definition for SCB_HFSR register  *******************/
#define  SCB_HFSR_VECTTBL                    ((unsigned long)0x00000002)        /*!< Fault occurs because of vector table read on exception processing */
#define  SCB_HFSR_FORCED                     ((unsigned long)0x40000000)        /*!< Hard Fault activated when a configurable Fault was received and cannot activate */
#define  SCB_HFSR_DEBUGEVT                   ((unsigned long)0x80000000)        /*!< Fault related to debug */

/*******************  Bit definition for SCB_DFSR register  *******************/
#define  SCB_DFSR_HALTED                     ((unsigned char)0x01)               /*!< Halt request flag */
#define  SCB_DFSR_BKPT                       ((unsigned char)0x02)               /*!< BKPT flag */
#define  SCB_DFSR_DWTTRAP                    ((unsigned char)0x04)               /*!< Data Watchpoint and Trace (DWT) flag */
#define  SCB_DFSR_VCATCH                     ((unsigned char)0x08)               /*!< Vector catch flag */
#define  SCB_DFSR_EXTERNAL                   ((unsigned char)0x10)               /*!< External debug request flag */

/*******************  Bit definition for SCB_MMFAR register  ******************/
#define  SCB_MMFAR_ADDRESS                   ((unsigned long)0xFFFFFFFF)        /*!< Mem Manage fault address field */

/*******************  Bit definition for SCB_BFAR register  *******************/
#define  SCB_BFAR_ADDRESS                    ((unsigned long)0xFFFFFFFF)        /*!< Bus fault address field */

/*******************  Bit definition for SCB_afsr register  *******************/
#define  SCB_AFSR_IMPDEF                     ((unsigned long)0xFFFFFFFF)        /*!< Implementation defined */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((unsigned long)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((unsigned long)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((unsigned long)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((unsigned long)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((unsigned long)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((unsigned long)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((unsigned long)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((unsigned long)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((unsigned long)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((unsigned long)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((unsigned long)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((unsigned long)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((unsigned long)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((unsigned long)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((unsigned long)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((unsigned long)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((unsigned long)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((unsigned long)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((unsigned long)0x00040000)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       ((unsigned long)0x00080000)        /*!< Interrupt Mask on line 19 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((unsigned long)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((unsigned long)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((unsigned long)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((unsigned long)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((unsigned long)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((unsigned long)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((unsigned long)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((unsigned long)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((unsigned long)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((unsigned long)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((unsigned long)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((unsigned long)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((unsigned long)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((unsigned long)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((unsigned long)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((unsigned long)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((unsigned long)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((unsigned long)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((unsigned long)0x00040000)        /*!< Event Mask on line 18 */
#define  EXTI_EMR_MR19                       ((unsigned long)0x00080000)        /*!< Event Mask on line 19 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((unsigned long)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((unsigned long)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((unsigned long)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((unsigned long)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((unsigned long)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((unsigned long)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((unsigned long)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((unsigned long)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((unsigned long)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((unsigned long)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((unsigned long)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((unsigned long)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((unsigned long)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((unsigned long)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((unsigned long)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((unsigned long)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((unsigned long)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((unsigned long)0x00020000)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((unsigned long)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      ((unsigned long)0x00080000)        /*!< Rising trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((unsigned long)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((unsigned long)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((unsigned long)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((unsigned long)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((unsigned long)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((unsigned long)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((unsigned long)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((unsigned long)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((unsigned long)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((unsigned long)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((unsigned long)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((unsigned long)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((unsigned long)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((unsigned long)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((unsigned long)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((unsigned long)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((unsigned long)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((unsigned long)0x00020000)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((unsigned long)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      ((unsigned long)0x00080000)        /*!< Falling trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((unsigned long)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((unsigned long)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((unsigned long)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((unsigned long)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((unsigned long)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((unsigned long)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((unsigned long)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((unsigned long)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((unsigned long)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((unsigned long)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((unsigned long)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((unsigned long)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((unsigned long)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((unsigned long)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((unsigned long)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((unsigned long)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((unsigned long)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((unsigned long)0x00020000)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((unsigned long)0x00040000)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  ((unsigned long)0x00080000)        /*!< Software Interrupt on line 19 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((unsigned long)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                         ((unsigned long)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                         ((unsigned long)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                         ((unsigned long)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                         ((unsigned long)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                         ((unsigned long)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                         ((unsigned long)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                         ((unsigned long)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                         ((unsigned long)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                         ((unsigned long)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10                        ((unsigned long)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11                        ((unsigned long)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12                        ((unsigned long)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13                        ((unsigned long)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14                        ((unsigned long)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15                        ((unsigned long)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16                        ((unsigned long)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17                        ((unsigned long)0x00020000)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18                        ((unsigned long)0x00040000)        /*!< Pending bit for line 18 */
#define  EXTI_PR_PR19                        ((unsigned long)0x00080000)        /*!< Pending bit for line 19 */

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((unsigned long)0x00000001)        /*!< Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((unsigned long)0x00000002)        /*!< Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((unsigned long)0x00000004)        /*!< Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((unsigned long)0x00000008)        /*!< Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((unsigned long)0x00000010)        /*!< Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((unsigned long)0x00000020)        /*!< Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((unsigned long)0x00000040)        /*!< Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((unsigned long)0x00000080)        /*!< Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((unsigned long)0x00000100)        /*!< Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((unsigned long)0x00000200)        /*!< Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((unsigned long)0x00000400)        /*!< Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((unsigned long)0x00000800)        /*!< Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((unsigned long)0x00001000)        /*!< Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((unsigned long)0x00002000)        /*!< Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((unsigned long)0x00004000)        /*!< Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((unsigned long)0x00008000)        /*!< Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((unsigned long)0x00010000)        /*!< Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((unsigned long)0x00020000)        /*!< Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((unsigned long)0x00040000)        /*!< Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((unsigned long)0x00080000)        /*!< Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((unsigned long)0x00100000)        /*!< Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((unsigned long)0x00200000)        /*!< Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((unsigned long)0x00400000)        /*!< Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((unsigned long)0x00800000)        /*!< Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((unsigned long)0x01000000)        /*!< Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((unsigned long)0x02000000)        /*!< Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((unsigned long)0x04000000)        /*!< Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((unsigned long)0x08000000)        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((unsigned long)0x00000001)        /*!< Channel 1 Global interrupt clear */
#define  DMA_IFCR_CTCIF1                     ((unsigned long)0x00000002)        /*!< Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((unsigned long)0x00000004)        /*!< Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((unsigned long)0x00000008)        /*!< Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((unsigned long)0x00000010)        /*!< Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((unsigned long)0x00000020)        /*!< Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((unsigned long)0x00000040)        /*!< Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((unsigned long)0x00000080)        /*!< Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((unsigned long)0x00000100)        /*!< Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((unsigned long)0x00000200)        /*!< Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((unsigned long)0x00000400)        /*!< Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((unsigned long)0x00000800)        /*!< Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((unsigned long)0x00001000)        /*!< Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((unsigned long)0x00002000)        /*!< Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((unsigned long)0x00004000)        /*!< Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((unsigned long)0x00008000)        /*!< Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((unsigned long)0x00010000)        /*!< Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((unsigned long)0x00020000)        /*!< Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((unsigned long)0x00040000)        /*!< Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((unsigned long)0x00080000)        /*!< Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((unsigned long)0x00100000)        /*!< Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((unsigned long)0x00200000)        /*!< Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((unsigned long)0x00400000)        /*!< Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((unsigned long)0x00800000)        /*!< Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((unsigned long)0x01000000)        /*!< Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((unsigned long)0x02000000)        /*!< Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((unsigned long)0x04000000)        /*!< Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((unsigned long)0x08000000)        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR1 register  *******************/
#define  DMA_CCR1_EN                         ((unsigned short)0x0001)            /*!< Channel enable*/
#define  DMA_CCR1_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR1_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR1_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR1_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR1_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR1_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR1_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR1_PSIZE                      ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR1_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR1_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR1_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR1_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR1_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR1_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits(Channel Priority level) */
#define  DMA_CCR1_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR1_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR1_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR2 register  *******************/
#define  DMA_CCR2_EN                         ((unsigned short)0x0001)            /*!< Channel enable */
#define  DMA_CCR2_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR2_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR2_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR2_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR2_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR2_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR2_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR2_PSIZE                      ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR2_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR2_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR2_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR2_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR2_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR2_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR2_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR2_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR2_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR3 register  *******************/
#define  DMA_CCR3_EN                         ((unsigned short)0x0001)            /*!< Channel enable */
#define  DMA_CCR3_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR3_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR3_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR3_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR3_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR3_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR3_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR3_PSIZE                      ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR3_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR3_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR3_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR3_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR3_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR3_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR3_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR3_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR3_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode */

/*!<******************  Bit definition for DMA_CCR4 register  *******************/
#define  DMA_CCR4_EN                         ((unsigned short)0x0001)            /*!< Channel enable */
#define  DMA_CCR4_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR4_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR4_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR4_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR4_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR4_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR4_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR4_PSIZE                      ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR4_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR4_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR4_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR4_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR4_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR4_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR4_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR4_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR4_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode */

/******************  Bit definition for DMA_CCR5 register  *******************/
#define  DMA_CCR5_EN                         ((unsigned short)0x0001)            /*!< Channel enable */
#define  DMA_CCR5_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR5_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR5_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR5_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR5_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR5_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR5_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR5_PSIZE                      ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR5_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR5_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR5_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR5_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR5_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR5_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR5_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR5_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR5_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode enable */

/*******************  Bit definition for DMA_CCR6 register  *******************/
#define  DMA_CCR6_EN                         ((unsigned short)0x0001)            /*!< Channel enable */
#define  DMA_CCR6_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR6_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR6_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR6_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR6_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR6_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR6_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR6_PSIZE                      ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR6_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR6_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR6_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR6_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR6_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR6_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR6_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR6_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR6_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR7 register  *******************/
#define  DMA_CCR7_EN                         ((unsigned short)0x0001)            /*!< Channel enable */
#define  DMA_CCR7_TCIE                       ((unsigned short)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR7_HTIE                       ((unsigned short)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR7_TEIE                       ((unsigned short)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR7_DIR                        ((unsigned short)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR7_CIRC                       ((unsigned short)0x0020)            /*!< Circular mode */
#define  DMA_CCR7_PINC                       ((unsigned short)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR7_MINC                       ((unsigned short)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR7_PSIZE            ,         ((unsigned short)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR7_PSIZE_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  DMA_CCR7_PSIZE_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  DMA_CCR7_MSIZE                      ((unsigned short)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR7_MSIZE_0                    ((unsigned short)0x0400)            /*!< Bit 0 */
#define  DMA_CCR7_MSIZE_1                    ((unsigned short)0x0800)            /*!< Bit 1 */

#define  DMA_CCR7_PL                         ((unsigned short)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR7_PL_0                       ((unsigned short)0x1000)            /*!< Bit 0 */
#define  DMA_CCR7_PL_1                       ((unsigned short)0x2000)            /*!< Bit 1 */

#define  DMA_CCR7_MEM2MEM                    ((unsigned short)0x4000)            /*!< Memory to memory mode enable */

/******************  Bit definition for DMA_CNDTR1 register  ******************/
#define  DMA_CNDTR1_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR2 register  ******************/
#define  DMA_CNDTR2_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR3 register  ******************/
#define  DMA_CNDTR3_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR4 register  ******************/
#define  DMA_CNDTR4_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR5 register  ******************/
#define  DMA_CNDTR5_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR6 register  ******************/
#define  DMA_CNDTR6_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR7 register  ******************/
#define  DMA_CNDTR7_NDT                      ((unsigned short)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CPAR1 register  *******************/
#define  DMA_CPAR1_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR2 register  *******************/
#define  DMA_CPAR2_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR3 register  *******************/
#define  DMA_CPAR3_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */


/******************  Bit definition for DMA_CPAR4 register  *******************/
#define  DMA_CPAR4_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR5 register  *******************/
#define  DMA_CPAR5_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR6 register  *******************/
#define  DMA_CPAR6_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */


/******************  Bit definition for DMA_CPAR7 register  *******************/
#define  DMA_CPAR7_PA                        ((unsigned long)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CMAR1 register  *******************/
#define  DMA_CMAR1_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR2 register  *******************/
#define  DMA_CMAR2_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR3 register  *******************/
#define  DMA_CMAR3_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */


/******************  Bit definition for DMA_CMAR4 register  *******************/
#define  DMA_CMAR4_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR5 register  *******************/
#define  DMA_CMAR5_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR6 register  *******************/
#define  DMA_CMAR6_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR7 register  *******************/
#define  DMA_CMAR7_MA                        ((unsigned long)0xFFFFFFFF)        /*!< Memory Address */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((unsigned char)0x01)               /*!< Analog watchdog flag */
#define  ADC_SR_EOC                          ((unsigned char)0x02)               /*!< End of conversion */
#define  ADC_SR_JEOC                         ((unsigned char)0x04)               /*!< Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((unsigned char)0x08)               /*!< Injected channel Start flag */
#define  ADC_SR_STRT                         ((unsigned char)0x10)               /*!< Regular channel Start flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((unsigned long)0x0000001F)        /*!< AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((unsigned long)0x00000010)        /*!< Bit 4 */

#define  ADC_CR1_EOCIE                       ((unsigned long)0x00000020)        /*!< Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((unsigned long)0x00000040)        /*!< Analog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((unsigned long)0x00000080)        /*!< Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((unsigned long)0x00000100)        /*!< Scan mode */
#define  ADC_CR1_AWDSGL                      ((unsigned long)0x00000200)        /*!< Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((unsigned long)0x00000400)        /*!< Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((unsigned long)0x00000800)        /*!< Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((unsigned long)0x00001000)        /*!< Discontinuous mode on injected channels */

#define  ADC_CR1_DISCNUM                     ((unsigned long)0x0000E000)        /*!< DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((unsigned long)0x00002000)        /*!< Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((unsigned long)0x00004000)        /*!< Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((unsigned long)0x00008000)        /*!< Bit 2 */

#define  ADC_CR1_DUALMOD                     ((unsigned long)0x000F0000)        /*!< DUALMOD[3:0] bits (Dual mode selection) */
#define  ADC_CR1_DUALMOD_0                   ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  ADC_CR1_DUALMOD_1                   ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  ADC_CR1_DUALMOD_2                   ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  ADC_CR1_DUALMOD_3                   ((unsigned long)0x00080000)        /*!< Bit 3 */

#define  ADC_CR1_JAWDEN                      ((unsigned long)0x00400000)        /*!< Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((unsigned long)0x00800000)        /*!< Analog watchdog enable on regular channels */


/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((unsigned long)0x00000001)        /*!< A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((unsigned long)0x00000002)        /*!< Continuous Conversion */
#define  ADC_CR2_CAL                         ((unsigned long)0x00000004)        /*!< A/D Calibration */
#define  ADC_CR2_RSTCAL                      ((unsigned long)0x00000008)        /*!< Reset Calibration */
#define  ADC_CR2_DMA                         ((unsigned long)0x00000100)        /*!< Direct Memory access mode */
#define  ADC_CR2_ALIGN                       ((unsigned long)0x00000800)        /*!< Data Alignment */

#define  ADC_CR2_JEXTSEL                     ((unsigned long)0x00007000)        /*!< JEXTSEL[2:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((unsigned long)0x00001000)        /*!< Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((unsigned long)0x00002000)        /*!< Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((unsigned long)0x00004000)        /*!< Bit 2 */

#define  ADC_CR2_JEXTTRIG                    ((unsigned long)0x00008000)        /*!< External Trigger Conversion mode for injected channels */

#define  ADC_CR2_EXTSEL                      ((unsigned long)0x000E0000)        /*!< EXTSEL[2:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((unsigned long)0x00020000)        /*!< Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((unsigned long)0x00040000)        /*!< Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((unsigned long)0x00080000)        /*!< Bit 2 */

#define  ADC_CR2_EXTTRIG                     ((unsigned long)0x00100000)        /*!< External Trigger Conversion mode for regular channels */
#define  ADC_CR2_JSWSTART                    ((unsigned long)0x00200000)        /*!< Start Conversion of injected channels */
#define  ADC_CR2_SWSTART                     ((unsigned long)0x00400000)        /*!< Start Conversion of regular channels */
#define  ADC_CR2_TSVREFE                     ((unsigned long)0x00800000)        /*!< Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((unsigned long)0x00000007)        /*!< SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((unsigned long)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP11                     ((unsigned long)0x00000038)        /*!< SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((unsigned long)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((unsigned long)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((unsigned long)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP12                     ((unsigned long)0x000001C0)        /*!< SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((unsigned long)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((unsigned long)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((unsigned long)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP13                     ((unsigned long)0x00000E00)        /*!< SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((unsigned long)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((unsigned long)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((unsigned long)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP14                     ((unsigned long)0x00007000)        /*!< SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((unsigned long)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((unsigned long)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((unsigned long)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP15                     ((unsigned long)0x00038000)        /*!< SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((unsigned long)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((unsigned long)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((unsigned long)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP16                     ((unsigned long)0x001C0000)        /*!< SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((unsigned long)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((unsigned long)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((unsigned long)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP17                     ((unsigned long)0x00E00000)        /*!< SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((unsigned long)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((unsigned long)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((unsigned long)0x00800000)        /*!< Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((unsigned long)0x00000007)        /*!< SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((unsigned long)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP1                      ((unsigned long)0x00000038)        /*!< SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((unsigned long)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((unsigned long)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((unsigned long)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP2                      ((unsigned long)0x000001C0)        /*!< SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((unsigned long)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((unsigned long)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((unsigned long)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP3                      ((unsigned long)0x00000E00)        /*!< SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((unsigned long)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((unsigned long)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((unsigned long)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP4                      ((unsigned long)0x00007000)        /*!< SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((unsigned long)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((unsigned long)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((unsigned long)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP5                      ((unsigned long)0x00038000)        /*!< SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((unsigned long)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((unsigned long)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((unsigned long)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP6                      ((unsigned long)0x001C0000)        /*!< SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((unsigned long)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((unsigned long)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((unsigned long)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP7                      ((unsigned long)0x00E00000)        /*!< SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((unsigned long)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((unsigned long)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((unsigned long)0x00800000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP8                      ((unsigned long)0x07000000)        /*!< SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((unsigned long)0x04000000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP9                      ((unsigned long)0x38000000)        /*!< SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((unsigned long)0x08000000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((unsigned long)0x10000000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((unsigned long)0x20000000)        /*!< Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((unsigned short)0x0FFF)            /*!< Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((unsigned short)0x0FFF)            /*!< Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((unsigned short)0x0FFF)            /*!< Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((unsigned short)0x0FFF)            /*!< Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((unsigned short)0x0FFF)            /*!< Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((unsigned short)0x0FFF)            /*!< Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((unsigned long)0x0000001F)        /*!< SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((unsigned long)0x00000010)        /*!< Bit 4 */

#define  ADC_SQR1_SQ14                       ((unsigned long)0x000003E0)        /*!< SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((unsigned long)0x00000020)        /*!< Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((unsigned long)0x00000040)        /*!< Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((unsigned long)0x00000080)        /*!< Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((unsigned long)0x00000100)        /*!< Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((unsigned long)0x00000200)        /*!< Bit 4 */

#define  ADC_SQR1_SQ15                       ((unsigned long)0x00007C00)        /*!< SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  ADC_SQR1_SQ16                       ((unsigned long)0x000F8000)        /*!< SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((unsigned long)0x00008000)        /*!< Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((unsigned long)0x00010000)        /*!< Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((unsigned long)0x00020000)        /*!< Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((unsigned long)0x00040000)        /*!< Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((unsigned long)0x00080000)        /*!< Bit 4 */

#define  ADC_SQR1_L                          ((unsigned long)0x00F00000)        /*!< L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  ADC_SQR1_L_1                        ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  ADC_SQR1_L_2                        ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  ADC_SQR1_L_3                        ((unsigned long)0x00800000)        /*!< Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((unsigned long)0x0000001F)        /*!< SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((unsigned long)0x00000010)        /*!< Bit 4 */

#define  ADC_SQR2_SQ8                        ((unsigned long)0x000003E0)        /*!< SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((unsigned long)0x00000020)        /*!< Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((unsigned long)0x00000040)        /*!< Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((unsigned long)0x00000080)        /*!< Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((unsigned long)0x00000100)        /*!< Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((unsigned long)0x00000200)        /*!< Bit 4 */

#define  ADC_SQR2_SQ9                        ((unsigned long)0x00007C00)        /*!< SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  ADC_SQR2_SQ10                       ((unsigned long)0x000F8000)        /*!< SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((unsigned long)0x00008000)        /*!< Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((unsigned long)0x00010000)        /*!< Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((unsigned long)0x00020000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((unsigned long)0x00040000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((unsigned long)0x00080000)        /*!< Bit 4 */

#define  ADC_SQR2_SQ11                       ((unsigned long)0x01F00000)        /*!< SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((unsigned long)0x00800000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((unsigned long)0x01000000)        /*!< Bit 4 */

#define  ADC_SQR2_SQ12                       ((unsigned long)0x3E000000)        /*!< SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((unsigned long)0x02000000)        /*!< Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((unsigned long)0x04000000)        /*!< Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((unsigned long)0x08000000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((unsigned long)0x10000000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((unsigned long)0x20000000)        /*!< Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((unsigned long)0x0000001F)        /*!< SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((unsigned long)0x00000010)        /*!< Bit 4 */

#define  ADC_SQR3_SQ2                        ((unsigned long)0x000003E0)        /*!< SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((unsigned long)0x00000020)        /*!< Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((unsigned long)0x00000040)        /*!< Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((unsigned long)0x00000080)        /*!< Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((unsigned long)0x00000100)        /*!< Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((unsigned long)0x00000200)        /*!< Bit 4 */

#define  ADC_SQR3_SQ3                        ((unsigned long)0x00007C00)        /*!< SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  ADC_SQR3_SQ4                        ((unsigned long)0x000F8000)        /*!< SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((unsigned long)0x00008000)        /*!< Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((unsigned long)0x00010000)        /*!< Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((unsigned long)0x00020000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((unsigned long)0x00040000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((unsigned long)0x00080000)        /*!< Bit 4 */

#define  ADC_SQR3_SQ5                        ((unsigned long)0x01F00000)        /*!< SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((unsigned long)0x00800000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((unsigned long)0x01000000)        /*!< Bit 4 */

#define  ADC_SQR3_SQ6                        ((unsigned long)0x3E000000)        /*!< SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((unsigned long)0x02000000)        /*!< Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((unsigned long)0x04000000)        /*!< Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((unsigned long)0x08000000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((unsigned long)0x10000000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((unsigned long)0x20000000)        /*!< Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((unsigned long)0x0000001F)        /*!< JSQ1[4:0] bits (1st conversion in injected sequence) */
#define  ADC_JSQR_JSQ1_0                     ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((unsigned long)0x00000010)        /*!< Bit 4 */

#define  ADC_JSQR_JSQ2                       ((unsigned long)0x000003E0)        /*!< JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((unsigned long)0x00000020)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((unsigned long)0x00000040)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((unsigned long)0x00000080)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((unsigned long)0x00000100)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((unsigned long)0x00000200)        /*!< Bit 4 */

#define  ADC_JSQR_JSQ3                       ((unsigned long)0x00007C00)        /*!< JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  ADC_JSQR_JSQ4                       ((unsigned long)0x000F8000)        /*!< JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((unsigned long)0x00008000)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((unsigned long)0x00010000)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((unsigned long)0x00020000)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((unsigned long)0x00040000)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((unsigned long)0x00080000)        /*!< Bit 4 */

#define  ADC_JSQR_JL                         ((unsigned long)0x00300000)        /*!< JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  ADC_JSQR_JL_1                       ((unsigned long)0x00200000)        /*!< Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((unsigned short)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((unsigned short)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((unsigned short)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((unsigned short)0xFFFF)            /*!< Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((unsigned long)0x0000FFFF)        /*!< Regular data */
#define  ADC_DR_ADC2DATA                     ((unsigned long)0xFFFF0000)        /*!< ADC2 data */

/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((unsigned long)0x00000001)        /*!< DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((unsigned long)0x00000002)        /*!< DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((unsigned long)0x00000004)        /*!< DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((unsigned long)0x00000038)        /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((unsigned long)0x00000008)        /*!< Bit 0 */
#define  DAC_CR_TSEL1_1                      ((unsigned long)0x00000010)        /*!< Bit 1 */
#define  DAC_CR_TSEL1_2                      ((unsigned long)0x00000020)        /*!< Bit 2 */

#define  DAC_CR_WAVE1                        ((unsigned long)0x000000C0)        /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((unsigned long)0x00000040)        /*!< Bit 0 */
#define  DAC_CR_WAVE1_1                      ((unsigned long)0x00000080)        /*!< Bit 1 */

#define  DAC_CR_MAMP1                        ((unsigned long)0x00000F00)        /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  DAC_CR_MAMP1_1                      ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  DAC_CR_MAMP1_2                      ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  DAC_CR_MAMP1_3                      ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  DAC_CR_DMAEN1                       ((unsigned long)0x00001000)        /*!< DAC channel1 DMA enable */
#define  DAC_CR_EN2                          ((unsigned long)0x00010000)        /*!< DAC channel2 enable */
#define  DAC_CR_BOFF2                        ((unsigned long)0x00020000)        /*!< DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         ((unsigned long)0x00040000)        /*!< DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((unsigned long)0x00380000)        /*!< TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((unsigned long)0x00080000)        /*!< Bit 0 */
#define  DAC_CR_TSEL2_1                      ((unsigned long)0x00100000)        /*!< Bit 1 */
#define  DAC_CR_TSEL2_2                      ((unsigned long)0x00200000)        /*!< Bit 2 */

#define  DAC_CR_WAVE2                        ((unsigned long)0x00C00000)        /*!< WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((unsigned long)0x00400000)        /*!< Bit 0 */
#define  DAC_CR_WAVE2_1                      ((unsigned long)0x00800000)        /*!< Bit 1 */

#define  DAC_CR_MAMP2                        ((unsigned long)0x0F000000)        /*!< MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  DAC_CR_MAMP2_1                      ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  DAC_CR_MAMP2_2                      ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  DAC_CR_MAMP2_3                      ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  DAC_CR_DMAEN2                       ((unsigned long)0x10000000)        /*!< DAC channel2 DMA enabled */

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((unsigned char)0x01)               /*!< DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((unsigned char)0x02)               /*!< DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((unsigned short)0x0FFF)            /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((unsigned short)0xFFF0)            /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((unsigned char)0xFF)               /*!< DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((unsigned short)0x0FFF)            /*!< DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((unsigned short)0xFFF0)            /*!< DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((unsigned char)0xFF)               /*!< DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((unsigned long)0x00000FFF)        /*!< DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((unsigned long)0x0FFF0000)        /*!< DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((unsigned long)0x0000FFF0)        /*!< DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((unsigned long)0xFFF00000)        /*!< DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((unsigned short)0x00FF)            /*!< DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((unsigned short)0xFF00)            /*!< DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((unsigned short)0x0FFF)            /*!< DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((unsigned short)0x0FFF)            /*!< DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((unsigned long)0x00002000)        /*!< DAC channel1 DMA underrun flag */
#define  DAC_SR_DMAUDR2                      ((unsigned long)0x20000000)        /*!< DAC channel2 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                    CEC                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for CEC_CFGR register  ******************/
#define  CEC_CFGR_PE              ((unsigned short)0x0001)     /*!<  Peripheral Enable */
#define  CEC_CFGR_IE              ((unsigned short)0x0002)     /*!<  Interrupt Enable */
#define  CEC_CFGR_BTEM            ((unsigned short)0x0004)     /*!<  Bit Timing Error Mode */
#define  CEC_CFGR_BPEM            ((unsigned short)0x0008)     /*!<  Bit Period Error Mode */

/********************  Bit definition for CEC_OAR register  ******************/
#define  CEC_OAR_OA               ((unsigned short)0x000F)     /*!<  OA[3:0]: Own Address */
#define  CEC_OAR_OA_0             ((unsigned short)0x0001)     /*!<  Bit 0 */
#define  CEC_OAR_OA_1             ((unsigned short)0x0002)     /*!<  Bit 1 */
#define  CEC_OAR_OA_2             ((unsigned short)0x0004)     /*!<  Bit 2 */
#define  CEC_OAR_OA_3             ((unsigned short)0x0008)     /*!<  Bit 3 */

/********************  Bit definition for CEC_PRES register  ******************/
#define  CEC_PRES_PRES            ((unsigned short)0x3FFF)   /*!<  Prescaler Counter Value */

/********************  Bit definition for CEC_ESR register  ******************/
#define  CEC_ESR_BTE              ((unsigned short)0x0001)     /*!<  Bit Timing Error */
#define  CEC_ESR_BPE              ((unsigned short)0x0002)     /*!<  Bit Period Error */
#define  CEC_ESR_RBTFE            ((unsigned short)0x0004)     /*!<  Rx Block Transfer Finished Error */
#define  CEC_ESR_SBE              ((unsigned short)0x0008)     /*!<  Start Bit Error */
#define  CEC_ESR_ACKE             ((unsigned short)0x0010)     /*!<  Block Acknowledge Error */
#define  CEC_ESR_LINE             ((unsigned short)0x0020)     /*!<  Line Error */
#define  CEC_ESR_TBTFE            ((unsigned short)0x0040)     /*!<  Tx Block Transfer Finished Error */

/********************  Bit definition for CEC_CSR register  ******************/
#define  CEC_CSR_TSOM             ((unsigned short)0x0001)     /*!<  Tx Start Of Message */
#define  CEC_CSR_TEOM             ((unsigned short)0x0002)     /*!<  Tx End Of Message */
#define  CEC_CSR_TERR             ((unsigned short)0x0004)     /*!<  Tx Error */
#define  CEC_CSR_TBTRF            ((unsigned short)0x0008)     /*!<  Tx Byte Transfer Request or Block Transfer Finished */
#define  CEC_CSR_RSOM             ((unsigned short)0x0010)     /*!<  Rx Start Of Message */
#define  CEC_CSR_REOM             ((unsigned short)0x0020)     /*!<  Rx End Of Message */
#define  CEC_CSR_RERR             ((unsigned short)0x0040)     /*!<  Rx Error */
#define  CEC_CSR_RBTF             ((unsigned short)0x0080)     /*!<  Rx Block Transfer Finished */

/********************  Bit definition for CEC_TXD register  ******************/
#define  CEC_TXD_TXD              ((unsigned short)0x00FF)     /*!<  Tx Data register */

/********************  Bit definition for CEC_RXD register  ******************/
#define  CEC_RXD_RXD              ((unsigned short)0x00FF)     /*!<  Rx Data register */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((unsigned short)0x0001)            /*!< Counter enable */
#define  TIM_CR1_UDIS                        ((unsigned short)0x0002)            /*!< Update disable */
#define  TIM_CR1_URS                         ((unsigned short)0x0004)            /*!< Update request source */
#define  TIM_CR1_OPM                         ((unsigned short)0x0008)            /*!< One pulse mode */
#define  TIM_CR1_DIR                         ((unsigned short)0x0010)            /*!< Direction */

#define  TIM_CR1_CMS                         ((unsigned short)0x0060)            /*!< CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((unsigned short)0x0020)            /*!< Bit 0 */
#define  TIM_CR1_CMS_1                       ((unsigned short)0x0040)            /*!< Bit 1 */

#define  TIM_CR1_ARPE                        ((unsigned short)0x0080)            /*!< Auto-reload preload enable */

#define  TIM_CR1_CKD                         ((unsigned short)0x0300)            /*!< CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((unsigned short)0x0100)            /*!< Bit 0 */
#define  TIM_CR1_CKD_1                       ((unsigned short)0x0200)            /*!< Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((unsigned short)0x0001)            /*!< Capture/Compare Preloaded Control */
#define  TIM_CR2_CCUS                        ((unsigned short)0x0004)            /*!< Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((unsigned short)0x0008)            /*!< Capture/Compare DMA Selection */

#define  TIM_CR2_MMS                         ((unsigned short)0x0070)            /*!< MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((unsigned short)0x0010)            /*!< Bit 0 */
#define  TIM_CR2_MMS_1                       ((unsigned short)0x0020)            /*!< Bit 1 */
#define  TIM_CR2_MMS_2                       ((unsigned short)0x0040)            /*!< Bit 2 */

#define  TIM_CR2_TI1S                        ((unsigned short)0x0080)            /*!< TI1 Selection */
#define  TIM_CR2_OIS1                        ((unsigned short)0x0100)            /*!< Output Idle state 1 (OC1 output) */
#define  TIM_CR2_OIS1N                       ((unsigned short)0x0200)            /*!< Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((unsigned short)0x0400)            /*!< Output Idle state 2 (OC2 output) */
#define  TIM_CR2_OIS2N                       ((unsigned short)0x0800)            /*!< Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((unsigned short)0x1000)            /*!< Output Idle state 3 (OC3 output) */
#define  TIM_CR2_OIS3N                       ((unsigned short)0x2000)            /*!< Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((unsigned short)0x4000)            /*!< Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((unsigned short)0x0007)            /*!< SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMCR_SMS_0                      ((unsigned short)0x0001)            /*!< Bit 0 */
#define  TIM_SMCR_SMS_1                      ((unsigned short)0x0002)            /*!< Bit 1 */
#define  TIM_SMCR_SMS_2                      ((unsigned short)0x0004)            /*!< Bit 2 */

#define  TIM_SMCR_TS                         ((unsigned short)0x0070)            /*!< TS[2:0] bits (Trigger selection) */
#define  TIM_SMCR_TS_0                       ((unsigned short)0x0010)            /*!< Bit 0 */
#define  TIM_SMCR_TS_1                       ((unsigned short)0x0020)            /*!< Bit 1 */
#define  TIM_SMCR_TS_2                       ((unsigned short)0x0040)            /*!< Bit 2 */

#define  TIM_SMCR_MSM                        ((unsigned short)0x0080)            /*!< Master/slave mode */

#define  TIM_SMCR_ETF                        ((unsigned short)0x0F00)            /*!< ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((unsigned short)0x0100)            /*!< Bit 0 */
#define  TIM_SMCR_ETF_1                      ((unsigned short)0x0200)            /*!< Bit 1 */
#define  TIM_SMCR_ETF_2                      ((unsigned short)0x0400)            /*!< Bit 2 */
#define  TIM_SMCR_ETF_3                      ((unsigned short)0x0800)            /*!< Bit 3 */

#define  TIM_SMCR_ETPS                       ((unsigned short)0x3000)            /*!< ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((unsigned short)0x1000)            /*!< Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((unsigned short)0x2000)            /*!< Bit 1 */

#define  TIM_SMCR_ECE                        ((unsigned short)0x4000)            /*!< External clock enable */
#define  TIM_SMCR_ETP                        ((unsigned short)0x8000)            /*!< External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((unsigned short)0x0001)            /*!< Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((unsigned short)0x0002)            /*!< Capture/Compare 1 interrupt enable */
#define  TIM_DIER_CC2IE                      ((unsigned short)0x0004)            /*!< Capture/Compare 2 interrupt enable */
#define  TIM_DIER_CC3IE                      ((unsigned short)0x0008)            /*!< Capture/Compare 3 interrupt enable */
#define  TIM_DIER_CC4IE                      ((unsigned short)0x0010)            /*!< Capture/Compare 4 interrupt enable */
#define  TIM_DIER_COMIE                      ((unsigned short)0x0020)            /*!< COM interrupt enable */
#define  TIM_DIER_TIE                        ((unsigned short)0x0040)            /*!< Trigger interrupt enable */
#define  TIM_DIER_BIE                        ((unsigned short)0x0080)            /*!< Break interrupt enable */
#define  TIM_DIER_UDE                        ((unsigned short)0x0100)            /*!< Update DMA request enable */
#define  TIM_DIER_CC1DE                      ((unsigned short)0x0200)            /*!< Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((unsigned short)0x0400)            /*!< Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((unsigned short)0x0800)            /*!< Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((unsigned short)0x1000)            /*!< Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((unsigned short)0x2000)            /*!< COM DMA request enable */
#define  TIM_DIER_TDE                        ((unsigned short)0x4000)            /*!< Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((unsigned short)0x0001)            /*!< Update interrupt Flag */
#define  TIM_SR_CC1IF                        ((unsigned short)0x0002)            /*!< Capture/Compare 1 interrupt Flag */
#define  TIM_SR_CC2IF                        ((unsigned short)0x0004)            /*!< Capture/Compare 2 interrupt Flag */
#define  TIM_SR_CC3IF                        ((unsigned short)0x0008)            /*!< Capture/Compare 3 interrupt Flag */
#define  TIM_SR_CC4IF                        ((unsigned short)0x0010)            /*!< Capture/Compare 4 interrupt Flag */
#define  TIM_SR_COMIF                        ((unsigned short)0x0020)            /*!< COM interrupt Flag */
#define  TIM_SR_TIF                          ((unsigned short)0x0040)            /*!< Trigger interrupt Flag */
#define  TIM_SR_BIF                          ((unsigned short)0x0080)            /*!< Break interrupt Flag */
#define  TIM_SR_CC1OF                        ((unsigned short)0x0200)            /*!< Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((unsigned short)0x0400)            /*!< Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((unsigned short)0x0800)            /*!< Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((unsigned short)0x1000)            /*!< Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((unsigned char)0x01)               /*!< Update Generation */
#define  TIM_EGR_CC1G                        ((unsigned char)0x02)               /*!< Capture/Compare 1 Generation */
#define  TIM_EGR_CC2G                        ((unsigned char)0x04)               /*!< Capture/Compare 2 Generation */
#define  TIM_EGR_CC3G                        ((unsigned char)0x08)               /*!< Capture/Compare 3 Generation */
#define  TIM_EGR_CC4G                        ((unsigned char)0x10)               /*!< Capture/Compare 4 Generation */
#define  TIM_EGR_COMG                        ((unsigned char)0x20)               /*!< Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((unsigned char)0x40)               /*!< Trigger Generation */
#define  TIM_EGR_BG                          ((unsigned char)0x80)               /*!< Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((unsigned short)0x0003)            /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((unsigned short)0x0001)            /*!< Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((unsigned short)0x0002)            /*!< Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((unsigned short)0x0004)            /*!< Output Compare 1 Fast enable */
#define  TIM_CCMR1_OC1PE                     ((unsigned short)0x0008)            /*!< Output Compare 1 Preload enable */

#define  TIM_CCMR1_OC1M                      ((unsigned short)0x0070)            /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_CCMR1_OC1M_0                    ((unsigned short)0x0010)            /*!< Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((unsigned short)0x0020)            /*!< Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((unsigned short)0x0040)            /*!< Bit 2 */

#define  TIM_CCMR1_OC1CE                     ((unsigned short)0x0080)            /*!< Output Compare 1Clear Enable */

#define  TIM_CCMR1_CC2S                      ((unsigned short)0x0300)            /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((unsigned short)0x0400)            /*!< Output Compare 2 Fast enable */
#define  TIM_CCMR1_OC2PE                     ((unsigned short)0x0800)            /*!< Output Compare 2 Preload enable */

#define  TIM_CCMR1_OC2M                      ((unsigned short)0x7000)            /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_CCMR1_OC2M_0                    ((unsigned short)0x1000)            /*!< Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((unsigned short)0x2000)            /*!< Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((unsigned short)0x4000)            /*!< Bit 2 */

#define  TIM_CCMR1_OC2CE                     ((unsigned short)0x8000)            /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((unsigned short)0x000C)            /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((unsigned short)0x0004)            /*!< Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((unsigned short)0x0008)            /*!< Bit 1 */

#define  TIM_CCMR1_IC1F                      ((unsigned short)0x00F0)            /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_CCMR1_IC1F_0                    ((unsigned short)0x0010)            /*!< Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((unsigned short)0x0020)            /*!< Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((unsigned short)0x0040)            /*!< Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((unsigned short)0x0080)            /*!< Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((unsigned short)0x0C00)            /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_CCMR1_IC2PSC_0                  ((unsigned short)0x0400)            /*!< Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((unsigned short)0x0800)            /*!< Bit 1 */

#define  TIM_CCMR1_IC2F                      ((unsigned short)0xF000)            /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_CCMR1_IC2F_0                    ((unsigned short)0x1000)            /*!< Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((unsigned short)0x2000)            /*!< Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((unsigned short)0x4000)            /*!< Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((unsigned short)0x8000)            /*!< Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((unsigned short)0x0003)            /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CCMR2_CC3S_0                    ((unsigned short)0x0001)            /*!< Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((unsigned short)0x0002)            /*!< Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((unsigned short)0x0004)            /*!< Output Compare 3 Fast enable */
#define  TIM_CCMR2_OC3PE                     ((unsigned short)0x0008)            /*!< Output Compare 3 Preload enable */

#define  TIM_CCMR2_OC3M                      ((unsigned short)0x0070)            /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((unsigned short)0x0010)            /*!< Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((unsigned short)0x0020)            /*!< Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((unsigned short)0x0040)            /*!< Bit 2 */

#define  TIM_CCMR2_OC3CE                     ((unsigned short)0x0080)            /*!< Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((unsigned short)0x0300)            /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((unsigned short)0x0100)            /*!< Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((unsigned short)0x0200)            /*!< Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((unsigned short)0x0400)            /*!< Output Compare 4 Fast enable */
#define  TIM_CCMR2_OC4PE                     ((unsigned short)0x0800)            /*!< Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((unsigned short)0x7000)            /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((unsigned short)0x1000)            /*!< Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((unsigned short)0x2000)            /*!< Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((unsigned short)0x4000)            /*!< Bit 2 */

#define  TIM_CCMR2_OC4CE                     ((unsigned short)0x8000)            /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((unsigned short)0x000C)            /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((unsigned short)0x0004)            /*!< Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((unsigned short)0x0008)            /*!< Bit 1 */

#define  TIM_CCMR2_IC3F                      ((unsigned short)0x00F0)            /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((unsigned short)0x0010)            /*!< Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((unsigned short)0x0020)            /*!< Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((unsigned short)0x0040)            /*!< Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((unsigned short)0x0080)            /*!< Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((unsigned short)0x0C00)            /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((unsigned short)0x0400)            /*!< Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((unsigned short)0x0800)            /*!< Bit 1 */

#define  TIM_CCMR2_IC4F                      ((unsigned short)0xF000)            /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((unsigned short)0x1000)            /*!< Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((unsigned short)0x2000)            /*!< Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((unsigned short)0x4000)            /*!< Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((unsigned short)0x8000)            /*!< Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((unsigned short)0x0001)            /*!< Capture/Compare 1 output enable */
#define  TIM_CCER_CC1P                       ((unsigned short)0x0002)            /*!< Capture/Compare 1 output Polarity */
#define  TIM_CCER_CC1NE                      ((unsigned short)0x0004)            /*!< Capture/Compare 1 Complementary output enable */
#define  TIM_CCER_CC1NP                      ((unsigned short)0x0008)            /*!< Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((unsigned short)0x0010)            /*!< Capture/Compare 2 output enable */
#define  TIM_CCER_CC2P                       ((unsigned short)0x0020)            /*!< Capture/Compare 2 output Polarity */
#define  TIM_CCER_CC2NE                      ((unsigned short)0x0040)            /*!< Capture/Compare 2 Complementary output enable */
#define  TIM_CCER_CC2NP                      ((unsigned short)0x0080)            /*!< Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((unsigned short)0x0100)            /*!< Capture/Compare 3 output enable */
#define  TIM_CCER_CC3P                       ((unsigned short)0x0200)            /*!< Capture/Compare 3 output Polarity */
#define  TIM_CCER_CC3NE                      ((unsigned short)0x0400)            /*!< Capture/Compare 3 Complementary output enable */
#define  TIM_CCER_CC3NP                      ((unsigned short)0x0800)            /*!< Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((unsigned short)0x1000)            /*!< Capture/Compare 4 output enable */
#define  TIM_CCER_CC4P                       ((unsigned short)0x2000)            /*!< Capture/Compare 4 output Polarity */
#define  TIM_CCER_CC4NP                      ((unsigned short)0x8000)            /*!< Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((unsigned short)0xFFFF)            /*!< Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((unsigned short)0xFFFF)            /*!< Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((unsigned short)0xFFFF)            /*!< actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((unsigned char)0xFF)               /*!< Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((unsigned short)0xFFFF)            /*!< Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((unsigned short)0xFFFF)            /*!< Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((unsigned short)0xFFFF)            /*!< Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((unsigned short)0xFFFF)            /*!< Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((unsigned short)0x00FF)            /*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((unsigned short)0x0001)            /*!< Bit 0 */
#define  TIM_BDTR_DTG_1                      ((unsigned short)0x0002)            /*!< Bit 1 */
#define  TIM_BDTR_DTG_2                      ((unsigned short)0x0004)            /*!< Bit 2 */
#define  TIM_BDTR_DTG_3                      ((unsigned short)0x0008)            /*!< Bit 3 */
#define  TIM_BDTR_DTG_4                      ((unsigned short)0x0010)            /*!< Bit 4 */
#define  TIM_BDTR_DTG_5                      ((unsigned short)0x0020)            /*!< Bit 5 */
#define  TIM_BDTR_DTG_6                      ((unsigned short)0x0040)            /*!< Bit 6 */
#define  TIM_BDTR_DTG_7                      ((unsigned short)0x0080)            /*!< Bit 7 */

#define  TIM_BDTR_LOCK                       ((unsigned short)0x0300)            /*!< LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((unsigned short)0x0100)            /*!< Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((unsigned short)0x0200)            /*!< Bit 1 */

#define  TIM_BDTR_OSSI                       ((unsigned short)0x0400)            /*!< Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((unsigned short)0x0800)            /*!< Off-State Selection for Run mode */
#define  TIM_BDTR_BKE                        ((unsigned short)0x1000)            /*!< Break enable */
#define  TIM_BDTR_BKP                        ((unsigned short)0x2000)            /*!< Break Polarity */
#define  TIM_BDTR_AOE                        ((unsigned short)0x4000)            /*!< Automatic Output enable */
#define  TIM_BDTR_MOE                        ((unsigned short)0x8000)            /*!< Main Output enable */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((unsigned short)0x001F)            /*!< DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((unsigned short)0x0001)            /*!< Bit 0 */
#define  TIM_DCR_DBA_1                       ((unsigned short)0x0002)            /*!< Bit 1 */
#define  TIM_DCR_DBA_2                       ((unsigned short)0x0004)            /*!< Bit 2 */
#define  TIM_DCR_DBA_3                       ((unsigned short)0x0008)            /*!< Bit 3 */
#define  TIM_DCR_DBA_4                       ((unsigned short)0x0010)            /*!< Bit 4 */

#define  TIM_DCR_DBL                         ((unsigned short)0x1F00)            /*!< DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((unsigned short)0x0100)            /*!< Bit 0 */
#define  TIM_DCR_DBL_1                       ((unsigned short)0x0200)            /*!< Bit 1 */
#define  TIM_DCR_DBL_2                       ((unsigned short)0x0400)            /*!< Bit 2 */
#define  TIM_DCR_DBL_3                       ((unsigned short)0x0800)            /*!< Bit 3 */
#define  TIM_DCR_DBL_4                       ((unsigned short)0x1000)            /*!< Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((unsigned short)0xFFFF)            /*!< DMA register for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                             Real-Time Clock                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for RTC_CRH register  ********************/
#define  RTC_CRH_SECIE                       ((unsigned char)0x01)               /*!< Second Interrupt Enable */
#define  RTC_CRH_ALRIE                       ((unsigned char)0x02)               /*!< Alarm Interrupt Enable */
#define  RTC_CRH_OWIE                        ((unsigned char)0x04)               /*!< OverfloW Interrupt Enable */

/*******************  Bit definition for RTC_CRL register  ********************/
#define  RTC_CRL_SECF                        ((unsigned char)0x01)               /*!< Second Flag */
#define  RTC_CRL_ALRF                        ((unsigned char)0x02)               /*!< Alarm Flag */
#define  RTC_CRL_OWF                         ((unsigned char)0x04)               /*!< OverfloW Flag */
#define  RTC_CRL_RSF                         ((unsigned char)0x08)               /*!< Registers Synchronized Flag */
#define  RTC_CRL_CNF                         ((unsigned char)0x10)               /*!< Configuration Flag */
#define  RTC_CRL_RTOFF                       ((unsigned char)0x20)               /*!< RTC operation OFF */

/*******************  Bit definition for RTC_PRLH register  *******************/
#define  RTC_PRLH_PRL                        ((unsigned short)0x000F)            /*!< RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_PRLL register  *******************/
#define  RTC_PRLL_PRL                        ((unsigned short)0xFFFF)            /*!< RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define  RTC_DIVH_RTC_DIV                    ((unsigned short)0x000F)            /*!< RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define  RTC_DIVL_RTC_DIV                    ((unsigned short)0xFFFF)            /*!< RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define  RTC_CNTH_RTC_CNT                    ((unsigned short)0xFFFF)            /*!< RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define  RTC_CNTL_RTC_CNT                    ((unsigned short)0xFFFF)            /*!< RTC Counter Low */

/*******************  Bit definition for RTC_ALRH register  *******************/
#define  RTC_ALRH_RTC_ALR                    ((unsigned short)0xFFFF)            /*!< RTC Alarm High */

/*******************  Bit definition for RTC_ALRL register  *******************/
#define  RTC_ALRL_RTC_ALR                    ((unsigned short)0xFFFF)            /*!< RTC Alarm Low */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((unsigned short)0xFFFF)            /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((unsigned char)0x07)               /*!< PR[2:0] (Prescaler divider) */
#define  IWDG_PR_PR_0                        ((unsigned char)0x01)               /*!< Bit 0 */
#define  IWDG_PR_PR_1                        ((unsigned char)0x02)               /*!< Bit 1 */
#define  IWDG_PR_PR_2                        ((unsigned char)0x04)               /*!< Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((unsigned short)0x0FFF)            /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((unsigned char)0x01)               /*!< Watchdog prescaler value update */
#define  IWDG_SR_RVU                         ((unsigned char)0x02)               /*!< Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((unsigned char)0x7F)               /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((unsigned char)0x01)               /*!< Bit 0 */
#define  WWDG_CR_T1                          ((unsigned char)0x02)               /*!< Bit 1 */
#define  WWDG_CR_T2                          ((unsigned char)0x04)               /*!< Bit 2 */
#define  WWDG_CR_T3                          ((unsigned char)0x08)               /*!< Bit 3 */
#define  WWDG_CR_T4                          ((unsigned char)0x10)               /*!< Bit 4 */
#define  WWDG_CR_T5                          ((unsigned char)0x20)               /*!< Bit 5 */
#define  WWDG_CR_T6                          ((unsigned char)0x40)               /*!< Bit 6 */

#define  WWDG_CR_WDGA                        ((unsigned char)0x80)               /*!< Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((unsigned short)0x007F)            /*!< W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((unsigned short)0x0001)            /*!< Bit 0 */
#define  WWDG_CFR_W1                         ((unsigned short)0x0002)            /*!< Bit 1 */
#define  WWDG_CFR_W2                         ((unsigned short)0x0004)            /*!< Bit 2 */
#define  WWDG_CFR_W3                         ((unsigned short)0x0008)            /*!< Bit 3 */
#define  WWDG_CFR_W4                         ((unsigned short)0x0010)            /*!< Bit 4 */
#define  WWDG_CFR_W5                         ((unsigned short)0x0020)            /*!< Bit 5 */
#define  WWDG_CFR_W6                         ((unsigned short)0x0040)            /*!< Bit 6 */

#define  WWDG_CFR_WDGTB                      ((unsigned short)0x0180)            /*!< WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((unsigned short)0x0080)            /*!< Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((unsigned short)0x0100)            /*!< Bit 1 */

#define  WWDG_CFR_EWI                        ((unsigned short)0x0200)            /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((unsigned char)0x01)               /*!< Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                       Flexible Static Memory Controller                    */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for FSMC_BCR1 register  *******************/
#define  FSMC_BCR1_MBKEN                     ((unsigned long)0x00000001)        /*!< Memory bank enable bit */
#define  FSMC_BCR1_MUXEN                     ((unsigned long)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  FSMC_BCR1_MTYP                      ((unsigned long)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR1_MTYP_0                    ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  FSMC_BCR1_MTYP_1                    ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  FSMC_BCR1_MWID                      ((unsigned long)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR1_MWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BCR1_MWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_BCR1_FACCEN                    ((unsigned long)0x00000040)        /*!< Flash access enable */
#define  FSMC_BCR1_BURSTEN                   ((unsigned long)0x00000100)        /*!< Burst enable bit */
#define  FSMC_BCR1_WAITPOL                   ((unsigned long)0x00000200)        /*!< Wait signal polarity bit */
#define  FSMC_BCR1_WRAPMOD                   ((unsigned long)0x00000400)        /*!< Wrapped burst mode support */
#define  FSMC_BCR1_WAITCFG                   ((unsigned long)0x00000800)        /*!< Wait timing configuration */
#define  FSMC_BCR1_WREN                      ((unsigned long)0x00001000)        /*!< Write enable bit */
#define  FSMC_BCR1_WAITEN                    ((unsigned long)0x00002000)        /*!< Wait enable bit */
#define  FSMC_BCR1_EXTMOD                    ((unsigned long)0x00004000)        /*!< Extended mode enable */
#define  FSMC_BCR1_ASYNCWAIT                 ((unsigned long)0x00008000)       /*!< Asynchronous wait */
#define  FSMC_BCR1_CBURSTRW                  ((unsigned long)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for FSMC_BCR2 register  *******************/
#define  FSMC_BCR2_MBKEN                     ((unsigned long)0x00000001)        /*!< Memory bank enable bit */
#define  FSMC_BCR2_MUXEN                     ((unsigned long)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  FSMC_BCR2_MTYP                      ((unsigned long)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR2_MTYP_0                    ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  FSMC_BCR2_MTYP_1                    ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  FSMC_BCR2_MWID                      ((unsigned long)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR2_MWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BCR2_MWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_BCR2_FACCEN                    ((unsigned long)0x00000040)        /*!< Flash access enable */
#define  FSMC_BCR2_BURSTEN                   ((unsigned long)0x00000100)        /*!< Burst enable bit */
#define  FSMC_BCR2_WAITPOL                   ((unsigned long)0x00000200)        /*!< Wait signal polarity bit */
#define  FSMC_BCR2_WRAPMOD                   ((unsigned long)0x00000400)        /*!< Wrapped burst mode support */
#define  FSMC_BCR2_WAITCFG                   ((unsigned long)0x00000800)        /*!< Wait timing configuration */
#define  FSMC_BCR2_WREN                      ((unsigned long)0x00001000)        /*!< Write enable bit */
#define  FSMC_BCR2_WAITEN                    ((unsigned long)0x00002000)        /*!< Wait enable bit */
#define  FSMC_BCR2_EXTMOD                    ((unsigned long)0x00004000)        /*!< Extended mode enable */
#define  FSMC_BCR2_ASYNCWAIT                 ((unsigned long)0x00008000)       /*!< Asynchronous wait */
#define  FSMC_BCR2_CBURSTRW                  ((unsigned long)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for FSMC_BCR3 register  *******************/
#define  FSMC_BCR3_MBKEN                     ((unsigned long)0x00000001)        /*!< Memory bank enable bit */
#define  FSMC_BCR3_MUXEN                     ((unsigned long)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  FSMC_BCR3_MTYP                      ((unsigned long)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR3_MTYP_0                    ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  FSMC_BCR3_MTYP_1                    ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  FSMC_BCR3_MWID                      ((unsigned long)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR3_MWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BCR3_MWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_BCR3_FACCEN                    ((unsigned long)0x00000040)        /*!< Flash access enable */
#define  FSMC_BCR3_BURSTEN                   ((unsigned long)0x00000100)        /*!< Burst enable bit */
#define  FSMC_BCR3_WAITPOL                   ((unsigned long)0x00000200)        /*!< Wait signal polarity bit. */
#define  FSMC_BCR3_WRAPMOD                   ((unsigned long)0x00000400)        /*!< Wrapped burst mode support */
#define  FSMC_BCR3_WAITCFG                   ((unsigned long)0x00000800)        /*!< Wait timing configuration */
#define  FSMC_BCR3_WREN                      ((unsigned long)0x00001000)        /*!< Write enable bit */
#define  FSMC_BCR3_WAITEN                    ((unsigned long)0x00002000)        /*!< Wait enable bit */
#define  FSMC_BCR3_EXTMOD                    ((unsigned long)0x00004000)        /*!< Extended mode enable */
#define  FSMC_BCR3_ASYNCWAIT                 ((unsigned long)0x00008000)       /*!< Asynchronous wait */
#define  FSMC_BCR3_CBURSTRW                  ((unsigned long)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for FSMC_BCR4 register  *******************/
#define  FSMC_BCR4_MBKEN                     ((unsigned long)0x00000001)        /*!< Memory bank enable bit */
#define  FSMC_BCR4_MUXEN                     ((unsigned long)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  FSMC_BCR4_MTYP                      ((unsigned long)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR4_MTYP_0                    ((unsigned long)0x00000004)        /*!< Bit 0 */
#define  FSMC_BCR4_MTYP_1                    ((unsigned long)0x00000008)        /*!< Bit 1 */

#define  FSMC_BCR4_MWID                      ((unsigned long)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR4_MWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BCR4_MWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_BCR4_FACCEN                    ((unsigned long)0x00000040)        /*!< Flash access enable */
#define  FSMC_BCR4_BURSTEN                   ((unsigned long)0x00000100)        /*!< Burst enable bit */
#define  FSMC_BCR4_WAITPOL                   ((unsigned long)0x00000200)        /*!< Wait signal polarity bit */
#define  FSMC_BCR4_WRAPMOD                   ((unsigned long)0x00000400)        /*!< Wrapped burst mode support */
#define  FSMC_BCR4_WAITCFG                   ((unsigned long)0x00000800)        /*!< Wait timing configuration */
#define  FSMC_BCR4_WREN                      ((unsigned long)0x00001000)        /*!< Write enable bit */
#define  FSMC_BCR4_WAITEN                    ((unsigned long)0x00002000)        /*!< Wait enable bit */
#define  FSMC_BCR4_EXTMOD                    ((unsigned long)0x00004000)        /*!< Extended mode enable */
#define  FSMC_BCR4_ASYNCWAIT                 ((unsigned long)0x00008000)       /*!< Asynchronous wait */
#define  FSMC_BCR4_CBURSTRW                  ((unsigned long)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for FSMC_BTR1 register  ******************/
#define  FSMC_BTR1_ADDSET                    ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR1_ADDSET_0                  ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BTR1_ADDSET_1                  ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BTR1_ADDSET_2                  ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BTR1_ADDSET_3                  ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BTR1_ADDHLD                    ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR1_ADDHLD_0                  ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BTR1_ADDHLD_1                  ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BTR1_ADDHLD_2                  ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BTR1_ADDHLD_3                  ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BTR1_DATAST                    ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR1_DATAST_0                  ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BTR1_DATAST_1                  ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BTR1_DATAST_2                  ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BTR1_DATAST_3                  ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BTR1_BUSTURN                   ((unsigned long)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR1_BUSTURN_0                 ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_BTR1_BUSTURN_1                 ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_BTR1_BUSTURN_2                 ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_BTR1_BUSTURN_3                 ((unsigned long)0x00080000)        /*!< Bit 3 */

#define  FSMC_BTR1_CLKDIV                    ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR1_CLKDIV_0                  ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BTR1_CLKDIV_1                  ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BTR1_CLKDIV_2                  ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BTR1_CLKDIV_3                  ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BTR1_DATLAT                    ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR1_DATLAT_0                  ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BTR1_DATLAT_1                  ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BTR1_DATLAT_2                  ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BTR1_DATLAT_3                  ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BTR1_ACCMOD                    ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR1_ACCMOD_0                  ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BTR1_ACCMOD_1                  ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_BTR2 register  *******************/
#define  FSMC_BTR2_ADDSET                    ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR2_ADDSET_0                  ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BTR2_ADDSET_1                  ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BTR2_ADDSET_2                  ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BTR2_ADDSET_3                  ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BTR2_ADDHLD                    ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR2_ADDHLD_0                  ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BTR2_ADDHLD_1                  ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BTR2_ADDHLD_2                  ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BTR2_ADDHLD_3                  ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BTR2_DATAST                    ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR2_DATAST_0                  ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BTR2_DATAST_1                  ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BTR2_DATAST_2                  ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BTR2_DATAST_3                  ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BTR2_BUSTURN                   ((unsigned long)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR2_BUSTURN_0                 ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_BTR2_BUSTURN_1                 ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_BTR2_BUSTURN_2                 ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_BTR2_BUSTURN_3                 ((unsigned long)0x00080000)        /*!< Bit 3 */

#define  FSMC_BTR2_CLKDIV                    ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR2_CLKDIV_0                  ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BTR2_CLKDIV_1                  ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BTR2_CLKDIV_2                  ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BTR2_CLKDIV_3                  ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BTR2_DATLAT                    ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR2_DATLAT_0                  ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BTR2_DATLAT_1                  ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BTR2_DATLAT_2                  ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BTR2_DATLAT_3                  ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BTR2_ACCMOD                    ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR2_ACCMOD_0                  ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BTR2_ACCMOD_1                  ((unsigned long)0x20000000)        /*!< Bit 1 */

/*******************  Bit definition for FSMC_BTR3 register  *******************/
#define  FSMC_BTR3_ADDSET                    ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR3_ADDSET_0                  ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BTR3_ADDSET_1                  ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BTR3_ADDSET_2                  ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BTR3_ADDSET_3                  ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BTR3_ADDHLD                    ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR3_ADDHLD_0                  ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BTR3_ADDHLD_1                  ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BTR3_ADDHLD_2                  ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BTR3_ADDHLD_3                  ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BTR3_DATAST                    ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR3_DATAST_0                  ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BTR3_DATAST_1                  ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BTR3_DATAST_2                  ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BTR3_DATAST_3                  ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BTR3_BUSTURN                   ((unsigned long)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR3_BUSTURN_0                 ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_BTR3_BUSTURN_1                 ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_BTR3_BUSTURN_2                 ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_BTR3_BUSTURN_3                 ((unsigned long)0x00080000)        /*!< Bit 3 */

#define  FSMC_BTR3_CLKDIV                    ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR3_CLKDIV_0                  ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BTR3_CLKDIV_1                  ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BTR3_CLKDIV_2                  ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BTR3_CLKDIV_3                  ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BTR3_DATLAT                    ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR3_DATLAT_0                  ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BTR3_DATLAT_1                  ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BTR3_DATLAT_2                  ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BTR3_DATLAT_3                  ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BTR3_ACCMOD                    ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR3_ACCMOD_0                  ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BTR3_ACCMOD_1                  ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_BTR4 register  *******************/
#define  FSMC_BTR4_ADDSET                    ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR4_ADDSET_0                  ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BTR4_ADDSET_1                  ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BTR4_ADDSET_2                  ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BTR4_ADDSET_3                  ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BTR4_ADDHLD                    ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR4_ADDHLD_0                  ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BTR4_ADDHLD_1                  ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BTR4_ADDHLD_2                  ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BTR4_ADDHLD_3                  ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BTR4_DATAST                    ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR4_DATAST_0                  ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BTR4_DATAST_1                  ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BTR4_DATAST_2                  ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BTR4_DATAST_3                  ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BTR4_BUSTURN                   ((unsigned long)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR4_BUSTURN_0                 ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_BTR4_BUSTURN_1                 ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_BTR4_BUSTURN_2                 ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_BTR4_BUSTURN_3                 ((unsigned long)0x00080000)        /*!< Bit 3 */

#define  FSMC_BTR4_CLKDIV                    ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR4_CLKDIV_0                  ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BTR4_CLKDIV_1                  ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BTR4_CLKDIV_2                  ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BTR4_CLKDIV_3                  ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BTR4_DATLAT                    ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR4_DATLAT_0                  ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BTR4_DATLAT_1                  ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BTR4_DATLAT_2                  ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BTR4_DATLAT_3                  ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BTR4_ACCMOD                    ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR4_ACCMOD_0                  ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BTR4_ACCMOD_1                  ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_BWTR1 register  ******************/
#define  FSMC_BWTR1_ADDSET                   ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR1_ADDSET_0                 ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BWTR1_ADDSET_1                 ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BWTR1_ADDSET_2                 ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BWTR1_ADDSET_3                 ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BWTR1_ADDHLD                   ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR1_ADDHLD_0                 ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BWTR1_ADDHLD_1                 ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BWTR1_ADDHLD_2                 ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BWTR1_ADDHLD_3                 ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BWTR1_DATAST                   ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR1_DATAST_0                 ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BWTR1_DATAST_1                 ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BWTR1_DATAST_2                 ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BWTR1_DATAST_3                 ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BWTR1_CLKDIV                   ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR1_CLKDIV_0                 ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BWTR1_CLKDIV_1                 ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BWTR1_CLKDIV_2                 ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BWTR1_CLKDIV_3                 ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BWTR1_DATLAT                   ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR1_DATLAT_0                 ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BWTR1_DATLAT_1                 ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BWTR1_DATLAT_2                 ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BWTR1_DATLAT_3                 ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BWTR1_ACCMOD                   ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR1_ACCMOD_0                 ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BWTR1_ACCMOD_1                 ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_BWTR2 register  ******************/
#define  FSMC_BWTR2_ADDSET                   ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR2_ADDSET_0                 ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BWTR2_ADDSET_1                 ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BWTR2_ADDSET_2                 ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BWTR2_ADDSET_3                 ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BWTR2_ADDHLD                   ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR2_ADDHLD_0                 ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BWTR2_ADDHLD_1                 ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BWTR2_ADDHLD_2                 ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BWTR2_ADDHLD_3                 ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BWTR2_DATAST                   ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR2_DATAST_0                 ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BWTR2_DATAST_1                 ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BWTR2_DATAST_2                 ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BWTR2_DATAST_3                 ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BWTR2_CLKDIV                   ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR2_CLKDIV_0                 ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BWTR2_CLKDIV_1                 ((unsigned long)0x00200000)        /*!< Bit 1*/
#define  FSMC_BWTR2_CLKDIV_2                 ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BWTR2_CLKDIV_3                 ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BWTR2_DATLAT                   ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR2_DATLAT_0                 ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BWTR2_DATLAT_1                 ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BWTR2_DATLAT_2                 ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BWTR2_DATLAT_3                 ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BWTR2_ACCMOD                   ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR2_ACCMOD_0                 ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BWTR2_ACCMOD_1                 ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_BWTR3 register  ******************/
#define  FSMC_BWTR3_ADDSET                   ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR3_ADDSET_0                 ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BWTR3_ADDSET_1                 ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BWTR3_ADDSET_2                 ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BWTR3_ADDSET_3                 ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BWTR3_ADDHLD                   ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR3_ADDHLD_0                 ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BWTR3_ADDHLD_1                 ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BWTR3_ADDHLD_2                 ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BWTR3_ADDHLD_3                 ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BWTR3_DATAST                   ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR3_DATAST_0                 ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BWTR3_DATAST_1                 ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BWTR3_DATAST_2                 ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BWTR3_DATAST_3                 ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BWTR3_CLKDIV                   ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR3_CLKDIV_0                 ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BWTR3_CLKDIV_1                 ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BWTR3_CLKDIV_2                 ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BWTR3_CLKDIV_3                 ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BWTR3_DATLAT                   ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR3_DATLAT_0                 ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BWTR3_DATLAT_1                 ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BWTR3_DATLAT_2                 ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BWTR3_DATLAT_3                 ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BWTR3_ACCMOD                   ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR3_ACCMOD_0                 ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BWTR3_ACCMOD_1                 ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_BWTR4 register  ******************/
#define  FSMC_BWTR4_ADDSET                   ((unsigned long)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR4_ADDSET_0                 ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_BWTR4_ADDSET_1                 ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_BWTR4_ADDSET_2                 ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_BWTR4_ADDSET_3                 ((unsigned long)0x00000008)        /*!< Bit 3 */

#define  FSMC_BWTR4_ADDHLD                   ((unsigned long)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR4_ADDHLD_0                 ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_BWTR4_ADDHLD_1                 ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  FSMC_BWTR4_ADDHLD_2                 ((unsigned long)0x00000040)        /*!< Bit 2 */
#define  FSMC_BWTR4_ADDHLD_3                 ((unsigned long)0x00000080)        /*!< Bit 3 */

#define  FSMC_BWTR4_DATAST                   ((unsigned long)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR4_DATAST_0                 ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_BWTR4_DATAST_1                 ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_BWTR4_DATAST_2                 ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_BWTR4_DATAST_3                 ((unsigned long)0x00000800)        /*!< Bit 3 */

#define  FSMC_BWTR4_CLKDIV                   ((unsigned long)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR4_CLKDIV_0                 ((unsigned long)0x00100000)        /*!< Bit 0 */
#define  FSMC_BWTR4_CLKDIV_1                 ((unsigned long)0x00200000)        /*!< Bit 1 */
#define  FSMC_BWTR4_CLKDIV_2                 ((unsigned long)0x00400000)        /*!< Bit 2 */
#define  FSMC_BWTR4_CLKDIV_3                 ((unsigned long)0x00800000)        /*!< Bit 3 */

#define  FSMC_BWTR4_DATLAT                   ((unsigned long)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR4_DATLAT_0                 ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_BWTR4_DATLAT_1                 ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_BWTR4_DATLAT_2                 ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_BWTR4_DATLAT_3                 ((unsigned long)0x08000000)        /*!< Bit 3 */

#define  FSMC_BWTR4_ACCMOD                   ((unsigned long)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR4_ACCMOD_0                 ((unsigned long)0x10000000)        /*!< Bit 0 */
#define  FSMC_BWTR4_ACCMOD_1                 ((unsigned long)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for FSMC_PCR2 register  *******************/
#define  FSMC_PCR2_PWAITEN                   ((unsigned long)0x00000002)        /*!< Wait feature enable bit */
#define  FSMC_PCR2_PBKEN                     ((unsigned long)0x00000004)        /*!< PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR2_PTYP                      ((unsigned long)0x00000008)        /*!< Memory type */

#define  FSMC_PCR2_PWID                      ((unsigned long)0x00000030)        /*!< PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR2_PWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_PCR2_PWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_PCR2_ECCEN                     ((unsigned long)0x00000040)        /*!< ECC computation logic enable bit */

#define  FSMC_PCR2_TCLR                      ((unsigned long)0x00001E00)        /*!< TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR2_TCLR_0                    ((unsigned long)0x00000200)        /*!< Bit 0 */
#define  FSMC_PCR2_TCLR_1                    ((unsigned long)0x00000400)        /*!< Bit 1 */
#define  FSMC_PCR2_TCLR_2                    ((unsigned long)0x00000800)        /*!< Bit 2 */
#define  FSMC_PCR2_TCLR_3                    ((unsigned long)0x00001000)        /*!< Bit 3 */

#define  FSMC_PCR2_TAR                       ((unsigned long)0x0001E000)        /*!< TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR2_TAR_0                     ((unsigned long)0x00002000)        /*!< Bit 0 */
#define  FSMC_PCR2_TAR_1                     ((unsigned long)0x00004000)        /*!< Bit 1 */
#define  FSMC_PCR2_TAR_2                     ((unsigned long)0x00008000)        /*!< Bit 2 */
#define  FSMC_PCR2_TAR_3                     ((unsigned long)0x00010000)        /*!< Bit 3 */

#define  FSMC_PCR2_ECCPS                     ((unsigned long)0x000E0000)        /*!< ECCPS[1:0] bits (ECC page size) */
#define  FSMC_PCR2_ECCPS_0                   ((unsigned long)0x00020000)        /*!< Bit 0 */
#define  FSMC_PCR2_ECCPS_1                   ((unsigned long)0x00040000)        /*!< Bit 1 */
#define  FSMC_PCR2_ECCPS_2                   ((unsigned long)0x00080000)        /*!< Bit 2 */

/******************  Bit definition for FSMC_PCR3 register  *******************/
#define  FSMC_PCR3_PWAITEN                   ((unsigned long)0x00000002)        /*!< Wait feature enable bit */
#define  FSMC_PCR3_PBKEN                     ((unsigned long)0x00000004)        /*!< PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR3_PTYP                      ((unsigned long)0x00000008)        /*!< Memory type */

#define  FSMC_PCR3_PWID                      ((unsigned long)0x00000030)        /*!< PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR3_PWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_PCR3_PWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_PCR3_ECCEN                     ((unsigned long)0x00000040)        /*!< ECC computation logic enable bit */

#define  FSMC_PCR3_TCLR                      ((unsigned long)0x00001E00)        /*!< TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR3_TCLR_0                    ((unsigned long)0x00000200)        /*!< Bit 0 */
#define  FSMC_PCR3_TCLR_1                    ((unsigned long)0x00000400)        /*!< Bit 1 */
#define  FSMC_PCR3_TCLR_2                    ((unsigned long)0x00000800)        /*!< Bit 2 */
#define  FSMC_PCR3_TCLR_3                    ((unsigned long)0x00001000)        /*!< Bit 3 */

#define  FSMC_PCR3_TAR                       ((unsigned long)0x0001E000)        /*!< TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR3_TAR_0                     ((unsigned long)0x00002000)        /*!< Bit 0 */
#define  FSMC_PCR3_TAR_1                     ((unsigned long)0x00004000)        /*!< Bit 1 */
#define  FSMC_PCR3_TAR_2                     ((unsigned long)0x00008000)        /*!< Bit 2 */
#define  FSMC_PCR3_TAR_3                     ((unsigned long)0x00010000)        /*!< Bit 3 */

#define  FSMC_PCR3_ECCPS                     ((unsigned long)0x000E0000)        /*!< ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR3_ECCPS_0                   ((unsigned long)0x00020000)        /*!< Bit 0 */
#define  FSMC_PCR3_ECCPS_1                   ((unsigned long)0x00040000)        /*!< Bit 1 */
#define  FSMC_PCR3_ECCPS_2                   ((unsigned long)0x00080000)        /*!< Bit 2 */

/******************  Bit definition for FSMC_PCR4 register  *******************/
#define  FSMC_PCR4_PWAITEN                   ((unsigned long)0x00000002)        /*!< Wait feature enable bit */
#define  FSMC_PCR4_PBKEN                     ((unsigned long)0x00000004)        /*!< PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR4_PTYP                      ((unsigned long)0x00000008)        /*!< Memory type */

#define  FSMC_PCR4_PWID                      ((unsigned long)0x00000030)        /*!< PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR4_PWID_0                    ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  FSMC_PCR4_PWID_1                    ((unsigned long)0x00000020)        /*!< Bit 1 */

#define  FSMC_PCR4_ECCEN                     ((unsigned long)0x00000040)        /*!< ECC computation logic enable bit */

#define  FSMC_PCR4_TCLR                      ((unsigned long)0x00001E00)        /*!< TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR4_TCLR_0                    ((unsigned long)0x00000200)        /*!< Bit 0 */
#define  FSMC_PCR4_TCLR_1                    ((unsigned long)0x00000400)        /*!< Bit 1 */
#define  FSMC_PCR4_TCLR_2                    ((unsigned long)0x00000800)        /*!< Bit 2 */
#define  FSMC_PCR4_TCLR_3                    ((unsigned long)0x00001000)        /*!< Bit 3 */

#define  FSMC_PCR4_TAR                       ((unsigned long)0x0001E000)        /*!< TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR4_TAR_0                     ((unsigned long)0x00002000)        /*!< Bit 0 */
#define  FSMC_PCR4_TAR_1                     ((unsigned long)0x00004000)        /*!< Bit 1 */
#define  FSMC_PCR4_TAR_2                     ((unsigned long)0x00008000)        /*!< Bit 2 */
#define  FSMC_PCR4_TAR_3                     ((unsigned long)0x00010000)        /*!< Bit 3 */

#define  FSMC_PCR4_ECCPS                     ((unsigned long)0x000E0000)        /*!< ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR4_ECCPS_0                   ((unsigned long)0x00020000)        /*!< Bit 0 */
#define  FSMC_PCR4_ECCPS_1                   ((unsigned long)0x00040000)        /*!< Bit 1 */
#define  FSMC_PCR4_ECCPS_2                   ((unsigned long)0x00080000)        /*!< Bit 2 */

/*******************  Bit definition for FSMC_SR2 register  *******************/
#define  FSMC_SR2_IRS                        ((unsigned char)0x01)               /*!< Interrupt Rising Edge status */
#define  FSMC_SR2_ILS                        ((unsigned char)0x02)               /*!< Interrupt Level status */
#define  FSMC_SR2_IFS                        ((unsigned char)0x04)               /*!< Interrupt Falling Edge status */
#define  FSMC_SR2_IREN                       ((unsigned char)0x08)               /*!< Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR2_ILEN                       ((unsigned char)0x10)               /*!< Interrupt Level detection Enable bit */
#define  FSMC_SR2_IFEN                       ((unsigned char)0x20)               /*!< Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR2_FEMPT                      ((unsigned char)0x40)               /*!< FIFO empty */

/*******************  Bit definition for FSMC_SR3 register  *******************/
#define  FSMC_SR3_IRS                        ((unsigned char)0x01)               /*!< Interrupt Rising Edge status */
#define  FSMC_SR3_ILS                        ((unsigned char)0x02)               /*!< Interrupt Level status */
#define  FSMC_SR3_IFS                        ((unsigned char)0x04)               /*!< Interrupt Falling Edge status */
#define  FSMC_SR3_IREN                       ((unsigned char)0x08)               /*!< Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR3_ILEN                       ((unsigned char)0x10)               /*!< Interrupt Level detection Enable bit */
#define  FSMC_SR3_IFEN                       ((unsigned char)0x20)               /*!< Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR3_FEMPT                      ((unsigned char)0x40)               /*!< FIFO empty */

/*******************  Bit definition for FSMC_SR4 register  *******************/
#define  FSMC_SR4_IRS                        ((unsigned char)0x01)               /*!< Interrupt Rising Edge status */
#define  FSMC_SR4_ILS                        ((unsigned char)0x02)               /*!< Interrupt Level status */
#define  FSMC_SR4_IFS                        ((unsigned char)0x04)               /*!< Interrupt Falling Edge status */
#define  FSMC_SR4_IREN                       ((unsigned char)0x08)               /*!< Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR4_ILEN                       ((unsigned char)0x10)               /*!< Interrupt Level detection Enable bit */
#define  FSMC_SR4_IFEN                       ((unsigned char)0x20)               /*!< Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR4_FEMPT                      ((unsigned char)0x40)               /*!< FIFO empty */

/******************  Bit definition for FSMC_PMEM2 register  ******************/
#define  FSMC_PMEM2_MEMSET2                  ((unsigned long)0x000000FF)        /*!< MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FSMC_PMEM2_MEMSET2_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PMEM2_MEMSET2_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PMEM2_MEMSET2_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PMEM2_MEMSET2_3                ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PMEM2_MEMSET2_4                ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PMEM2_MEMSET2_5                ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PMEM2_MEMSET2_6                ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PMEM2_MEMSET2_7                ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PMEM2_MEMWAIT2                 ((unsigned long)0x0000FF00)        /*!< MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FSMC_PMEM2_MEMWAIT2_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PMEM2_MEMWAIT2_1               ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PMEM2_MEMWAIT2_2               ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PMEM2_MEMWAIT2_3               ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PMEM2_MEMWAIT2_4               ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PMEM2_MEMWAIT2_5               ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PMEM2_MEMWAIT2_6               ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PMEM2_MEMWAIT2_7               ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PMEM2_MEMHOLD2                 ((unsigned long)0x00FF0000)        /*!< MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FSMC_PMEM2_MEMHOLD2_0               ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PMEM2_MEMHOLD2_1               ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PMEM2_MEMHOLD2_2               ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PMEM2_MEMHOLD2_3               ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PMEM2_MEMHOLD2_4               ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PMEM2_MEMHOLD2_5               ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PMEM2_MEMHOLD2_6               ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PMEM2_MEMHOLD2_7               ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PMEM2_MEMHIZ2                  ((unsigned long)0xFF000000)        /*!< MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FSMC_PMEM2_MEMHIZ2_0                ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PMEM2_MEMHIZ2_1                ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PMEM2_MEMHIZ2_2                ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PMEM2_MEMHIZ2_3                ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PMEM2_MEMHIZ2_4                ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PMEM2_MEMHIZ2_5                ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PMEM2_MEMHIZ2_6                ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PMEM2_MEMHIZ2_7                ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_PMEM3 register  ******************/
#define  FSMC_PMEM3_MEMSET3                  ((unsigned long)0x000000FF)        /*!< MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FSMC_PMEM3_MEMSET3_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PMEM3_MEMSET3_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PMEM3_MEMSET3_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PMEM3_MEMSET3_3                ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PMEM3_MEMSET3_4                ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PMEM3_MEMSET3_5                ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PMEM3_MEMSET3_6                ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PMEM3_MEMSET3_7                ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PMEM3_MEMWAIT3                 ((unsigned long)0x0000FF00)        /*!< MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FSMC_PMEM3_MEMWAIT3_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PMEM3_MEMWAIT3_1               ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PMEM3_MEMWAIT3_2               ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PMEM3_MEMWAIT3_3               ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PMEM3_MEMWAIT3_4               ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PMEM3_MEMWAIT3_5               ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PMEM3_MEMWAIT3_6               ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PMEM3_MEMWAIT3_7               ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PMEM3_MEMHOLD3                 ((unsigned long)0x00FF0000)        /*!< MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FSMC_PMEM3_MEMHOLD3_0               ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PMEM3_MEMHOLD3_1               ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PMEM3_MEMHOLD3_2               ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PMEM3_MEMHOLD3_3               ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PMEM3_MEMHOLD3_4               ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PMEM3_MEMHOLD3_5               ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PMEM3_MEMHOLD3_6               ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PMEM3_MEMHOLD3_7               ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PMEM3_MEMHIZ3                  ((unsigned long)0xFF000000)        /*!< MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FSMC_PMEM3_MEMHIZ3_0                ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PMEM3_MEMHIZ3_1                ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PMEM3_MEMHIZ3_2                ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PMEM3_MEMHIZ3_3                ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PMEM3_MEMHIZ3_4                ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PMEM3_MEMHIZ3_5                ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PMEM3_MEMHIZ3_6                ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PMEM3_MEMHIZ3_7                ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_PMEM4 register  ******************/
#define  FSMC_PMEM4_MEMSET4                  ((unsigned long)0x000000FF)        /*!< MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FSMC_PMEM4_MEMSET4_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PMEM4_MEMSET4_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PMEM4_MEMSET4_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PMEM4_MEMSET4_3                ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PMEM4_MEMSET4_4                ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PMEM4_MEMSET4_5                ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PMEM4_MEMSET4_6                ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PMEM4_MEMSET4_7                ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PMEM4_MEMWAIT4                 ((unsigned long)0x0000FF00)        /*!< MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FSMC_PMEM4_MEMWAIT4_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PMEM4_MEMWAIT4_1               ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PMEM4_MEMWAIT4_2               ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PMEM4_MEMWAIT4_3               ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PMEM4_MEMWAIT4_4               ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PMEM4_MEMWAIT4_5               ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PMEM4_MEMWAIT4_6               ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PMEM4_MEMWAIT4_7               ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PMEM4_MEMHOLD4                 ((unsigned long)0x00FF0000)        /*!< MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FSMC_PMEM4_MEMHOLD4_0               ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PMEM4_MEMHOLD4_1               ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PMEM4_MEMHOLD4_2               ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PMEM4_MEMHOLD4_3               ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PMEM4_MEMHOLD4_4               ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PMEM4_MEMHOLD4_5               ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PMEM4_MEMHOLD4_6               ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PMEM4_MEMHOLD4_7               ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PMEM4_MEMHIZ4                  ((unsigned long)0xFF000000)        /*!< MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FSMC_PMEM4_MEMHIZ4_0                ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PMEM4_MEMHIZ4_1                ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PMEM4_MEMHIZ4_2                ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PMEM4_MEMHIZ4_3                ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PMEM4_MEMHIZ4_4                ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PMEM4_MEMHIZ4_5                ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PMEM4_MEMHIZ4_6                ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PMEM4_MEMHIZ4_7                ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_PATT2 register  ******************/
#define  FSMC_PATT2_ATTSET2                  ((unsigned long)0x000000FF)        /*!< ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FSMC_PATT2_ATTSET2_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PATT2_ATTSET2_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PATT2_ATTSET2_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PATT2_ATTSET2_3                ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PATT2_ATTSET2_4                ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PATT2_ATTSET2_5                ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PATT2_ATTSET2_6                ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PATT2_ATTSET2_7                ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PATT2_ATTWAIT2                 ((unsigned long)0x0000FF00)        /*!< ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FSMC_PATT2_ATTWAIT2_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PATT2_ATTWAIT2_1               ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PATT2_ATTWAIT2_2               ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PATT2_ATTWAIT2_3               ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PATT2_ATTWAIT2_4               ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PATT2_ATTWAIT2_5               ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PATT2_ATTWAIT2_6               ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PATT2_ATTWAIT2_7               ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PATT2_ATTHOLD2                 ((unsigned long)0x00FF0000)        /*!< ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FSMC_PATT2_ATTHOLD2_0               ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PATT2_ATTHOLD2_1               ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PATT2_ATTHOLD2_2               ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PATT2_ATTHOLD2_3               ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PATT2_ATTHOLD2_4               ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PATT2_ATTHOLD2_5               ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PATT2_ATTHOLD2_6               ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PATT2_ATTHOLD2_7               ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PATT2_ATTHIZ2                  ((unsigned long)0xFF000000)        /*!< ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FSMC_PATT2_ATTHIZ2_0                ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PATT2_ATTHIZ2_1                ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PATT2_ATTHIZ2_2                ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PATT2_ATTHIZ2_3                ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PATT2_ATTHIZ2_4                ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PATT2_ATTHIZ2_5                ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PATT2_ATTHIZ2_6                ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PATT2_ATTHIZ2_7                ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_PATT3 register  ******************/
#define  FSMC_PATT3_ATTSET3                  ((unsigned long)0x000000FF)        /*!< ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FSMC_PATT3_ATTSET3_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PATT3_ATTSET3_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PATT3_ATTSET3_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PATT3_ATTSET3_3                ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PATT3_ATTSET3_4                ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PATT3_ATTSET3_5                ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PATT3_ATTSET3_6                ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PATT3_ATTSET3_7                ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PATT3_ATTWAIT3                 ((unsigned long)0x0000FF00)        /*!< ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FSMC_PATT3_ATTWAIT3_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PATT3_ATTWAIT3_1               ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PATT3_ATTWAIT3_2               ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PATT3_ATTWAIT3_3               ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PATT3_ATTWAIT3_4               ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PATT3_ATTWAIT3_5               ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PATT3_ATTWAIT3_6               ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PATT3_ATTWAIT3_7               ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PATT3_ATTHOLD3                 ((unsigned long)0x00FF0000)        /*!< ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FSMC_PATT3_ATTHOLD3_0               ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PATT3_ATTHOLD3_1               ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PATT3_ATTHOLD3_2               ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PATT3_ATTHOLD3_3               ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PATT3_ATTHOLD3_4               ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PATT3_ATTHOLD3_5               ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PATT3_ATTHOLD3_6               ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PATT3_ATTHOLD3_7               ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PATT3_ATTHIZ3                  ((unsigned long)0xFF000000)        /*!< ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FSMC_PATT3_ATTHIZ3_0                ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PATT3_ATTHIZ3_1                ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PATT3_ATTHIZ3_2                ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PATT3_ATTHIZ3_3                ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PATT3_ATTHIZ3_4                ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PATT3_ATTHIZ3_5                ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PATT3_ATTHIZ3_6                ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PATT3_ATTHIZ3_7                ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_PATT4 register  ******************/
#define  FSMC_PATT4_ATTSET4                  ((unsigned long)0x000000FF)        /*!< ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FSMC_PATT4_ATTSET4_0                ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PATT4_ATTSET4_1                ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PATT4_ATTSET4_2                ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PATT4_ATTSET4_3                ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PATT4_ATTSET4_4                ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PATT4_ATTSET4_5                ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PATT4_ATTSET4_6                ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PATT4_ATTSET4_7                ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PATT4_ATTWAIT4                 ((unsigned long)0x0000FF00)        /*!< ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FSMC_PATT4_ATTWAIT4_0               ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PATT4_ATTWAIT4_1               ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PATT4_ATTWAIT4_2               ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PATT4_ATTWAIT4_3               ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PATT4_ATTWAIT4_4               ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PATT4_ATTWAIT4_5               ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PATT4_ATTWAIT4_6               ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PATT4_ATTWAIT4_7               ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PATT4_ATTHOLD4                 ((unsigned long)0x00FF0000)        /*!< ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FSMC_PATT4_ATTHOLD4_0               ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PATT4_ATTHOLD4_1               ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PATT4_ATTHOLD4_2               ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PATT4_ATTHOLD4_3               ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PATT4_ATTHOLD4_4               ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PATT4_ATTHOLD4_5               ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PATT4_ATTHOLD4_6               ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PATT4_ATTHOLD4_7               ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PATT4_ATTHIZ4                  ((unsigned long)0xFF000000)        /*!< ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FSMC_PATT4_ATTHIZ4_0                ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PATT4_ATTHIZ4_1                ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PATT4_ATTHIZ4_2                ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PATT4_ATTHIZ4_3                ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PATT4_ATTHIZ4_4                ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PATT4_ATTHIZ4_5                ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PATT4_ATTHIZ4_6                ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PATT4_ATTHIZ4_7                ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_PIO4 register  *******************/
#define  FSMC_PIO4_IOSET4                    ((unsigned long)0x000000FF)        /*!< IOSET4[7:0] bits (I/O 4 setup time) */
#define  FSMC_PIO4_IOSET4_0                  ((unsigned long)0x00000001)        /*!< Bit 0 */
#define  FSMC_PIO4_IOSET4_1                  ((unsigned long)0x00000002)        /*!< Bit 1 */
#define  FSMC_PIO4_IOSET4_2                  ((unsigned long)0x00000004)        /*!< Bit 2 */
#define  FSMC_PIO4_IOSET4_3                  ((unsigned long)0x00000008)        /*!< Bit 3 */
#define  FSMC_PIO4_IOSET4_4                  ((unsigned long)0x00000010)        /*!< Bit 4 */
#define  FSMC_PIO4_IOSET4_5                  ((unsigned long)0x00000020)        /*!< Bit 5 */
#define  FSMC_PIO4_IOSET4_6                  ((unsigned long)0x00000040)        /*!< Bit 6 */
#define  FSMC_PIO4_IOSET4_7                  ((unsigned long)0x00000080)        /*!< Bit 7 */

#define  FSMC_PIO4_IOWAIT4                   ((unsigned long)0x0000FF00)        /*!< IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FSMC_PIO4_IOWAIT4_0                 ((unsigned long)0x00000100)        /*!< Bit 0 */
#define  FSMC_PIO4_IOWAIT4_1                 ((unsigned long)0x00000200)        /*!< Bit 1 */
#define  FSMC_PIO4_IOWAIT4_2                 ((unsigned long)0x00000400)        /*!< Bit 2 */
#define  FSMC_PIO4_IOWAIT4_3                 ((unsigned long)0x00000800)        /*!< Bit 3 */
#define  FSMC_PIO4_IOWAIT4_4                 ((unsigned long)0x00001000)        /*!< Bit 4 */
#define  FSMC_PIO4_IOWAIT4_5                 ((unsigned long)0x00002000)        /*!< Bit 5 */
#define  FSMC_PIO4_IOWAIT4_6                 ((unsigned long)0x00004000)        /*!< Bit 6 */
#define  FSMC_PIO4_IOWAIT4_7                 ((unsigned long)0x00008000)        /*!< Bit 7 */

#define  FSMC_PIO4_IOHOLD4                   ((unsigned long)0x00FF0000)        /*!< IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FSMC_PIO4_IOHOLD4_0                 ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  FSMC_PIO4_IOHOLD4_1                 ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  FSMC_PIO4_IOHOLD4_2                 ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  FSMC_PIO4_IOHOLD4_3                 ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  FSMC_PIO4_IOHOLD4_4                 ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  FSMC_PIO4_IOHOLD4_5                 ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  FSMC_PIO4_IOHOLD4_6                 ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  FSMC_PIO4_IOHOLD4_7                 ((unsigned long)0x00800000)        /*!< Bit 7 */

#define  FSMC_PIO4_IOHIZ4                    ((unsigned long)0xFF000000)        /*!< IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FSMC_PIO4_IOHIZ4_0                  ((unsigned long)0x01000000)        /*!< Bit 0 */
#define  FSMC_PIO4_IOHIZ4_1                  ((unsigned long)0x02000000)        /*!< Bit 1 */
#define  FSMC_PIO4_IOHIZ4_2                  ((unsigned long)0x04000000)        /*!< Bit 2 */
#define  FSMC_PIO4_IOHIZ4_3                  ((unsigned long)0x08000000)        /*!< Bit 3 */
#define  FSMC_PIO4_IOHIZ4_4                  ((unsigned long)0x10000000)        /*!< Bit 4 */
#define  FSMC_PIO4_IOHIZ4_5                  ((unsigned long)0x20000000)        /*!< Bit 5 */
#define  FSMC_PIO4_IOHIZ4_6                  ((unsigned long)0x40000000)        /*!< Bit 6 */
#define  FSMC_PIO4_IOHIZ4_7                  ((unsigned long)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for FSMC_ECCR2 register  ******************/
#define  FSMC_ECCR2_ECC2                     ((unsigned long)0xFFFFFFFF)        /*!< ECC result */

/******************  Bit definition for FSMC_ECCR3 register  ******************/
#define  FSMC_ECCR3_ECC3                     ((unsigned long)0xFFFFFFFF)        /*!< ECC result */

/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  ((unsigned char)0x03)               /*!< PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                ((unsigned char)0x01)               /*!< Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                ((unsigned char)0x02)               /*!< Bit 1 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   ((unsigned short)0x00FF)            /*!< Clock divide factor */
#define  SDIO_CLKCR_CLKEN                    ((unsigned short)0x0100)            /*!< Clock enable bit */
#define  SDIO_CLKCR_PWRSAV                   ((unsigned short)0x0200)            /*!< Power saving configuration bit */
#define  SDIO_CLKCR_BYPASS                   ((unsigned short)0x0400)            /*!< Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   ((unsigned short)0x1800)            /*!< WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 ((unsigned short)0x0800)            /*!< Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 ((unsigned short)0x1000)            /*!< Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  ((unsigned short)0x2000)            /*!< SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  ((unsigned short)0x4000)            /*!< HW Flow Control enable */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     ((unsigned long)0xFFFFFFFF)            /*!< Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   ((unsigned short)0x003F)            /*!< Command Index */

#define  SDIO_CMD_WAITRESP                   ((unsigned short)0x00C0)            /*!< WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 ((unsigned short)0x0040)            /*!<  Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 ((unsigned short)0x0080)            /*!<  Bit 1 */

#define  SDIO_CMD_WAITINT                    ((unsigned short)0x0100)            /*!< CPSM Waits for Interrupt Request */
#define  SDIO_CMD_WAITPEND                   ((unsigned short)0x0200)            /*!< CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     ((unsigned short)0x0400)            /*!< Command path state machine (CPSM) Enable bit */
#define  SDIO_CMD_SDIOSUSPEND                ((unsigned short)0x0800)            /*!< SD I/O suspend command */
#define  SDIO_CMD_ENCMDCOMPL                 ((unsigned short)0x1000)            /*!< Enable CMD completion */
#define  SDIO_CMD_NIEN                       ((unsigned short)0x2000)            /*!< Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   ((unsigned short)0x4000)            /*!< CE-ATA command */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                ((unsigned char)0x3F)               /*!< Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0              ((unsigned long)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RESP1 register  ******************/
#define  SDIO_RESP1_CARDSTATUS1              ((unsigned long)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RESP2 register  ******************/
#define  SDIO_RESP2_CARDSTATUS2              ((unsigned long)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RESP3 register  ******************/
#define  SDIO_RESP3_CARDSTATUS3              ((unsigned long)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RESP4 register  ******************/
#define  SDIO_RESP4_CARDSTATUS4              ((unsigned long)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_DTIMER register  *****************/
#define  SDIO_DTIMER_DATATIME                ((unsigned long)0xFFFFFFFF)        /*!< Data timeout period. */

/******************  Bit definition for SDIO_DLEN register  *******************/
#define  SDIO_DLEN_DATALENGTH                ((unsigned long)0x01FFFFFF)        /*!< Data length value */

/******************  Bit definition for SDIO_DCTRL register  ******************/
#define  SDIO_DCTRL_DTEN                     ((unsigned short)0x0001)            /*!< Data transfer enabled bit */
#define  SDIO_DCTRL_DTDIR                    ((unsigned short)0x0002)            /*!< Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   ((unsigned short)0x0004)            /*!< Data transfer mode selection */
#define  SDIO_DCTRL_DMAEN                    ((unsigned short)0x0008)            /*!< DMA enabled bit */

#define  SDIO_DCTRL_DBLOCKSIZE               ((unsigned short)0x00F0)            /*!< DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             ((unsigned short)0x0010)            /*!< Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             ((unsigned short)0x0020)            /*!< Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             ((unsigned short)0x0040)            /*!< Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             ((unsigned short)0x0080)            /*!< Bit 3 */

#define  SDIO_DCTRL_RWSTART                  ((unsigned short)0x0100)            /*!< Read wait start */
#define  SDIO_DCTRL_RWSTOP                   ((unsigned short)0x0200)            /*!< Read wait stop */
#define  SDIO_DCTRL_RWMOD                    ((unsigned short)0x0400)            /*!< Read wait mode */
#define  SDIO_DCTRL_SDIOEN                   ((unsigned short)0x0800)            /*!< SD I/O enable functions */

/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define  SDIO_DCOUNT_DATACOUNT               ((unsigned long)0x01FFFFFF)        /*!< Data count value */

/******************  Bit definition for SDIO_STA register  ********************/
#define  SDIO_STA_CCRCFAIL                   ((unsigned long)0x00000001)        /*!< Command response received (CRC check failed) */
#define  SDIO_STA_DCRCFAIL                   ((unsigned long)0x00000002)        /*!< Data block sent/received (CRC check failed) */
#define  SDIO_STA_CTIMEOUT                   ((unsigned long)0x00000004)        /*!< Command response timeout */
#define  SDIO_STA_DTIMEOUT                   ((unsigned long)0x00000008)        /*!< Data timeout */
#define  SDIO_STA_TXUNDERR                   ((unsigned long)0x00000010)        /*!< Transmit FIFO underrun error */
#define  SDIO_STA_RXOVERR                    ((unsigned long)0x00000020)        /*!< Received FIFO overrun error */
#define  SDIO_STA_CMDREND                    ((unsigned long)0x00000040)        /*!< Command response received (CRC check passed) */
#define  SDIO_STA_CMDSENT                    ((unsigned long)0x00000080)        /*!< Command sent (no response required) */
#define  SDIO_STA_DATAEND                    ((unsigned long)0x00000100)        /*!< Data end (data counter, SDIDCOUNT, is zero) */
#define  SDIO_STA_STBITERR                   ((unsigned long)0x00000200)        /*!< Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STA_DBCKEND                    ((unsigned long)0x00000400)        /*!< Data block sent/received (CRC check passed) */
#define  SDIO_STA_CMDACT                     ((unsigned long)0x00000800)        /*!< Command transfer in progress */
#define  SDIO_STA_TXACT                      ((unsigned long)0x00001000)        /*!< Data transmit in progress */
#define  SDIO_STA_RXACT                      ((unsigned long)0x00002000)        /*!< Data receive in progress */
#define  SDIO_STA_TXFIFOHE                   ((unsigned long)0x00004000)        /*!< Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STA_RXFIFOHF                   ((unsigned long)0x00008000)        /*!< Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STA_TXFIFOF                    ((unsigned long)0x00010000)        /*!< Transmit FIFO full */
#define  SDIO_STA_RXFIFOF                    ((unsigned long)0x00020000)        /*!< Receive FIFO full */
#define  SDIO_STA_TXFIFOE                    ((unsigned long)0x00040000)        /*!< Transmit FIFO empty */
#define  SDIO_STA_RXFIFOE                    ((unsigned long)0x00080000)        /*!< Receive FIFO empty */
#define  SDIO_STA_TXDAVL                     ((unsigned long)0x00100000)        /*!< Data available in transmit FIFO */
#define  SDIO_STA_RXDAVL                     ((unsigned long)0x00200000)        /*!< Data available in receive FIFO */
#define  SDIO_STA_SDIOIT                     ((unsigned long)0x00400000)        /*!< SDIO interrupt received */
#define  SDIO_STA_CEATAEND                   ((unsigned long)0x00800000)        /*!< CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_ICR register  *******************/
#define  SDIO_ICR_CCRCFAILC                  ((unsigned long)0x00000001)        /*!< CCRCFAIL flag clear bit */
#define  SDIO_ICR_DCRCFAILC                  ((unsigned long)0x00000002)        /*!< DCRCFAIL flag clear bit */
#define  SDIO_ICR_CTIMEOUTC                  ((unsigned long)0x00000004)        /*!< CTIMEOUT flag clear bit */
#define  SDIO_ICR_DTIMEOUTC                  ((unsigned long)0x00000008)        /*!< DTIMEOUT flag clear bit */
#define  SDIO_ICR_TXUNDERRC                  ((unsigned long)0x00000010)        /*!< TXUNDERR flag clear bit */
#define  SDIO_ICR_RXOVERRC                   ((unsigned long)0x00000020)        /*!< RXOVERR flag clear bit */
#define  SDIO_ICR_CMDRENDC                   ((unsigned long)0x00000040)        /*!< CMDREND flag clear bit */
#define  SDIO_ICR_CMDSENTC                   ((unsigned long)0x00000080)        /*!< CMDSENT flag clear bit */
#define  SDIO_ICR_DATAENDC                   ((unsigned long)0x00000100)        /*!< DATAEND flag clear bit */
#define  SDIO_ICR_STBITERRC                  ((unsigned long)0x00000200)        /*!< STBITERR flag clear bit */
#define  SDIO_ICR_DBCKENDC                   ((unsigned long)0x00000400)        /*!< DBCKEND flag clear bit */
#define  SDIO_ICR_SDIOITC                    ((unsigned long)0x00400000)        /*!< SDIOIT flag clear bit */
#define  SDIO_ICR_CEATAENDC                  ((unsigned long)0x00800000)        /*!< CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE                ((unsigned long)0x00000001)        /*!< Command CRC Fail Interrupt Enable */
#define  SDIO_MASK_DCRCFAILIE                ((unsigned long)0x00000002)        /*!< Data CRC Fail Interrupt Enable */
#define  SDIO_MASK_CTIMEOUTIE                ((unsigned long)0x00000004)        /*!< Command TimeOut Interrupt Enable */
#define  SDIO_MASK_DTIMEOUTIE                ((unsigned long)0x00000008)        /*!< Data TimeOut Interrupt Enable */
#define  SDIO_MASK_TXUNDERRIE                ((unsigned long)0x00000010)        /*!< Tx FIFO UnderRun Error Interrupt Enable */
#define  SDIO_MASK_RXOVERRIE                 ((unsigned long)0x00000020)        /*!< Rx FIFO OverRun Error Interrupt Enable */
#define  SDIO_MASK_CMDRENDIE                 ((unsigned long)0x00000040)        /*!< Command Response Received Interrupt Enable */
#define  SDIO_MASK_CMDSENTIE                 ((unsigned long)0x00000080)        /*!< Command Sent Interrupt Enable */
#define  SDIO_MASK_DATAENDIE                 ((unsigned long)0x00000100)        /*!< Data End Interrupt Enable */
#define  SDIO_MASK_STBITERRIE                ((unsigned long)0x00000200)        /*!< Start Bit Error Interrupt Enable */
#define  SDIO_MASK_DBCKENDIE                 ((unsigned long)0x00000400)        /*!< Data Block End Interrupt Enable */
#define  SDIO_MASK_CMDACTIE                  ((unsigned long)0x00000800)        /*!< Command Acting Interrupt Enable */
#define  SDIO_MASK_TXACTIE                   ((unsigned long)0x00001000)        /*!< Data Transmit Acting Interrupt Enable */
#define  SDIO_MASK_RXACTIE                   ((unsigned long)0x00002000)        /*!< Data receive acting interrupt enabled */
#define  SDIO_MASK_TXFIFOHEIE                ((unsigned long)0x00004000)        /*!< Tx FIFO Half Empty interrupt Enable */
#define  SDIO_MASK_RXFIFOHFIE                ((unsigned long)0x00008000)        /*!< Rx FIFO Half Full interrupt Enable */
#define  SDIO_MASK_TXFIFOFIE                 ((unsigned long)0x00010000)        /*!< Tx FIFO Full interrupt Enable */
#define  SDIO_MASK_RXFIFOFIE                 ((unsigned long)0x00020000)        /*!< Rx FIFO Full interrupt Enable */
#define  SDIO_MASK_TXFIFOEIE                 ((unsigned long)0x00040000)        /*!< Tx FIFO Empty interrupt Enable */
#define  SDIO_MASK_RXFIFOEIE                 ((unsigned long)0x00080000)        /*!< Rx FIFO Empty interrupt Enable */
#define  SDIO_MASK_TXDAVLIE                  ((unsigned long)0x00100000)        /*!< Data available in Tx FIFO interrupt Enable */
#define  SDIO_MASK_RXDAVLIE                  ((unsigned long)0x00200000)        /*!< Data available in Rx FIFO interrupt Enable */
#define  SDIO_MASK_SDIOITIE                  ((unsigned long)0x00400000)        /*!< SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_MASK_CEATAENDIE                ((unsigned long)0x00800000)        /*!< CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define  SDIO_FIFOCNT_FIFOCOUNT              ((unsigned long)0x00FFFFFF)        /*!< Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_FIFO register  *******************/
#define  SDIO_FIFO_FIFODATA                  ((unsigned long)0xFFFFFFFF)        /*!< Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                                   USB Device FS                            */
/*                                                                            */
/******************************************************************************/

/*!< Endpoint-specific registers */
/*******************  Bit definition for USB_EP0R register  *******************/
#define  USB_EP0R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP0R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP0R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP0R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP0R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP0R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP0R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP0R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP0R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP0R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP0R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP0R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP0R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP0R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP0R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP0R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP1R register  *******************/
#define  USB_EP1R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP1R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP1R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP1R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP1R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP1R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP1R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP1R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP1R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP1R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP1R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP1R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP1R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP1R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP1R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP1R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP2R register  *******************/
#define  USB_EP2R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP2R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP2R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP2R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP2R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP2R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP2R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP2R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP2R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP2R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP2R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP2R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP2R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP2R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP2R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP2R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP3R register  *******************/
#define  USB_EP3R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP3R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP3R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP3R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP3R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP3R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP3R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP3R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP3R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP3R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP3R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP3R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP3R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP3R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP3R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP3R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP4R register  *******************/
#define  USB_EP4R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP4R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP4R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP4R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP4R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP4R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP4R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP4R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP4R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP4R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP4R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP4R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP4R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP4R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP4R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP4R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP5R register  *******************/
#define  USB_EP5R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP5R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP5R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP5R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP5R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP5R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP5R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP5R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP5R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP5R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP5R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP5R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP5R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP5R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP5R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP5R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP6R register  *******************/
#define  USB_EP6R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP6R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP6R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP6R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP6R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP6R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP6R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP6R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP6R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP6R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP6R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP6R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP6R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP6R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP6R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP6R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP7R register  *******************/
#define  USB_EP7R_EA                         ((unsigned short)0x000F)            /*!< Endpoint Address */

#define  USB_EP7R_STAT_TX                    ((unsigned short)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP7R_STAT_TX_0                  ((unsigned short)0x0010)            /*!< Bit 0 */
#define  USB_EP7R_STAT_TX_1                  ((unsigned short)0x0020)            /*!< Bit 1 */

#define  USB_EP7R_DTOG_TX                    ((unsigned short)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EP7R_CTR_TX                     ((unsigned short)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EP7R_EP_KIND                    ((unsigned short)0x0100)            /*!< Endpoint Kind */

#define  USB_EP7R_EP_TYPE                    ((unsigned short)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP7R_EP_TYPE_0                  ((unsigned short)0x0200)            /*!< Bit 0 */
#define  USB_EP7R_EP_TYPE_1                  ((unsigned short)0x0400)            /*!< Bit 1 */

#define  USB_EP7R_SETUP                      ((unsigned short)0x0800)            /*!< Setup transaction completed */

#define  USB_EP7R_STAT_RX                    ((unsigned short)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP7R_STAT_RX_0                  ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USB_EP7R_STAT_RX_1                  ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USB_EP7R_DTOG_RX                    ((unsigned short)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EP7R_CTR_RX                     ((unsigned short)0x8000)            /*!< Correct Transfer for reception */

/*!< Common registers */
/*******************  Bit definition for USB_CNTR register  *******************/
#define  USB_CNTR_FRES                       ((unsigned short)0x0001)            /*!< Force USB Reset */
#define  USB_CNTR_PDWN                       ((unsigned short)0x0002)            /*!< Power down */
#define  USB_CNTR_LP_MODE                    ((unsigned short)0x0004)            /*!< Low-power mode */
#define  USB_CNTR_FSUSP                      ((unsigned short)0x0008)            /*!< Force suspend */
#define  USB_CNTR_RESUME                     ((unsigned short)0x0010)            /*!< Resume request */
#define  USB_CNTR_ESOFM                      ((unsigned short)0x0100)            /*!< Expected Start Of Frame Interrupt Mask */
#define  USB_CNTR_SOFM                       ((unsigned short)0x0200)            /*!< Start Of Frame Interrupt Mask */
#define  USB_CNTR_RESETM                     ((unsigned short)0x0400)            /*!< RESET Interrupt Mask */
#define  USB_CNTR_SUSPM                      ((unsigned short)0x0800)            /*!< Suspend mode Interrupt Mask */
#define  USB_CNTR_WKUPM                      ((unsigned short)0x1000)            /*!< Wakeup Interrupt Mask */
#define  USB_CNTR_ERRM                       ((unsigned short)0x2000)            /*!< Error Interrupt Mask */
#define  USB_CNTR_PMAOVRM                    ((unsigned short)0x4000)            /*!< Packet Memory Area Over / Underrun Interrupt Mask */
#define  USB_CNTR_CTRM                       ((unsigned short)0x8000)            /*!< Correct Transfer Interrupt Mask */

/*******************  Bit definition for USB_ISTR register  *******************/
#define  USB_ISTR_EP_ID                      ((unsigned short)0x000F)            /*!< Endpoint Identifier */
#define  USB_ISTR_DIR                        ((unsigned short)0x0010)            /*!< Direction of transaction */
#define  USB_ISTR_ESOF                       ((unsigned short)0x0100)            /*!< Expected Start Of Frame */
#define  USB_ISTR_SOF                        ((unsigned short)0x0200)            /*!< Start Of Frame */
#define  USB_ISTR_RESET                      ((unsigned short)0x0400)            /*!< USB RESET request */
#define  USB_ISTR_SUSP                       ((unsigned short)0x0800)            /*!< Suspend mode request */
#define  USB_ISTR_WKUP                       ((unsigned short)0x1000)            /*!< Wake up */
#define  USB_ISTR_ERR                        ((unsigned short)0x2000)            /*!< Error */
#define  USB_ISTR_PMAOVR                     ((unsigned short)0x4000)            /*!< Packet Memory Area Over / Underrun */
#define  USB_ISTR_CTR                        ((unsigned short)0x8000)            /*!< Correct Transfer */

/*******************  Bit definition for USB_FNR register  ********************/
#define  USB_FNR_FN                          ((unsigned short)0x07FF)            /*!< Frame Number */
#define  USB_FNR_LSOF                        ((unsigned short)0x1800)            /*!< Lost SOF */
#define  USB_FNR_LCK                         ((unsigned short)0x2000)            /*!< Locked */
#define  USB_FNR_RXDM                        ((unsigned short)0x4000)            /*!< Receive Data - Line Status */
#define  USB_FNR_RXDP                        ((unsigned short)0x8000)            /*!< Receive Data + Line Status */

/******************  Bit definition for USB_DADDR register  *******************/
#define  USB_DADDR_ADD                       ((unsigned char)0x7F)               /*!< ADD[6:0] bits (Device Address) */
#define  USB_DADDR_ADD0                      ((unsigned char)0x01)               /*!< Bit 0 */
#define  USB_DADDR_ADD1                      ((unsigned char)0x02)               /*!< Bit 1 */
#define  USB_DADDR_ADD2                      ((unsigned char)0x04)               /*!< Bit 2 */
#define  USB_DADDR_ADD3                      ((unsigned char)0x08)               /*!< Bit 3 */
#define  USB_DADDR_ADD4                      ((unsigned char)0x10)               /*!< Bit 4 */
#define  USB_DADDR_ADD5                      ((unsigned char)0x20)               /*!< Bit 5 */
#define  USB_DADDR_ADD6                      ((unsigned char)0x40)               /*!< Bit 6 */

#define  USB_DADDR_EF                        ((unsigned char)0x80)               /*!< Enable Function */

/******************  Bit definition for USB_BTABLE register  ******************/
#define  USB_BTABLE_BTABLE                   ((unsigned short)0xFFF8)            /*!< Buffer Table */

/*!< Buffer descriptor table */
/*****************  Bit definition for USB_ADDR0_TX register  *****************/
#define  USB_ADDR0_TX_ADDR0_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 0 */

/*****************  Bit definition for USB_ADDR1_TX register  *****************/
#define  USB_ADDR1_TX_ADDR1_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 1 */

/*****************  Bit definition for USB_ADDR2_TX register  *****************/
#define  USB_ADDR2_TX_ADDR2_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 2 */

/*****************  Bit definition for USB_ADDR3_TX register  *****************/
#define  USB_ADDR3_TX_ADDR3_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 3 */

/*****************  Bit definition for USB_ADDR4_TX register  *****************/
#define  USB_ADDR4_TX_ADDR4_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 4 */

/*****************  Bit definition for USB_ADDR5_TX register  *****************/
#define  USB_ADDR5_TX_ADDR5_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 5 */

/*****************  Bit definition for USB_ADDR6_TX register  *****************/
#define  USB_ADDR6_TX_ADDR6_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 6 */

/*****************  Bit definition for USB_ADDR7_TX register  *****************/
#define  USB_ADDR7_TX_ADDR7_TX               ((unsigned short)0xFFFE)            /*!< Transmission Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_COUNT0_TX register  ****************/
#define  USB_COUNT0_TX_COUNT0_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 0 */

/*****************  Bit definition for USB_COUNT1_TX register  ****************/
#define  USB_COUNT1_TX_COUNT1_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 1 */

/*****************  Bit definition for USB_COUNT2_TX register  ****************/
#define  USB_COUNT2_TX_COUNT2_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 2 */

/*****************  Bit definition for USB_COUNT3_TX register  ****************/
#define  USB_COUNT3_TX_COUNT3_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 3 */

/*****************  Bit definition for USB_COUNT4_TX register  ****************/
#define  USB_COUNT4_TX_COUNT4_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 4 */

/*****************  Bit definition for USB_COUNT5_TX register  ****************/
#define  USB_COUNT5_TX_COUNT5_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 5 */

/*****************  Bit definition for USB_COUNT6_TX register  ****************/
#define  USB_COUNT6_TX_COUNT6_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 6 */

/*****************  Bit definition for USB_COUNT7_TX register  ****************/
#define  USB_COUNT7_TX_COUNT7_TX             ((unsigned short)0x03FF)            /*!< Transmission Byte Count 7 */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_COUNT0_TX_0 register  ***************/
#define  USB_COUNT0_TX_0_COUNT0_TX_0         ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 0 (low) */

/****************  Bit definition for USB_COUNT0_TX_1 register  ***************/
#define  USB_COUNT0_TX_1_COUNT0_TX_1         ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 0 (high) */

/****************  Bit definition for USB_COUNT1_TX_0 register  ***************/
#define  USB_COUNT1_TX_0_COUNT1_TX_0          ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 1 (low) */

/****************  Bit definition for USB_COUNT1_TX_1 register  ***************/
#define  USB_COUNT1_TX_1_COUNT1_TX_1          ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 1 (high) */

/****************  Bit definition for USB_COUNT2_TX_0 register  ***************/
#define  USB_COUNT2_TX_0_COUNT2_TX_0         ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 2 (low) */

/****************  Bit definition for USB_COUNT2_TX_1 register  ***************/
#define  USB_COUNT2_TX_1_COUNT2_TX_1         ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 2 (high) */

/****************  Bit definition for USB_COUNT3_TX_0 register  ***************/
#define  USB_COUNT3_TX_0_COUNT3_TX_0         ((unsigned short)0x000003FF)        /*!< Transmission Byte Count 3 (low) */

/****************  Bit definition for USB_COUNT3_TX_1 register  ***************/
#define  USB_COUNT3_TX_1_COUNT3_TX_1         ((unsigned short)0x03FF0000)        /*!< Transmission Byte Count 3 (high) */

/****************  Bit definition for USB_COUNT4_TX_0 register  ***************/
#define  USB_COUNT4_TX_0_COUNT4_TX_0         ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 4 (low) */

/****************  Bit definition for USB_COUNT4_TX_1 register  ***************/
#define  USB_COUNT4_TX_1_COUNT4_TX_1         ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 4 (high) */

/****************  Bit definition for USB_COUNT5_TX_0 register  ***************/
#define  USB_COUNT5_TX_0_COUNT5_TX_0         ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 5 (low) */

/****************  Bit definition for USB_COUNT5_TX_1 register  ***************/
#define  USB_COUNT5_TX_1_COUNT5_TX_1         ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 5 (high) */

/****************  Bit definition for USB_COUNT6_TX_0 register  ***************/
#define  USB_COUNT6_TX_0_COUNT6_TX_0         ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 6 (low) */

/****************  Bit definition for USB_COUNT6_TX_1 register  ***************/
#define  USB_COUNT6_TX_1_COUNT6_TX_1         ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 6 (high) */

/****************  Bit definition for USB_COUNT7_TX_0 register  ***************/
#define  USB_COUNT7_TX_0_COUNT7_TX_0         ((unsigned long)0x000003FF)        /*!< Transmission Byte Count 7 (low) */

/****************  Bit definition for USB_COUNT7_TX_1 register  ***************/
#define  USB_COUNT7_TX_1_COUNT7_TX_1         ((unsigned long)0x03FF0000)        /*!< Transmission Byte Count 7 (high) */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_ADDR0_RX register  *****************/
#define  USB_ADDR0_RX_ADDR0_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 0 */

/*****************  Bit definition for USB_ADDR1_RX register  *****************/
#define  USB_ADDR1_RX_ADDR1_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 1 */

/*****************  Bit definition for USB_ADDR2_RX register  *****************/
#define  USB_ADDR2_RX_ADDR2_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 2 */

/*****************  Bit definition for USB_ADDR3_RX register  *****************/
#define  USB_ADDR3_RX_ADDR3_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 3 */

/*****************  Bit definition for USB_ADDR4_RX register  *****************/
#define  USB_ADDR4_RX_ADDR4_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 4 */

/*****************  Bit definition for USB_ADDR5_RX register  *****************/
#define  USB_ADDR5_RX_ADDR5_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 5 */

/*****************  Bit definition for USB_ADDR6_RX register  *****************/
#define  USB_ADDR6_RX_ADDR6_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 6 */

/*****************  Bit definition for USB_ADDR7_RX register  *****************/
#define  USB_ADDR7_RX_ADDR7_RX               ((unsigned short)0xFFFE)            /*!< Reception Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_COUNT0_RX register  ****************/
#define  USB_COUNT0_RX_COUNT0_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT0_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT0_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT0_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT0_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT0_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT0_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT0_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT1_RX register  ****************/
#define  USB_COUNT1_RX_COUNT1_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT1_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT1_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT1_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT1_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT1_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT1_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT1_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT2_RX register  ****************/
#define  USB_COUNT2_RX_COUNT2_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT2_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT2_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT2_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT2_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT2_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT2_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT2_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT3_RX register  ****************/
#define  USB_COUNT3_RX_COUNT3_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT3_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT3_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT3_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT3_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT3_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT3_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT3_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT4_RX register  ****************/
#define  USB_COUNT4_RX_COUNT4_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT4_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT4_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT4_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT4_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT4_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT4_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT4_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT5_RX register  ****************/
#define  USB_COUNT5_RX_COUNT5_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT5_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT5_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT5_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT5_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT5_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT5_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT5_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT6_RX register  ****************/
#define  USB_COUNT6_RX_COUNT6_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT6_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT6_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT6_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT6_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT6_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT6_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT6_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT7_RX register  ****************/
#define  USB_COUNT7_RX_COUNT7_RX             ((unsigned short)0x03FF)            /*!< Reception Byte Count */

#define  USB_COUNT7_RX_NUM_BLOCK             ((unsigned short)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT7_RX_NUM_BLOCK_0           ((unsigned short)0x0400)            /*!< Bit 0 */
#define  USB_COUNT7_RX_NUM_BLOCK_1           ((unsigned short)0x0800)            /*!< Bit 1 */
#define  USB_COUNT7_RX_NUM_BLOCK_2           ((unsigned short)0x1000)            /*!< Bit 2 */
#define  USB_COUNT7_RX_NUM_BLOCK_3           ((unsigned short)0x2000)            /*!< Bit 3 */
#define  USB_COUNT7_RX_NUM_BLOCK_4           ((unsigned short)0x4000)            /*!< Bit 4 */

#define  USB_COUNT7_RX_BLSIZE                ((unsigned short)0x8000)            /*!< BLock SIZE */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_COUNT0_RX_0 register  ***************/
#define  USB_COUNT0_RX_0_COUNT0_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT0_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT0_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT0_RX_1 register  ***************/
#define  USB_COUNT0_RX_1_COUNT0_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT0_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 1 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT0_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT1_RX_0 register  ***************/
#define  USB_COUNT1_RX_0_COUNT1_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT1_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT1_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT1_RX_1 register  ***************/
#define  USB_COUNT1_RX_1_COUNT1_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT1_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT1_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT2_RX_0 register  ***************/
#define  USB_COUNT2_RX_0_COUNT2_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT2_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT2_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT2_RX_1 register  ***************/
#define  USB_COUNT2_RX_1_COUNT2_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT2_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT2_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT3_RX_0 register  ***************/
#define  USB_COUNT3_RX_0_COUNT3_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT3_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT3_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT3_RX_1 register  ***************/
#define  USB_COUNT3_RX_1_COUNT3_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT3_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT3_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT4_RX_0 register  ***************/
#define  USB_COUNT4_RX_0_COUNT4_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT4_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_0      ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_1      ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_2      ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_3      ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_4      ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT4_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT4_RX_1 register  ***************/
#define  USB_COUNT4_RX_1_COUNT4_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT4_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT4_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT5_RX_0 register  ***************/
#define  USB_COUNT5_RX_0_COUNT5_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT5_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT5_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT5_RX_1 register  ***************/
#define  USB_COUNT5_RX_1_COUNT5_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT5_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT5_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/***************  Bit definition for USB_COUNT6_RX_0  register  ***************/
#define  USB_COUNT6_RX_0_COUNT6_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT6_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT6_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT6_RX_1 register  ***************/
#define  USB_COUNT6_RX_1_COUNT6_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT6_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT6_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/***************  Bit definition for USB_COUNT7_RX_0 register  ****************/
#define  USB_COUNT7_RX_0_COUNT7_RX_0         ((unsigned long)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_COUNT7_RX_0_NUM_BLOCK_0         ((unsigned long)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_0       ((unsigned long)0x00000400)        /*!< Bit 0 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_1       ((unsigned long)0x00000800)        /*!< Bit 1 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_2       ((unsigned long)0x00001000)        /*!< Bit 2 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_3       ((unsigned long)0x00002000)        /*!< Bit 3 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_4       ((unsigned long)0x00004000)        /*!< Bit 4 */

#define  USB_COUNT7_RX_0_BLSIZE_0            ((unsigned long)0x00008000)        /*!< BLock SIZE (low) */

/***************  Bit definition for USB_COUNT7_RX_1 register  ****************/
#define  USB_COUNT7_RX_1_COUNT7_RX_1         ((unsigned long)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_COUNT7_RX_1_NUM_BLOCK_1         ((unsigned long)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_0       ((unsigned long)0x04000000)        /*!< Bit 0 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_1       ((unsigned long)0x08000000)        /*!< Bit 1 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_2       ((unsigned long)0x10000000)        /*!< Bit 2 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_3       ((unsigned long)0x20000000)        /*!< Bit 3 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_4       ((unsigned long)0x40000000)        /*!< Bit 4 */

#define  USB_COUNT7_RX_1_BLSIZE_1            ((unsigned long)0x80000000)        /*!< BLock SIZE (high) */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/

/*!< CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        ((unsigned short)0x0001)            /*!< Initialization Request */
#define  CAN_MCR_SLEEP                       ((unsigned short)0x0002)            /*!< Sleep Mode Request */
#define  CAN_MCR_TXFP                        ((unsigned short)0x0004)            /*!< Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        ((unsigned short)0x0008)            /*!< Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        ((unsigned short)0x0010)            /*!< No Automatic Retransmission */
#define  CAN_MCR_AWUM                        ((unsigned short)0x0020)            /*!< Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        ((unsigned short)0x0040)            /*!< Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        ((unsigned short)0x0080)            /*!< Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       ((unsigned short)0x8000)            /*!< CAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((unsigned short)0x0001)            /*!< Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((unsigned short)0x0002)            /*!< Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((unsigned short)0x0004)            /*!< Error Interrupt */
#define  CAN_MSR_WKUI                        ((unsigned short)0x0008)            /*!< Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((unsigned short)0x0010)            /*!< Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((unsigned short)0x0100)            /*!< Transmit Mode */
#define  CAN_MSR_RXM                         ((unsigned short)0x0200)            /*!< Receive Mode */
#define  CAN_MSR_SAMP                        ((unsigned short)0x0400)            /*!< Last Sample Point */
#define  CAN_MSR_RX                          ((unsigned short)0x0800)            /*!< CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       ((unsigned long)0x00000001)        /*!< Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       ((unsigned long)0x00000002)        /*!< Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       ((unsigned long)0x00000004)        /*!< Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       ((unsigned long)0x00000008)        /*!< Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       ((unsigned long)0x00000080)        /*!< Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       ((unsigned long)0x00000100)        /*!< Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       ((unsigned long)0x00000200)        /*!< Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       ((unsigned long)0x00000400)        /*!< Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       ((unsigned long)0x00000800)        /*!< Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       ((unsigned long)0x00008000)        /*!< Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       ((unsigned long)0x00010000)        /*!< Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       ((unsigned long)0x00020000)        /*!< Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       ((unsigned long)0x00040000)        /*!< Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       ((unsigned long)0x00080000)        /*!< Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       ((unsigned long)0x00800000)        /*!< Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        ((unsigned long)0x03000000)        /*!< Mailbox Code */

#define  CAN_TSR_TME                         ((unsigned long)0x1C000000)        /*!< TME[2:0] bits */
#define  CAN_TSR_TME0                        ((unsigned long)0x04000000)        /*!< Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        ((unsigned long)0x08000000)        /*!< Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        ((unsigned long)0x10000000)        /*!< Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         ((unsigned long)0xE0000000)        /*!< LOW[2:0] bits */
#define  CAN_TSR_LOW0                        ((unsigned long)0x20000000)        /*!< Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        ((unsigned long)0x40000000)        /*!< Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        ((unsigned long)0x80000000)        /*!< Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       ((unsigned char)0x03)               /*!< FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((unsigned char)0x08)               /*!< FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((unsigned char)0x10)               /*!< FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((unsigned char)0x20)               /*!< Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((unsigned char)0x03)               /*!< FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((unsigned char)0x08)               /*!< FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((unsigned char)0x10)               /*!< FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((unsigned char)0x20)               /*!< Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       ((unsigned long)0x00000001)        /*!< Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      ((unsigned long)0x00000002)        /*!< FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       ((unsigned long)0x00000004)        /*!< FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      ((unsigned long)0x00000008)        /*!< FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      ((unsigned long)0x00000010)        /*!< FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       ((unsigned long)0x00000020)        /*!< FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      ((unsigned long)0x00000040)        /*!< FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       ((unsigned long)0x00000100)        /*!< Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       ((unsigned long)0x00000200)        /*!< Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       ((unsigned long)0x00000400)        /*!< Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       ((unsigned long)0x00000800)        /*!< Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       ((unsigned long)0x00008000)        /*!< Error Interrupt Enable */
#define  CAN_IER_WKUIE                       ((unsigned long)0x00010000)        /*!< Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       ((unsigned long)0x00020000)        /*!< Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        ((unsigned long)0x00000001)        /*!< Error Warning Flag */
#define  CAN_ESR_EPVF                        ((unsigned long)0x00000002)        /*!< Error Passive Flag */
#define  CAN_ESR_BOFF                        ((unsigned long)0x00000004)        /*!< Bus-Off Flag */

#define  CAN_ESR_LEC                         ((unsigned long)0x00000070)        /*!< LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       ((unsigned long)0x00000010)        /*!< Bit 0 */
#define  CAN_ESR_LEC_1                       ((unsigned long)0x00000020)        /*!< Bit 1 */
#define  CAN_ESR_LEC_2                       ((unsigned long)0x00000040)        /*!< Bit 2 */

#define  CAN_ESR_TEC                         ((unsigned long)0x00FF0000)        /*!< Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         ((unsigned long)0xFF000000)        /*!< Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         ((unsigned long)0x000003FF)        /*!< Baud Rate Prescaler */
#define  CAN_BTR_TS1                         ((unsigned long)0x000F0000)        /*!< Time Segment 1 */
#define  CAN_BTR_TS2                         ((unsigned long)0x00700000)        /*!< Time Segment 2 */
#define  CAN_BTR_SJW                         ((unsigned long)0x03000000)        /*!< Resynchronization Jump Width */
#define  CAN_BTR_LBKM                        ((unsigned long)0x40000000)        /*!< Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        ((unsigned long)0x80000000)        /*!< Silent Mode */

/*!< Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       ((unsigned long)0x00000001)        /*!< Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        ((unsigned long)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_TI0R_IDE                        ((unsigned long)0x00000004)        /*!< Identifier Extension */
#define  CAN_TI0R_EXID                       ((unsigned long)0x001FFFF8)        /*!< Extended Identifier */
#define  CAN_TI0R_STID                       ((unsigned long)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       ((unsigned long)0x0000000F)        /*!< Data Length Code */
#define  CAN_TDT0R_TGT                       ((unsigned long)0x00000100)        /*!< Transmit Global Time */
#define  CAN_TDT0R_TIME                      ((unsigned long)0xFFFF0000)        /*!< Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     ((unsigned long)0x000000FF)        /*!< Data byte 0 */
#define  CAN_TDL0R_DATA1                     ((unsigned long)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_TDL0R_DATA2                     ((unsigned long)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_TDL0R_DATA3                     ((unsigned long)0xFF000000)        /*!< Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     ((unsigned long)0x000000FF)        /*!< Data byte 4 */
#define  CAN_TDH0R_DATA5                     ((unsigned long)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_TDH0R_DATA6                     ((unsigned long)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_TDH0R_DATA7                     ((unsigned long)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       ((unsigned long)0x00000001)        /*!< Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        ((unsigned long)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_TI1R_IDE                        ((unsigned long)0x00000004)        /*!< Identifier Extension */
#define  CAN_TI1R_EXID                       ((unsigned long)0x001FFFF8)        /*!< Extended Identifier */
#define  CAN_TI1R_STID                       ((unsigned long)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       ((unsigned long)0x0000000F)        /*!< Data Length Code */
#define  CAN_TDT1R_TGT                       ((unsigned long)0x00000100)        /*!< Transmit Global Time */
#define  CAN_TDT1R_TIME                      ((unsigned long)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     ((unsigned long)0x000000FF)        /*!< Data byte 0 */
#define  CAN_TDL1R_DATA1                     ((unsigned long)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_TDL1R_DATA2                     ((unsigned long)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_TDL1R_DATA3                     ((unsigned long)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     ((unsigned long)0x000000FF)        /*!< Data byte 4 */
#define  CAN_TDH1R_DATA5                     ((unsigned long)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_TDH1R_DATA6                     ((unsigned long)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_TDH1R_DATA7                     ((unsigned long)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       ((unsigned long)0x00000001)        /*!< Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        ((unsigned long)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_TI2R_IDE                        ((unsigned long)0x00000004)        /*!< Identifier Extension */
#define  CAN_TI2R_EXID                       ((unsigned long)0x001FFFF8)        /*!< Extended identifier */
#define  CAN_TI2R_STID                       ((unsigned long)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define  CAN_TDT2R_DLC                       ((unsigned long)0x0000000F)        /*!< Data Length Code */
#define  CAN_TDT2R_TGT                       ((unsigned long)0x00000100)        /*!< Transmit Global Time */
#define  CAN_TDT2R_TIME                      ((unsigned long)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     ((unsigned long)0x000000FF)        /*!< Data byte 0 */
#define  CAN_TDL2R_DATA1                     ((unsigned long)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_TDL2R_DATA2                     ((unsigned long)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_TDL2R_DATA3                     ((unsigned long)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     ((unsigned long)0x000000FF)        /*!< Data byte 4 */
#define  CAN_TDH2R_DATA5                     ((unsigned long)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_TDH2R_DATA6                     ((unsigned long)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_TDH2R_DATA7                     ((unsigned long)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        ((unsigned long)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_RI0R_IDE                        ((unsigned long)0x00000004)        /*!< Identifier Extension */
#define  CAN_RI0R_EXID                       ((unsigned long)0x001FFFF8)        /*!< Extended Identifier */
#define  CAN_RI0R_STID                       ((unsigned long)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       ((unsigned long)0x0000000F)        /*!< Data Length Code */
#define  CAN_RDT0R_FMI                       ((unsigned long)0x0000FF00)        /*!< Filter Match Index */
#define  CAN_RDT0R_TIME                      ((unsigned long)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     ((unsigned long)0x000000FF)        /*!< Data byte 0 */
#define  CAN_RDL0R_DATA1                     ((unsigned long)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_RDL0R_DATA2                     ((unsigned long)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_RDL0R_DATA3                     ((unsigned long)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     ((unsigned long)0x000000FF)        /*!< Data byte 4 */
#define  CAN_RDH0R_DATA5                     ((unsigned long)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_RDH0R_DATA6                     ((unsigned long)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_RDH0R_DATA7                     ((unsigned long)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        ((unsigned long)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_RI1R_IDE                        ((unsigned long)0x00000004)        /*!< Identifier Extension */
#define  CAN_RI1R_EXID                       ((unsigned long)0x001FFFF8)        /*!< Extended identifier */
#define  CAN_RI1R_STID                       ((unsigned long)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       ((unsigned long)0x0000000F)        /*!< Data Length Code */
#define  CAN_RDT1R_FMI                       ((unsigned long)0x0000FF00)        /*!< Filter Match Index */
#define  CAN_RDT1R_TIME                      ((unsigned long)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     ((unsigned long)0x000000FF)        /*!< Data byte 0 */
#define  CAN_RDL1R_DATA1                     ((unsigned long)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_RDL1R_DATA2                     ((unsigned long)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_RDL1R_DATA3                     ((unsigned long)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     ((unsigned long)0x000000FF)        /*!< Data byte 4 */
#define  CAN_RDH1R_DATA5                     ((unsigned long)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_RDH1R_DATA6                     ((unsigned long)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_RDH1R_DATA7                     ((unsigned long)0xFF000000)        /*!< Data byte 7 */

/*!< CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       ((unsigned char)0x01)               /*!< Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((unsigned short)0x3FFF)            /*!< Filter Mode */
#define  CAN_FM1R_FBM0                       ((unsigned short)0x0001)            /*!< Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       ((unsigned short)0x0002)            /*!< Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       ((unsigned short)0x0004)            /*!< Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       ((unsigned short)0x0008)            /*!< Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       ((unsigned short)0x0010)            /*!< Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       ((unsigned short)0x0020)            /*!< Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       ((unsigned short)0x0040)            /*!< Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       ((unsigned short)0x0080)            /*!< Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       ((unsigned short)0x0100)            /*!< Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       ((unsigned short)0x0200)            /*!< Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      ((unsigned short)0x0400)            /*!< Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      ((unsigned short)0x0800)            /*!< Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      ((unsigned short)0x1000)            /*!< Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      ((unsigned short)0x2000)            /*!< Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((unsigned short)0x3FFF)            /*!< Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       ((unsigned short)0x0001)            /*!< Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       ((unsigned short)0x0002)            /*!< Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       ((unsigned short)0x0004)            /*!< Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       ((unsigned short)0x0008)            /*!< Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       ((unsigned short)0x0010)            /*!< Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       ((unsigned short)0x0020)            /*!< Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       ((unsigned short)0x0040)            /*!< Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       ((unsigned short)0x0080)            /*!< Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       ((unsigned short)0x0100)            /*!< Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       ((unsigned short)0x0200)            /*!< Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      ((unsigned short)0x0400)            /*!< Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      ((unsigned short)0x0800)            /*!< Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      ((unsigned short)0x1000)            /*!< Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      ((unsigned short)0x2000)            /*!< Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       ((unsigned short)0x3FFF)            /*!< Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      ((unsigned short)0x0001)            /*!< Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      ((unsigned short)0x0002)            /*!< Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      ((unsigned short)0x0004)            /*!< Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      ((unsigned short)0x0008)            /*!< Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      ((unsigned short)0x0010)            /*!< Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      ((unsigned short)0x0020)            /*!< Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      ((unsigned short)0x0040)            /*!< Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      ((unsigned short)0x0080)            /*!< Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      ((unsigned short)0x0100)            /*!< Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      ((unsigned short)0x0200)            /*!< Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     ((unsigned short)0x0400)            /*!< Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     ((unsigned short)0x0800)            /*!< Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     ((unsigned short)0x1000)            /*!< Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     ((unsigned short)0x2000)            /*!< Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       ((unsigned short)0x3FFF)            /*!< Filter Active */
#define  CAN_FA1R_FACT0                      ((unsigned short)0x0001)            /*!< Filter 0 Active */
#define  CAN_FA1R_FACT1                      ((unsigned short)0x0002)            /*!< Filter 1 Active */
#define  CAN_FA1R_FACT2                      ((unsigned short)0x0004)            /*!< Filter 2 Active */
#define  CAN_FA1R_FACT3                      ((unsigned short)0x0008)            /*!< Filter 3 Active */
#define  CAN_FA1R_FACT4                      ((unsigned short)0x0010)            /*!< Filter 4 Active */
#define  CAN_FA1R_FACT5                      ((unsigned short)0x0020)            /*!< Filter 5 Active */
#define  CAN_FA1R_FACT6                      ((unsigned short)0x0040)            /*!< Filter 6 Active */
#define  CAN_FA1R_FACT7                      ((unsigned short)0x0080)            /*!< Filter 7 Active */
#define  CAN_FA1R_FACT8                      ((unsigned short)0x0100)            /*!< Filter 8 Active */
#define  CAN_FA1R_FACT9                      ((unsigned short)0x0200)            /*!< Filter 9 Active */
#define  CAN_FA1R_FACT10                     ((unsigned short)0x0400)            /*!< Filter 10 Active */
#define  CAN_FA1R_FACT11                     ((unsigned short)0x0800)            /*!< Filter 11 Active */
#define  CAN_FA1R_FACT12                     ((unsigned short)0x1000)            /*!< Filter 12 Active */
#define  CAN_FA1R_FACT13                     ((unsigned short)0x2000)            /*!< Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F0R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F0R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F0R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F0R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F0R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F0R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F0R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F0R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F0R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F0R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F0R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F0R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F0R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F0R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F0R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F0R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F0R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F0R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F0R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F0R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F0R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F0R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F0R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F0R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F0R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F0R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F0R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F0R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F0R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F0R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F0R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F1R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F1R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F1R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F1R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F1R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F1R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F1R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F1R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F1R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F1R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F1R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F1R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F1R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F1R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F1R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F1R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F1R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F1R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F1R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F1R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F1R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F1R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F1R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F1R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F1R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F1R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F1R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F1R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F1R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F1R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F1R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F2R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F2R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F2R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F2R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F2R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F2R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F2R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F2R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F2R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F2R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F2R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F2R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F2R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F2R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F2R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F2R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F2R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F2R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F2R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F2R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F2R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F2R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F2R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F2R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F2R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F2R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F2R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F2R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F2R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F2R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F2R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F3R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F3R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F3R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F3R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F3R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F3R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F3R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F3R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F3R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F3R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F3R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F3R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F3R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F3R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F3R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F3R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F3R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F3R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F3R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F3R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F3R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F3R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F3R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F3R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F3R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F3R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F3R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F3R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F3R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F3R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F3R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F4R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F4R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F4R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F4R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F4R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F4R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F4R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F4R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F4R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F4R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F4R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F4R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F4R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F4R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F4R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F4R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F4R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F4R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F4R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F4R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F4R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F4R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F4R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F4R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F4R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F4R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F4R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F4R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F4R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F4R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F4R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F5R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F5R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F5R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F5R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F5R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F5R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F5R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F5R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F5R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F5R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F5R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F5R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F5R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F5R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F5R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F5R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F5R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F5R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F5R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F5R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F5R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F5R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F5R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F5R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F5R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F5R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F5R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F5R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F5R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F5R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F5R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F6R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F6R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F6R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F6R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F6R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F6R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F6R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F6R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F6R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F6R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F6R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F6R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F6R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F6R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F6R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F6R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F6R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F6R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F6R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F6R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F6R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F6R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F6R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F6R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F6R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F6R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F6R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F6R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F6R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F6R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F6R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F7R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F7R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F7R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F7R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F7R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F7R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F7R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F7R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F7R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F7R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F7R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F7R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F7R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F7R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F7R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F7R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F7R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F7R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F7R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F7R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F7R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F7R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F7R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F7R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F7R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F7R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F7R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F7R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F7R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F7R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F7R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F8R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F8R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F8R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F8R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F8R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F8R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F8R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F8R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F8R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F8R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F8R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F8R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F8R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F8R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F8R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F8R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F8R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F8R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F8R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F8R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F8R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F8R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F8R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F8R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F8R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F8R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F8R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F8R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F8R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F8R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F8R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F9R1_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F9R1_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F9R1_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F9R1_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F9R1_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F9R1_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F9R1_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F9R1_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F9R1_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F9R1_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F9R1_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F9R1_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F9R1_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F9R1_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F9R1_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F9R1_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F9R1_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F9R1_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F9R1_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F9R1_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F9R1_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F9R1_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F9R1_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F9R1_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F9R1_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F9R1_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F9R1_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F9R1_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F9R1_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F9R1_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F9R1_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F10R1_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F10R1_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F10R1_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F10R1_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F10R1_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F10R1_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F10R1_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F10R1_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F10R1_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F10R1_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F10R1_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F10R1_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F10R1_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F10R1_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F10R1_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F10R1_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F10R1_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F10R1_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F10R1_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F10R1_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F10R1_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F10R1_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F10R1_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F10R1_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F10R1_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F10R1_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F10R1_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F10R1_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F10R1_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F10R1_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F10R1_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F11R1_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F11R1_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F11R1_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F11R1_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F11R1_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F11R1_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F11R1_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F11R1_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F11R1_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F11R1_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F11R1_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F11R1_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F11R1_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F11R1_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F11R1_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F11R1_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F11R1_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F11R1_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F11R1_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F11R1_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F11R1_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F11R1_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F11R1_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F11R1_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F11R1_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F11R1_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F11R1_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F11R1_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F11R1_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F11R1_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F11R1_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F12R1_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F12R1_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F12R1_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F12R1_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F12R1_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F12R1_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F12R1_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F12R1_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F12R1_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F12R1_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F12R1_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F12R1_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F12R1_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F12R1_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F12R1_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F12R1_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F12R1_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F12R1_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F12R1_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F12R1_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F12R1_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F12R1_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F12R1_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F12R1_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F12R1_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F12R1_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F12R1_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F12R1_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F12R1_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F12R1_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F12R1_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F13R1_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F13R1_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F13R1_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F13R1_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F13R1_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F13R1_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F13R1_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F13R1_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F13R1_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F13R1_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F13R1_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F13R1_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F13R1_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F13R1_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F13R1_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F13R1_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F13R1_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F13R1_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F13R1_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F13R1_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F13R1_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F13R1_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F13R1_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F13R1_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F13R1_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F13R1_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F13R1_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F13R1_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F13R1_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F13R1_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F13R1_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F0R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F0R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F0R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F0R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F0R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F0R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F0R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F0R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F0R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F0R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F0R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F0R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F0R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F0R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F0R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F0R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F0R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F0R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F0R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F0R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F0R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F0R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F0R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F0R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F0R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F0R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F0R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F0R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F0R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F0R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F0R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F1R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F1R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F1R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F1R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F1R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F1R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F1R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F1R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F1R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F1R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F1R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F1R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F1R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F1R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F1R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F1R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F1R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F1R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F1R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F1R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F1R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F1R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F1R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F1R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F1R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F1R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F1R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F1R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F1R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F1R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F1R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F2R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F2R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F2R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F2R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F2R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F2R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F2R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F2R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F2R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F2R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F2R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F2R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F2R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F2R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F2R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F2R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F2R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F2R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F2R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F2R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F2R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F2R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F2R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F2R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F2R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F2R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F2R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F2R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F2R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F2R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F2R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F3R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F3R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F3R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F3R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F3R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F3R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F3R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F3R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F3R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F3R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F3R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F3R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F3R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F3R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F3R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F3R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F3R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F3R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F3R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F3R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F3R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F3R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F3R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F3R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F3R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F3R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F3R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F3R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F3R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F3R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F3R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F4R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F4R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F4R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F4R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F4R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F4R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F4R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F4R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F4R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F4R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F4R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F4R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F4R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F4R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F4R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F4R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F4R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F4R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F4R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F4R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F4R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F4R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F4R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F4R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F4R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F4R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F4R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F4R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F4R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F4R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F4R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F5R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F5R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F5R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F5R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F5R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F5R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F5R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F5R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F5R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F5R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F5R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F5R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F5R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F5R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F5R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F5R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F5R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F5R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F5R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F5R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F5R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F5R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F5R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F5R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F5R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F5R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F5R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F5R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F5R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F5R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F5R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F6R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F6R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F6R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F6R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F6R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F6R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F6R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F6R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F6R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F6R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F6R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F6R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F6R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F6R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F6R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F6R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F6R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F6R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F6R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F6R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F6R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F6R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F6R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F6R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F6R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F6R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F6R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F6R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F6R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F6R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F6R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F7R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F7R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F7R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F7R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F7R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F7R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F7R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F7R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F7R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F7R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F7R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F7R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F7R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F7R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F7R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F7R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F7R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F7R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F7R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F7R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F7R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F7R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F7R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F7R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F7R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F7R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F7R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F7R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F7R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F7R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F7R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F8R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F8R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F8R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F8R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F8R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F8R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F8R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F8R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F8R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F8R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F8R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F8R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F8R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F8R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F8R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F8R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F8R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F8R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F8R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F8R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F8R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F8R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F8R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F8R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F8R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F8R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F8R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F8R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F8R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F8R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F8R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F9R2_FB1                        ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F9R2_FB2                        ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F9R2_FB3                        ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F9R2_FB4                        ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F9R2_FB5                        ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F9R2_FB6                        ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F9R2_FB7                        ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F9R2_FB8                        ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F9R2_FB9                        ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F9R2_FB10                       ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F9R2_FB11                       ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F9R2_FB12                       ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F9R2_FB13                       ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F9R2_FB14                       ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F9R2_FB15                       ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F9R2_FB16                       ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F9R2_FB17                       ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F9R2_FB18                       ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F9R2_FB19                       ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F9R2_FB20                       ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F9R2_FB21                       ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F9R2_FB22                       ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F9R2_FB23                       ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F9R2_FB24                       ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F9R2_FB25                       ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F9R2_FB26                       ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F9R2_FB27                       ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F9R2_FB28                       ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F9R2_FB29                       ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F9R2_FB30                       ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F9R2_FB31                       ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F10R2_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F10R2_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F10R2_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F10R2_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F10R2_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F10R2_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F10R2_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F10R2_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F10R2_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F10R2_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F10R2_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F10R2_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F10R2_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F10R2_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F10R2_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F10R2_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F10R2_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F10R2_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F10R2_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F10R2_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F10R2_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F10R2_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F10R2_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F10R2_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F10R2_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F10R2_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F10R2_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F10R2_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F10R2_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F10R2_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F10R2_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F11R2_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F11R2_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F11R2_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F11R2_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F11R2_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F11R2_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F11R2_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F11R2_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F11R2_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F11R2_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F11R2_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F11R2_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F11R2_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F11R2_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F11R2_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F11R2_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F11R2_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F11R2_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F11R2_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F11R2_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F11R2_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F11R2_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F11R2_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F11R2_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F11R2_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F11R2_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F11R2_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F11R2_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F11R2_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F11R2_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F11R2_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F12R2_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F12R2_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F12R2_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F12R2_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F12R2_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F12R2_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F12R2_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F12R2_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F12R2_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F12R2_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F12R2_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F12R2_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F12R2_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F12R2_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F12R2_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F12R2_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F12R2_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F12R2_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F12R2_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F12R2_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F12R2_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F12R2_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F12R2_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F12R2_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F12R2_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F12R2_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F12R2_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F12R2_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F12R2_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F12R2_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F12R2_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       ((unsigned long)0x00000001)        /*!< Filter bit 0 */
#define  CAN_F13R2_FB1                       ((unsigned long)0x00000002)        /*!< Filter bit 1 */
#define  CAN_F13R2_FB2                       ((unsigned long)0x00000004)        /*!< Filter bit 2 */
#define  CAN_F13R2_FB3                       ((unsigned long)0x00000008)        /*!< Filter bit 3 */
#define  CAN_F13R2_FB4                       ((unsigned long)0x00000010)        /*!< Filter bit 4 */
#define  CAN_F13R2_FB5                       ((unsigned long)0x00000020)        /*!< Filter bit 5 */
#define  CAN_F13R2_FB6                       ((unsigned long)0x00000040)        /*!< Filter bit 6 */
#define  CAN_F13R2_FB7                       ((unsigned long)0x00000080)        /*!< Filter bit 7 */
#define  CAN_F13R2_FB8                       ((unsigned long)0x00000100)        /*!< Filter bit 8 */
#define  CAN_F13R2_FB9                       ((unsigned long)0x00000200)        /*!< Filter bit 9 */
#define  CAN_F13R2_FB10                      ((unsigned long)0x00000400)        /*!< Filter bit 10 */
#define  CAN_F13R2_FB11                      ((unsigned long)0x00000800)        /*!< Filter bit 11 */
#define  CAN_F13R2_FB12                      ((unsigned long)0x00001000)        /*!< Filter bit 12 */
#define  CAN_F13R2_FB13                      ((unsigned long)0x00002000)        /*!< Filter bit 13 */
#define  CAN_F13R2_FB14                      ((unsigned long)0x00004000)        /*!< Filter bit 14 */
#define  CAN_F13R2_FB15                      ((unsigned long)0x00008000)        /*!< Filter bit 15 */
#define  CAN_F13R2_FB16                      ((unsigned long)0x00010000)        /*!< Filter bit 16 */
#define  CAN_F13R2_FB17                      ((unsigned long)0x00020000)        /*!< Filter bit 17 */
#define  CAN_F13R2_FB18                      ((unsigned long)0x00040000)        /*!< Filter bit 18 */
#define  CAN_F13R2_FB19                      ((unsigned long)0x00080000)        /*!< Filter bit 19 */
#define  CAN_F13R2_FB20                      ((unsigned long)0x00100000)        /*!< Filter bit 20 */
#define  CAN_F13R2_FB21                      ((unsigned long)0x00200000)        /*!< Filter bit 21 */
#define  CAN_F13R2_FB22                      ((unsigned long)0x00400000)        /*!< Filter bit 22 */
#define  CAN_F13R2_FB23                      ((unsigned long)0x00800000)        /*!< Filter bit 23 */
#define  CAN_F13R2_FB24                      ((unsigned long)0x01000000)        /*!< Filter bit 24 */
#define  CAN_F13R2_FB25                      ((unsigned long)0x02000000)        /*!< Filter bit 25 */
#define  CAN_F13R2_FB26                      ((unsigned long)0x04000000)        /*!< Filter bit 26 */
#define  CAN_F13R2_FB27                      ((unsigned long)0x08000000)        /*!< Filter bit 27 */
#define  CAN_F13R2_FB28                      ((unsigned long)0x10000000)        /*!< Filter bit 28 */
#define  CAN_F13R2_FB29                      ((unsigned long)0x20000000)        /*!< Filter bit 29 */
#define  CAN_F13R2_FB30                      ((unsigned long)0x40000000)        /*!< Filter bit 30 */
#define  CAN_F13R2_FB31                      ((unsigned long)0x80000000)        /*!< Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((unsigned short)0x0001)            /*!< Clock Phase */
#define  SPI_CR1_CPOL                        ((unsigned short)0x0002)            /*!< Clock Polarity */
#define  SPI_CR1_MSTR                        ((unsigned short)0x0004)            /*!< Master Selection */

#define  SPI_CR1_BR                          ((unsigned short)0x0038)            /*!< BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((unsigned short)0x0008)            /*!< Bit 0 */
#define  SPI_CR1_BR_1                        ((unsigned short)0x0010)            /*!< Bit 1 */
#define  SPI_CR1_BR_2                        ((unsigned short)0x0020)            /*!< Bit 2 */

#define  SPI_CR1_SPE                         ((unsigned short)0x0040)            /*!< SPI Enable */
#define  SPI_CR1_LSBFIRST                    ((unsigned short)0x0080)            /*!< Frame Format */
#define  SPI_CR1_SSI                         ((unsigned short)0x0100)            /*!< Internal slave select */
#define  SPI_CR1_SSM                         ((unsigned short)0x0200)            /*!< Software slave management */
#define  SPI_CR1_RXONLY                      ((unsigned short)0x0400)            /*!< Receive only */
#define  SPI_CR1_DFF                         ((unsigned short)0x0800)            /*!< Data Frame Format */
#define  SPI_CR1_CRCNEXT                     ((unsigned short)0x1000)            /*!< Transmit CRC next */
#define  SPI_CR1_CRCEN                       ((unsigned short)0x2000)            /*!< Hardware CRC calculation enable */
#define  SPI_CR1_BIDIOE                      ((unsigned short)0x4000)            /*!< Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((unsigned short)0x8000)            /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((unsigned char)0x01)               /*!< Rx Buffer DMA Enable */
#define  SPI_CR2_TXDMAEN                     ((unsigned char)0x02)               /*!< Tx Buffer DMA Enable */
#define  SPI_CR2_SSOE                        ((unsigned char)0x04)               /*!< SS Output Enable */
#define  SPI_CR2_ERRIE                       ((unsigned char)0x20)               /*!< Error Interrupt Enable */
#define  SPI_CR2_RXNEIE                      ((unsigned char)0x40)               /*!< RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((unsigned char)0x80)               /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((unsigned char)0x01)               /*!< Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((unsigned char)0x02)               /*!< Transmit buffer Empty */
#define  SPI_SR_CHSIDE                       ((unsigned char)0x04)               /*!< Channel side */
#define  SPI_SR_UDR                          ((unsigned char)0x08)               /*!< Underrun flag */
#define  SPI_SR_CRCERR                       ((unsigned char)0x10)               /*!< CRC Error flag */
#define  SPI_SR_MODF                         ((unsigned char)0x20)               /*!< Mode fault */
#define  SPI_SR_OVR                          ((unsigned char)0x40)               /*!< Overrun flag */
#define  SPI_SR_BSY                          ((unsigned char)0x80)               /*!< Busy flag */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((unsigned short)0xFFFF)            /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((unsigned short)0xFFFF)            /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((unsigned short)0xFFFF)            /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((unsigned short)0xFFFF)            /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((unsigned short)0x0001)            /*!< Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  ((unsigned short)0x0006)            /*!< DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCFGR_DATLEN_0                ((unsigned short)0x0002)            /*!< Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((unsigned short)0x0004)            /*!< Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   ((unsigned short)0x0008)            /*!< steady state clock polarity */

#define  SPI_I2SCFGR_I2SSTD                  ((unsigned short)0x0030)            /*!< I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((unsigned short)0x0010)            /*!< Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((unsigned short)0x0020)            /*!< Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 ((unsigned short)0x0080)            /*!< PCM frame synchronization */

#define  SPI_I2SCFGR_I2SCFG                  ((unsigned short)0x0300)            /*!< I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((unsigned short)0x0100)            /*!< Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((unsigned short)0x0200)            /*!< Bit 1 */

#define  SPI_I2SCFGR_I2SE                    ((unsigned short)0x0400)            /*!< I2S Enable */
#define  SPI_I2SCFGR_I2SMOD                  ((unsigned short)0x0800)            /*!< I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((unsigned short)0x00FF)            /*!< I2S Linear prescaler */
#define  SPI_I2SPR_ODD                       ((unsigned short)0x0100)            /*!< Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((unsigned short)0x0200)            /*!< Master Clock Output Enable */

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          ((unsigned short)0x0001)            /*!< Peripheral Enable */
#define  I2C_CR1_SMBUS                       ((unsigned short)0x0002)            /*!< SMBus Mode */
#define  I2C_CR1_SMBTYPE                     ((unsigned short)0x0008)            /*!< SMBus Type */
#define  I2C_CR1_ENARP                       ((unsigned short)0x0010)            /*!< ARP Enable */
#define  I2C_CR1_ENPEC                       ((unsigned short)0x0020)            /*!< PEC Enable */
#define  I2C_CR1_ENGC                        ((unsigned short)0x0040)            /*!< General Call Enable */
#define  I2C_CR1_NOSTRETCH                   ((unsigned short)0x0080)            /*!< Clock Stretching Disable (Slave mode) */
#define  I2C_CR1_START                       ((unsigned short)0x0100)            /*!< Start Generation */
#define  I2C_CR1_STOP                        ((unsigned short)0x0200)            /*!< Stop Generation */
#define  I2C_CR1_ACK                         ((unsigned short)0x0400)            /*!< Acknowledge Enable */
#define  I2C_CR1_POS                         ((unsigned short)0x0800)            /*!< Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         ((unsigned short)0x1000)            /*!< Packet Error Checking */
#define  I2C_CR1_ALERT                       ((unsigned short)0x2000)            /*!< SMBus Alert */
#define  I2C_CR1_SWRST                       ((unsigned short)0x8000)            /*!< Software Reset */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        ((unsigned short)0x003F)            /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define  I2C_CR2_FREQ_0                      ((unsigned short)0x0001)            /*!< Bit 0 */
#define  I2C_CR2_FREQ_1                      ((unsigned short)0x0002)            /*!< Bit 1 */
#define  I2C_CR2_FREQ_2                      ((unsigned short)0x0004)            /*!< Bit 2 */
#define  I2C_CR2_FREQ_3                      ((unsigned short)0x0008)            /*!< Bit 3 */
#define  I2C_CR2_FREQ_4                      ((unsigned short)0x0010)            /*!< Bit 4 */
#define  I2C_CR2_FREQ_5                      ((unsigned short)0x0020)            /*!< Bit 5 */

#define  I2C_CR2_ITERREN                     ((unsigned short)0x0100)            /*!< Error Interrupt Enable */
#define  I2C_CR2_ITEVTEN                     ((unsigned short)0x0200)            /*!< Event Interrupt Enable */
#define  I2C_CR2_ITBUFEN                     ((unsigned short)0x0400)            /*!< Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       ((unsigned short)0x0800)            /*!< DMA Requests Enable */
#define  I2C_CR2_LAST                        ((unsigned short)0x1000)            /*!< DMA Last Transfer */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     ((unsigned short)0x00FE)            /*!< Interface Address */
#define  I2C_OAR1_ADD8_9                     ((unsigned short)0x0300)            /*!< Interface Address */

#define  I2C_OAR1_ADD0                       ((unsigned short)0x0001)            /*!< Bit 0 */
#define  I2C_OAR1_ADD1                       ((unsigned short)0x0002)            /*!< Bit 1 */
#define  I2C_OAR1_ADD2                       ((unsigned short)0x0004)            /*!< Bit 2 */
#define  I2C_OAR1_ADD3                       ((unsigned short)0x0008)            /*!< Bit 3 */
#define  I2C_OAR1_ADD4                       ((unsigned short)0x0010)            /*!< Bit 4 */
#define  I2C_OAR1_ADD5                       ((unsigned short)0x0020)            /*!< Bit 5 */
#define  I2C_OAR1_ADD6                       ((unsigned short)0x0040)            /*!< Bit 6 */
#define  I2C_OAR1_ADD7                       ((unsigned short)0x0080)            /*!< Bit 7 */
#define  I2C_OAR1_ADD8                       ((unsigned short)0x0100)            /*!< Bit 8 */
#define  I2C_OAR1_ADD9                       ((unsigned short)0x0200)            /*!< Bit 9 */

#define  I2C_OAR1_ADDMODE                    ((unsigned short)0x8000)            /*!< Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     ((unsigned char)0x01)               /*!< Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       ((unsigned char)0xFE)               /*!< Interface address */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           ((unsigned char)0xFF)               /*!< 8-bit Data Register */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          ((unsigned short)0x0001)            /*!< Start Bit (Master mode) */
#define  I2C_SR1_ADDR                        ((unsigned short)0x0002)            /*!< Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         ((unsigned short)0x0004)            /*!< Byte Transfer Finished */
#define  I2C_SR1_ADD10                       ((unsigned short)0x0008)            /*!< 10-bit header sent (Master mode) */
#define  I2C_SR1_STOPF                       ((unsigned short)0x0010)            /*!< Stop detection (Slave mode) */
#define  I2C_SR1_RXNE                        ((unsigned short)0x0040)            /*!< Data Register not Empty (receivers) */
#define  I2C_SR1_TXE                         ((unsigned short)0x0080)            /*!< Data Register Empty (transmitters) */
#define  I2C_SR1_BERR                        ((unsigned short)0x0100)            /*!< Bus Error */
#define  I2C_SR1_ARLO                        ((unsigned short)0x0200)            /*!< Arbitration Lost (master mode) */
#define  I2C_SR1_AF                          ((unsigned short)0x0400)            /*!< Acknowledge Failure */
#define  I2C_SR1_OVR                         ((unsigned short)0x0800)            /*!< Overrun/Underrun */
#define  I2C_SR1_PECERR                      ((unsigned short)0x1000)            /*!< PEC Error in reception */
#define  I2C_SR1_TIMEOUT                     ((unsigned short)0x4000)            /*!< Timeout or Tlow Error */
#define  I2C_SR1_SMBALERT                    ((unsigned short)0x8000)            /*!< SMBus Alert */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         ((unsigned short)0x0001)            /*!< Master/Slave */
#define  I2C_SR2_BUSY                        ((unsigned short)0x0002)            /*!< Bus Busy */
#define  I2C_SR2_TRA                         ((unsigned short)0x0004)            /*!< Transmitter/Receiver */
#define  I2C_SR2_GENCALL                     ((unsigned short)0x0010)            /*!< General Call Address (Slave mode) */
#define  I2C_SR2_SMBDEFAULT                  ((unsigned short)0x0020)            /*!< SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     ((unsigned short)0x0040)            /*!< SMBus Host Header (Slave mode) */
#define  I2C_SR2_DUALF                       ((unsigned short)0x0080)            /*!< Dual Flag (Slave mode) */
#define  I2C_SR2_PEC                         ((unsigned short)0xFF00)            /*!< Packet Error Checking Register */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         ((unsigned short)0x0FFF)            /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        ((unsigned short)0x4000)            /*!< Fast Mode Duty Cycle */
#define  I2C_CCR_FS                          ((unsigned short)0x8000)            /*!< I2C Master Mode Selection */

/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     ((unsigned char)0x3F)               /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         ((unsigned short)0x0001)            /*!< Parity Error */
#define  USART_SR_FE                         ((unsigned short)0x0002)            /*!< Framing Error */
#define  USART_SR_NE                         ((unsigned short)0x0004)            /*!< Noise Error Flag */
#define  USART_SR_ORE                        ((unsigned short)0x0008)            /*!< OverRun Error */
#define  USART_SR_IDLE                       ((unsigned short)0x0010)            /*!< IDLE line detected */
#define  USART_SR_RXNE                       ((unsigned short)0x0020)            /*!< Read Data Register Not Empty */
#define  USART_SR_TC                         ((unsigned short)0x0040)            /*!< Transmission Complete */
#define  USART_SR_TXE                        ((unsigned short)0x0080)            /*!< Transmit Data Register Empty */
#define  USART_SR_LBD                        ((unsigned short)0x0100)            /*!< LIN Break Detection Flag */
#define  USART_SR_CTS                        ((unsigned short)0x0200)            /*!< CTS Flag */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         ((unsigned short)0x01FF)            /*!< Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              ((unsigned short)0x000F)            /*!< Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              ((unsigned short)0xFFF0)            /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       ((unsigned short)0x0001)            /*!< Send Break */
#define  USART_CR1_RWU                       ((unsigned short)0x0002)            /*!< Receiver wakeup */
#define  USART_CR1_RE                        ((unsigned short)0x0004)            /*!< Receiver Enable */
#define  USART_CR1_TE                        ((unsigned short)0x0008)            /*!< Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((unsigned short)0x0010)            /*!< IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((unsigned short)0x0020)            /*!< RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((unsigned short)0x0040)            /*!< Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((unsigned short)0x0080)            /*!< PE Interrupt Enable */
#define  USART_CR1_PEIE                      ((unsigned short)0x0100)            /*!< PE Interrupt Enable */
#define  USART_CR1_PS                        ((unsigned short)0x0200)            /*!< Parity Selection */
#define  USART_CR1_PCE                       ((unsigned short)0x0400)            /*!< Parity Control Enable */
#define  USART_CR1_WAKE                      ((unsigned short)0x0800)            /*!< Wakeup method */
#define  USART_CR1_M                         ((unsigned short)0x1000)            /*!< Word length */
#define  USART_CR1_UE                        ((unsigned short)0x2000)            /*!< USART Enable */
#define  USART_CR1_OVER8                     ((unsigned short)0x8000)            /*!< USART Oversmapling 8-bits */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       ((unsigned short)0x000F)            /*!< Address of the USART node */
#define  USART_CR2_LBDL                      ((unsigned short)0x0020)            /*!< LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((unsigned short)0x0040)            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((unsigned short)0x0100)            /*!< Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((unsigned short)0x0200)            /*!< Clock Phase */
#define  USART_CR2_CPOL                      ((unsigned short)0x0400)            /*!< Clock Polarity */
#define  USART_CR2_CLKEN                     ((unsigned short)0x0800)            /*!< Clock Enable */

#define  USART_CR2_STOP                      ((unsigned short)0x3000)            /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((unsigned short)0x1000)            /*!< Bit 0 */
#define  USART_CR2_STOP_1                    ((unsigned short)0x2000)            /*!< Bit 1 */

#define  USART_CR2_LINEN                     ((unsigned short)0x4000)            /*!< LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((unsigned short)0x0001)            /*!< Error Interrupt Enable */
#define  USART_CR3_IREN                      ((unsigned short)0x0002)            /*!< IrDA mode Enable */
#define  USART_CR3_IRLP                      ((unsigned short)0x0004)            /*!< IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((unsigned short)0x0008)            /*!< Half-Duplex Selection */
#define  USART_CR3_NACK                      ((unsigned short)0x0010)            /*!< Smartcard NACK enable */
#define  USART_CR3_SCEN                      ((unsigned short)0x0020)            /*!< Smartcard mode enable */
#define  USART_CR3_DMAR                      ((unsigned short)0x0040)            /*!< DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((unsigned short)0x0080)            /*!< DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((unsigned short)0x0100)            /*!< RTS Enable */
#define  USART_CR3_CTSE                      ((unsigned short)0x0200)            /*!< CTS Enable */
#define  USART_CR3_CTSIE                     ((unsigned short)0x0400)            /*!< CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    ((unsigned short)0x0800)            /*!< One Bit method */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((unsigned short)0x00FF)            /*!< PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    ((unsigned short)0x0001)            /*!< Bit 0 */
#define  USART_GTPR_PSC_1                    ((unsigned short)0x0002)            /*!< Bit 1 */
#define  USART_GTPR_PSC_2                    ((unsigned short)0x0004)            /*!< Bit 2 */
#define  USART_GTPR_PSC_3                    ((unsigned short)0x0008)            /*!< Bit 3 */
#define  USART_GTPR_PSC_4                    ((unsigned short)0x0010)            /*!< Bit 4 */
#define  USART_GTPR_PSC_5                    ((unsigned short)0x0020)            /*!< Bit 5 */
#define  USART_GTPR_PSC_6                    ((unsigned short)0x0040)            /*!< Bit 6 */
#define  USART_GTPR_PSC_7                    ((unsigned short)0x0080)            /*!< Bit 7 */

#define  USART_GTPR_GT                       ((unsigned short)0xFF00)            /*!< Guard time value */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define  DBGMCU_IDCODE_DEV_ID                ((unsigned long)0x00000FFF)        /*!< Device Identifier */

#define  DBGMCU_IDCODE_REV_ID                ((unsigned long)0xFFFF0000)        /*!< REV_ID[15:0] bits (Revision Identifier) */
#define  DBGMCU_IDCODE_REV_ID_0              ((unsigned long)0x00010000)        /*!< Bit 0 */
#define  DBGMCU_IDCODE_REV_ID_1              ((unsigned long)0x00020000)        /*!< Bit 1 */
#define  DBGMCU_IDCODE_REV_ID_2              ((unsigned long)0x00040000)        /*!< Bit 2 */
#define  DBGMCU_IDCODE_REV_ID_3              ((unsigned long)0x00080000)        /*!< Bit 3 */
#define  DBGMCU_IDCODE_REV_ID_4              ((unsigned long)0x00100000)        /*!< Bit 4 */
#define  DBGMCU_IDCODE_REV_ID_5              ((unsigned long)0x00200000)        /*!< Bit 5 */
#define  DBGMCU_IDCODE_REV_ID_6              ((unsigned long)0x00400000)        /*!< Bit 6 */
#define  DBGMCU_IDCODE_REV_ID_7              ((unsigned long)0x00800000)        /*!< Bit 7 */
#define  DBGMCU_IDCODE_REV_ID_8              ((unsigned long)0x01000000)        /*!< Bit 8 */
#define  DBGMCU_IDCODE_REV_ID_9              ((unsigned long)0x02000000)        /*!< Bit 9 */
#define  DBGMCU_IDCODE_REV_ID_10             ((unsigned long)0x04000000)        /*!< Bit 10 */
#define  DBGMCU_IDCODE_REV_ID_11             ((unsigned long)0x08000000)        /*!< Bit 11 */
#define  DBGMCU_IDCODE_REV_ID_12             ((unsigned long)0x10000000)        /*!< Bit 12 */
#define  DBGMCU_IDCODE_REV_ID_13             ((unsigned long)0x20000000)        /*!< Bit 13 */
#define  DBGMCU_IDCODE_REV_ID_14             ((unsigned long)0x40000000)        /*!< Bit 14 */
#define  DBGMCU_IDCODE_REV_ID_15             ((unsigned long)0x80000000)        /*!< Bit 15 */

/******************  Bit definition for DBGMCU_CR register  *******************/
#define  DBGMCU_CR_DBG_SLEEP                 ((unsigned long)0x00000001)        /*!< Debug Sleep Mode */
#define  DBGMCU_CR_DBG_STOP                  ((unsigned long)0x00000002)        /*!< Debug Stop Mode */
#define  DBGMCU_CR_DBG_STANDBY               ((unsigned long)0x00000004)        /*!< Debug Standby mode */
#define  DBGMCU_CR_TRACE_IOEN                ((unsigned long)0x00000020)        /*!< Trace Pin Assignment Control */

#define  DBGMCU_CR_TRACE_MODE                ((unsigned long)0x000000C0)        /*!< TRACE_MODE[1:0] bits (Trace Pin Assignment Control) */
#define  DBGMCU_CR_TRACE_MODE_0              ((unsigned long)0x00000040)        /*!< Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((unsigned long)0x00000080)        /*!< Bit 1 */

#define  DBGMCU_CR_DBG_IWDG_STOP             ((unsigned long)0x00000100)        /*!< Debug Independent Watchdog stopped when Core is halted */
#define  DBGMCU_CR_DBG_WWDG_STOP             ((unsigned long)0x00000200)        /*!< Debug Window Watchdog stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM1_STOP             ((unsigned long)0x00000400)        /*!< TIM1 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM2_STOP             ((unsigned long)0x00000800)        /*!< TIM2 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM3_STOP             ((unsigned long)0x00001000)        /*!< TIM3 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM4_STOP             ((unsigned long)0x00002000)        /*!< TIM4 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_CAN1_STOP             ((unsigned long)0x00004000)        /*!< Debug CAN1 stopped when Core is halted */
#define  DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT    ((unsigned long)0x00008000)        /*!< SMBUS timeout mode stopped when Core is halted */
#define  DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT    ((unsigned long)0x00010000)        /*!< SMBUS timeout mode stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM8_STOP             ((unsigned long)0x00020000)        /*!< TIM8 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM5_STOP             ((unsigned long)0x00040000)        /*!< TIM5 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM6_STOP             ((unsigned long)0x00080000)        /*!< TIM6 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM7_STOP             ((unsigned long)0x00100000)        /*!< TIM7 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_CAN2_STOP             ((unsigned long)0x00200000)        /*!< Debug CAN2 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM15_STOP            ((unsigned long)0x00400000)        /*!< Debug TIM15 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM16_STOP            ((unsigned long)0x00800000)        /*!< Debug TIM16 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM17_STOP            ((unsigned long)0x01000000)        /*!< Debug TIM17 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM12_STOP            ((unsigned long)0x02000000)        /*!< Debug TIM12 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM13_STOP            ((unsigned long)0x04000000)        /*!< Debug TIM13 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM14_STOP            ((unsigned long)0x08000000)        /*!< Debug TIM14 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM9_STOP             ((unsigned long)0x10000000)        /*!< Debug TIM9 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM10_STOP            ((unsigned long)0x20000000)        /*!< Debug TIM10 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM11_STOP            ((unsigned long)0x40000000)        /*!< Debug TIM11 stopped when Core is halted */

/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACR register  ******************/
#define  FLASH_ACR_LATENCY                   ((unsigned char)0x03)               /*!< LATENCY[2:0] bits (Latency) */
#define  FLASH_ACR_LATENCY_0                 ((unsigned char)0x00)               /*!< Bit 0 */
#define  FLASH_ACR_LATENCY_1                 ((unsigned char)0x01)               /*!< Bit 0 */
#define  FLASH_ACR_LATENCY_2                 ((unsigned char)0x02)               /*!< Bit 1 */

#define  FLASH_ACR_HLFCYA                    ((unsigned char)0x08)               /*!< Flash Half Cycle Access Enable */
#define  FLASH_ACR_PRFTBE                    ((unsigned char)0x10)               /*!< Prefetch Buffer Enable */
#define  FLASH_ACR_PRFTBS                    ((unsigned char)0x20)               /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define  FLASH_KEYR_FKEYR                    ((unsigned long)0xFFFFFFFF)        /*!< FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define  FLASH_OPTKEYR_OPTKEYR               ((unsigned long)0xFFFFFFFF)        /*!< Option Byte Key */

/******************  Bit definition for FLASH_SR register  *******************/
#define  FLASH_SR_BSY                        ((unsigned char)0x01)               /*!< Busy */
#define  FLASH_SR_PGERR                      ((unsigned char)0x04)               /*!< Programming Error */
#define  FLASH_SR_WRPRTERR                   ((unsigned char)0x10)               /*!< Write Protection Error */
#define  FLASH_SR_EOP                        ((unsigned char)0x20)               /*!< End of operation */

/*******************  Bit definition for FLASH_CR register  *******************/
#define  FLASH_CR_PG                         ((unsigned short)0x0001)            /*!< Programming */
#define  FLASH_CR_PER                        ((unsigned short)0x0002)            /*!< Page Erase */
#define  FLASH_CR_MER                        ((unsigned short)0x0004)            /*!< Mass Erase */
#define  FLASH_CR_OPTPG                      ((unsigned short)0x0010)            /*!< Option Byte Programming */
#define  FLASH_CR_OPTER                      ((unsigned short)0x0020)            /*!< Option Byte Erase */
#define  FLASH_CR_STRT                       ((unsigned short)0x0040)            /*!< Start */
#define  FLASH_CR_LOCK                       ((unsigned short)0x0080)            /*!< Lock */
#define  FLASH_CR_OPTWRE                     ((unsigned short)0x0200)            /*!< Option Bytes Write Enable */
#define  FLASH_CR_ERRIE                      ((unsigned short)0x0400)            /*!< Error Interrupt Enable */
#define  FLASH_CR_EOPIE                      ((unsigned short)0x1000)            /*!< End of operation interrupt enable */

/*******************  Bit definition for FLASH_AR register  *******************/
#define  FLASH_AR_FAR                        ((unsigned long)0xFFFFFFFF)        /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define  FLASH_OBR_OPTERR                    ((unsigned short)0x0001)            /*!< Option Byte Error */
#define  FLASH_OBR_RDPRT                     ((unsigned short)0x0002)            /*!< Read protection */

#define  FLASH_OBR_USER                      ((unsigned short)0x03FC)            /*!< User Option Bytes */
#define  FLASH_OBR_WDG_SW                    ((unsigned short)0x0004)            /*!< WDG_SW */
#define  FLASH_OBR_nRST_STOP                 ((unsigned short)0x0008)            /*!< nRST_STOP */
#define  FLASH_OBR_nRST_STDBY                ((unsigned short)0x0010)            /*!< nRST_STDBY */
#define  FLASH_OBR_BFB2                      ((unsigned short)0x0020)            /*!< BFB2 */

/******************  Bit definition for FLASH_WRPR register  ******************/
#define  FLASH_WRPR_WRP                        ((unsigned long)0xFFFFFFFF)        /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for FLASH_RDP register  *******************/
#define  FLASH_RDP_RDP                       ((unsigned long)0x000000FF)        /*!< Read protection option byte */
#define  FLASH_RDP_nRDP                      ((unsigned long)0x0000FF00)        /*!< Read protection complemented option byte */

/******************  Bit definition for FLASH_USER register  ******************/
#define  FLASH_USER_USER                     ((unsigned long)0x00FF0000)        /*!< User option byte */
#define  FLASH_USER_nUSER                    ((unsigned long)0xFF000000)        /*!< User complemented option byte */

/******************  Bit definition for FLASH_Data0 register  *****************/
#define  FLASH_Data0_Data0                   ((unsigned long)0x000000FF)        /*!< User data storage option byte */
#define  FLASH_Data0_nData0                  ((unsigned long)0x0000FF00)        /*!< User data storage complemented option byte */

/******************  Bit definition for FLASH_Data1 register  *****************/
#define  FLASH_Data1_Data1                   ((unsigned long)0x00FF0000)        /*!< User data storage option byte */
#define  FLASH_Data1_nData1                  ((unsigned long)0xFF000000)        /*!< User data storage complemented option byte */

/******************  Bit definition for FLASH_WRP0 register  ******************/
#define  FLASH_WRP0_WRP0                     ((unsigned long)0x000000FF)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRP0_nWRP0                    ((unsigned long)0x0000FF00)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP1 register  ******************/
#define  FLASH_WRP1_WRP1                     ((unsigned long)0x00FF0000)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRP1_nWRP1                    ((unsigned long)0xFF000000)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP2 register  ******************/
#define  FLASH_WRP2_WRP2                     ((unsigned long)0x000000FF)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRP2_nWRP2                    ((unsigned long)0x0000FF00)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP3 register  ******************/
#define  FLASH_WRP3_WRP3                     ((unsigned long)0x00FF0000)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRP3_nWRP3                    ((unsigned long)0xFF000000)        /*!< Flash memory write protection complemented option bytes */

#ifdef STM32F10X_CL
/******************************************************************************/
/*                Ethernet MAC Registers bits definitions                     */
/******************************************************************************/
/* Bit definition for Ethernet MAC Control Register register */
#define ETH_MACCR_WD      ((unsigned long)0x00800000)  /* Watchdog disable */
#define ETH_MACCR_JD      ((unsigned long)0x00400000)  /* Jabber disable */
#define ETH_MACCR_IFG     ((unsigned long)0x000E0000)  /* Inter-frame gap */
  #define ETH_MACCR_IFG_96Bit     ((unsigned long)0x00000000)  /* Minimum IFG between frames during transmission is 96Bit */
  #define ETH_MACCR_IFG_88Bit     ((unsigned long)0x00020000)  /* Minimum IFG between frames during transmission is 88Bit */
  #define ETH_MACCR_IFG_80Bit     ((unsigned long)0x00040000)  /* Minimum IFG between frames during transmission is 80Bit */
  #define ETH_MACCR_IFG_72Bit     ((unsigned long)0x00060000)  /* Minimum IFG between frames during transmission is 72Bit */
  #define ETH_MACCR_IFG_64Bit     ((unsigned long)0x00080000)  /* Minimum IFG between frames during transmission is 64Bit */
  #define ETH_MACCR_IFG_56Bit     ((unsigned long)0x000A0000)  /* Minimum IFG between frames during transmission is 56Bit */
  #define ETH_MACCR_IFG_48Bit     ((unsigned long)0x000C0000)  /* Minimum IFG between frames during transmission is 48Bit */
  #define ETH_MACCR_IFG_40Bit     ((unsigned long)0x000E0000)  /* Minimum IFG between frames during transmission is 40Bit */
#define ETH_MACCR_CSD     ((unsigned long)0x00010000)  /* Carrier sense disable (during transmission) */
#define ETH_MACCR_FES     ((unsigned long)0x00004000)  /* Fast ethernet speed */
#define ETH_MACCR_ROD     ((unsigned long)0x00002000)  /* Receive own disable */
#define ETH_MACCR_LM      ((unsigned long)0x00001000)  /* loopback mode */
#define ETH_MACCR_DM      ((unsigned long)0x00000800)  /* Duplex mode */
#define ETH_MACCR_IPCO    ((unsigned long)0x00000400)  /* IP Checksum offload */
#define ETH_MACCR_RD      ((unsigned long)0x00000200)  /* Retry disable */
#define ETH_MACCR_APCS    ((unsigned long)0x00000080)  /* Automatic Pad/CRC stripping */
#define ETH_MACCR_BL      ((unsigned long)0x00000060)  /* Back-off limit: random integer number (r) of slot time delays before rescheduling
                                                       a transmission attempt during retries after a collision: 0 =< r <2^k */
  #define ETH_MACCR_BL_10    ((unsigned long)0x00000000)  /* k = min (n, 10) */
  #define ETH_MACCR_BL_8     ((unsigned long)0x00000020)  /* k = min (n, 8) */
  #define ETH_MACCR_BL_4     ((unsigned long)0x00000040)  /* k = min (n, 4) */
  #define ETH_MACCR_BL_1     ((unsigned long)0x00000060)  /* k = min (n, 1) */
#define ETH_MACCR_DC      ((unsigned long)0x00000010)  /* Defferal check */
#define ETH_MACCR_TE      ((unsigned long)0x00000008)  /* Transmitter enable */
#define ETH_MACCR_RE      ((unsigned long)0x00000004)  /* Receiver enable */

/* Bit definition for Ethernet MAC Frame Filter Register */
#define ETH_MACFFR_RA     ((unsigned long)0x80000000)  /* Receive all */
#define ETH_MACFFR_HPF    ((unsigned long)0x00000400)  /* Hash or perfect filter */
#define ETH_MACFFR_SAF    ((unsigned long)0x00000200)  /* Source address filter enable */
#define ETH_MACFFR_SAIF   ((unsigned long)0x00000100)  /* SA inverse filtering */
#define ETH_MACFFR_PCF    ((unsigned long)0x000000C0)  /* Pass control frames: 3 cases */
  #define ETH_MACFFR_PCF_BlockAll                ((unsigned long)0x00000040)  /* MAC filters all control frames from reaching the application */
  #define ETH_MACFFR_PCF_ForwardAll              ((unsigned long)0x00000080)  /* MAC forwards all control frames to application even if they fail the Address Filter */
  #define ETH_MACFFR_PCF_ForwardPassedAddrFilter ((unsigned long)0x000000C0)  /* MAC forwards control frames that pass the Address Filter. */
#define ETH_MACFFR_BFD    ((unsigned long)0x00000020)  /* Broadcast frame disable */
#define ETH_MACFFR_PAM 	  ((unsigned long)0x00000010)  /* Pass all mutlicast */
#define ETH_MACFFR_DAIF   ((unsigned long)0x00000008)  /* DA Inverse filtering */
#define ETH_MACFFR_HM     ((unsigned long)0x00000004)  /* Hash multicast */
#define ETH_MACFFR_HU     ((unsigned long)0x00000002)  /* Hash unicast */
#define ETH_MACFFR_PM     ((unsigned long)0x00000001)  /* Promiscuous mode */

/* Bit definition for Ethernet MAC Hash Table High Register */
#define ETH_MACHTHR_HTH   ((unsigned long)0xFFFFFFFF)  /* Hash table high */

/* Bit definition for Ethernet MAC Hash Table Low Register */
#define ETH_MACHTLR_HTL   ((unsigned long)0xFFFFFFFF)  /* Hash table low */

/* Bit definition for Ethernet MAC MII Address Register */
#define ETH_MACMIIAR_PA   ((unsigned long)0x0000F800)  /* Physical layer address */
#define ETH_MACMIIAR_MR   ((unsigned long)0x000007C0)  /* MII register in the selected PHY */
#define ETH_MACMIIAR_CR   ((unsigned long)0x0000001C)  /* CR clock range: 6 cases */
  #define ETH_MACMIIAR_CR_Div42   ((unsigned long)0x00000000)  /* HCLK:60-72 MHz; MDC clock= HCLK/42 */
  #define ETH_MACMIIAR_CR_Div16   ((unsigned long)0x00000008)  /* HCLK:20-35 MHz; MDC clock= HCLK/16 */
  #define ETH_MACMIIAR_CR_Div26   ((unsigned long)0x0000000C)  /* HCLK:35-60 MHz; MDC clock= HCLK/26 */
#define ETH_MACMIIAR_MW   ((unsigned long)0x00000002)  /* MII write */
#define ETH_MACMIIAR_MB   ((unsigned long)0x00000001)  /* MII busy */

/* Bit definition for Ethernet MAC MII Data Register */
#define ETH_MACMIIDR_MD   ((unsigned long)0x0000FFFF)  /* MII data: read/write data from/to PHY */

/* Bit definition for Ethernet MAC Flow Control Register */
#define ETH_MACFCR_PT     ((unsigned long)0xFFFF0000)  /* Pause time */
#define ETH_MACFCR_ZQPD   ((unsigned long)0x00000080)  /* Zero-quanta pause disable */
#define ETH_MACFCR_PLT    ((unsigned long)0x00000030)  /* Pause low threshold: 4 cases */
  #define ETH_MACFCR_PLT_Minus4   ((unsigned long)0x00000000)  /* Pause time minus 4 slot times */
  #define ETH_MACFCR_PLT_Minus28  ((unsigned long)0x00000010)  /* Pause time minus 28 slot times */
  #define ETH_MACFCR_PLT_Minus144 ((unsigned long)0x00000020)  /* Pause time minus 144 slot times */
  #define ETH_MACFCR_PLT_Minus256 ((unsigned long)0x00000030)  /* Pause time minus 256 slot times */
#define ETH_MACFCR_UPFD   ((unsigned long)0x00000008)  /* Unicast pause frame detect */
#define ETH_MACFCR_RFCE   ((unsigned long)0x00000004)  /* Receive flow control enable */
#define ETH_MACFCR_TFCE   ((unsigned long)0x00000002)  /* Transmit flow control enable */
#define ETH_MACFCR_FCBBPA ((unsigned long)0x00000001)  /* Flow control busy/backpressure activate */

/* Bit definition for Ethernet MAC VLAN Tag Register */
#define ETH_MACVLANTR_VLANTC ((unsigned long)0x00010000)  /* 12-bit VLAN tag comparison */
#define ETH_MACVLANTR_VLANTI ((unsigned long)0x0000FFFF)  /* VLAN tag identifier (for receive frames) */

/* Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register */
#define ETH_MACRWUFFR_D   ((unsigned long)0xFFFFFFFF)  /* Wake-up frame filter register data */
/* Eight sequential Writes to this address (offset 0x28) will write all Wake-UpFrame Filter Registers.
   Eight sequential Reads from this address (offset 0x28) will read all Wake-UpFrame Filter Registers. */
/* Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
   Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
   Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
   Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
   Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command -
                              RSVD - Filter1 Command - RSVD - Filter0 Command
   Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
   Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
   Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16 */

/* Bit definition for Ethernet MAC PMT Control and Status Register */
#define ETH_MACPMTCSR_WFFRPR ((unsigned long)0x80000000)  /* Wake-Up Frame Filter Register Pointer Reset */
#define ETH_MACPMTCSR_GU     ((unsigned long)0x00000200)  /* Global Unicast */
#define ETH_MACPMTCSR_WFR    ((unsigned long)0x00000040)  /* Wake-Up Frame Received */
#define ETH_MACPMTCSR_MPR    ((unsigned long)0x00000020)  /* Magic Packet Received */
#define ETH_MACPMTCSR_WFE    ((unsigned long)0x00000004)  /* Wake-Up Frame Enable */
#define ETH_MACPMTCSR_MPE    ((unsigned long)0x00000002)  /* Magic Packet Enable */
#define ETH_MACPMTCSR_PD     ((unsigned long)0x00000001)  /* Power Down */

/* Bit definition for Ethernet MAC Status Register */
#define ETH_MACSR_TSTS      ((unsigned long)0x00000200)  /* Time stamp trigger status */
#define ETH_MACSR_MMCTS     ((unsigned long)0x00000040)  /* MMC transmit status */
#define ETH_MACSR_MMMCRS    ((unsigned long)0x00000020)  /* MMC receive status */
#define ETH_MACSR_MMCS      ((unsigned long)0x00000010)  /* MMC status */
#define ETH_MACSR_PMTS      ((unsigned long)0x00000008)  /* PMT status */

/* Bit definition for Ethernet MAC Interrupt Mask Register */
#define ETH_MACIMR_TSTIM     ((unsigned long)0x00000200)  /* Time stamp trigger interrupt mask */
#define ETH_MACIMR_PMTIM     ((unsigned long)0x00000008)  /* PMT interrupt mask */

/* Bit definition for Ethernet MAC Address0 High Register */
#define ETH_MACA0HR_MACA0H   ((unsigned long)0x0000FFFF)  /* MAC address0 high */

/* Bit definition for Ethernet MAC Address0 Low Register */
#define ETH_MACA0LR_MACA0L   ((unsigned long)0xFFFFFFFF)  /* MAC address0 low */

/* Bit definition for Ethernet MAC Address1 High Register */
#define ETH_MACA1HR_AE       ((unsigned long)0x80000000)  /* Address enable */
#define ETH_MACA1HR_SA       ((unsigned long)0x40000000)  /* Source address */
#define ETH_MACA1HR_MBC      ((unsigned long)0x3F000000)  /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
  #define ETH_MACA1HR_MBC_HBits15_8    ((unsigned long)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
  #define ETH_MACA1HR_MBC_HBits7_0     ((unsigned long)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
  #define ETH_MACA1HR_MBC_LBits31_24   ((unsigned long)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
  #define ETH_MACA1HR_MBC_LBits23_16   ((unsigned long)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
  #define ETH_MACA1HR_MBC_LBits15_8    ((unsigned long)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
  #define ETH_MACA1HR_MBC_LBits7_0     ((unsigned long)0x01000000)  /* Mask MAC Address low reg bits [7:0] */
#define ETH_MACA1HR_MACA1H   ((unsigned long)0x0000FFFF)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address1 Low Register */
#define ETH_MACA1LR_MACA1L   ((unsigned long)0xFFFFFFFF)  /* MAC address1 low */

/* Bit definition for Ethernet MAC Address2 High Register */
#define ETH_MACA2HR_AE       ((unsigned long)0x80000000)  /* Address enable */
#define ETH_MACA2HR_SA       ((unsigned long)0x40000000)  /* Source address */
#define ETH_MACA2HR_MBC      ((unsigned long)0x3F000000)  /* Mask byte control */
  #define ETH_MACA2HR_MBC_HBits15_8    ((unsigned long)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
  #define ETH_MACA2HR_MBC_HBits7_0     ((unsigned long)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
  #define ETH_MACA2HR_MBC_LBits31_24   ((unsigned long)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
  #define ETH_MACA2HR_MBC_LBits23_16   ((unsigned long)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
  #define ETH_MACA2HR_MBC_LBits15_8    ((unsigned long)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
  #define ETH_MACA2HR_MBC_LBits7_0     ((unsigned long)0x01000000)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA2HR_MACA2H   ((unsigned long)0x0000FFFF)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address2 Low Register */
#define ETH_MACA2LR_MACA2L   ((unsigned long)0xFFFFFFFF)  /* MAC address2 low */

/* Bit definition for Ethernet MAC Address3 High Register */
#define ETH_MACA3HR_AE       ((unsigned long)0x80000000)  /* Address enable */
#define ETH_MACA3HR_SA       ((unsigned long)0x40000000)  /* Source address */
#define ETH_MACA3HR_MBC      ((unsigned long)0x3F000000)  /* Mask byte control */
  #define ETH_MACA3HR_MBC_HBits15_8    ((unsigned long)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
  #define ETH_MACA3HR_MBC_HBits7_0     ((unsigned long)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
  #define ETH_MACA3HR_MBC_LBits31_24   ((unsigned long)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
  #define ETH_MACA3HR_MBC_LBits23_16   ((unsigned long)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
  #define ETH_MACA3HR_MBC_LBits15_8    ((unsigned long)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
  #define ETH_MACA3HR_MBC_LBits7_0     ((unsigned long)0x01000000)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA3HR_MACA3H   ((unsigned long)0x0000FFFF)  /* MAC address3 high */

/* Bit definition for Ethernet MAC Address3 Low Register */
#define ETH_MACA3LR_MACA3L   ((unsigned long)0xFFFFFFFF)  /* MAC address3 low */

/******************************************************************************/
/*                Ethernet MMC Registers bits definition                      */
/******************************************************************************/

/* Bit definition for Ethernet MMC Contol Register */
#define ETH_MMCCR_MCF        ((unsigned long)0x00000008)  /* MMC Counter Freeze */
#define ETH_MMCCR_ROR        ((unsigned long)0x00000004)  /* Reset on Read */
#define ETH_MMCCR_CSR        ((unsigned long)0x00000002)  /* Counter Stop Rollover */
#define ETH_MMCCR_CR         ((unsigned long)0x00000001)  /* Counters Reset */

/* Bit definition for Ethernet MMC Receive Interrupt Register */
#define ETH_MMCRIR_RGUFS     ((unsigned long)0x00020000)  /* Set when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIR_RFAES     ((unsigned long)0x00000040)  /* Set when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIR_RFCES     ((unsigned long)0x00000020)  /* Set when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Register */
#define ETH_MMCTIR_TGFS      ((unsigned long)0x00200000)  /* Set when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIR_TGFMSCS   ((unsigned long)0x00008000)  /* Set when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIR_TGFSCS    ((unsigned long)0x00004000)  /* Set when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Receive Interrupt Mask Register */
#define ETH_MMCRIMR_RGUFM    ((unsigned long)0x00020000)  /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIMR_RFAEM    ((unsigned long)0x00000040)  /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIMR_RFCEM    ((unsigned long)0x00000020)  /* Mask the interrupt when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Mask Register */
#define ETH_MMCTIMR_TGFM     ((unsigned long)0x00200000)  /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFMSCM  ((unsigned long)0x00008000)  /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFSCM   ((unsigned long)0x00004000)  /* Mask the interrupt when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register */
#define ETH_MMCTGFSCCR_TGFSCC     ((unsigned long)0xFFFFFFFF)  /* Number of successfully transmitted frames after a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register */
#define ETH_MMCTGFMSCCR_TGFMSCC   ((unsigned long)0xFFFFFFFF)  /* Number of successfully transmitted frames after more than a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames Counter Register */
#define ETH_MMCTGFCR_TGFC    ((unsigned long)0xFFFFFFFF)  /* Number of good frames transmitted. */

/* Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register */
#define ETH_MMCRFCECR_RFCEC  ((unsigned long)0xFFFFFFFF)  /* Number of frames received with CRC error. */

/* Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register */
#define ETH_MMCRFAECR_RFAEC  ((unsigned long)0xFFFFFFFF)  /* Number of frames received with alignment (dribble) error */

/* Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register */
#define ETH_MMCRGUFCR_RGUFC  ((unsigned long)0xFFFFFFFF)  /* Number of good unicast frames received. */

/******************************************************************************/
/*               Ethernet PTP Registers bits definition                       */
/******************************************************************************/

/* Bit definition for Ethernet PTP Time Stamp Contol Register */
#define ETH_PTPTSCR_TSARU    ((unsigned long)0x00000020)  /* Addend register update */
#define ETH_PTPTSCR_TSITE    ((unsigned long)0x00000010)  /* Time stamp interrupt trigger enable */
#define ETH_PTPTSCR_TSSTU    ((unsigned long)0x00000008)  /* Time stamp update */
#define ETH_PTPTSCR_TSSTI    ((unsigned long)0x00000004)  /* Time stamp initialize */
#define ETH_PTPTSCR_TSFCU    ((unsigned long)0x00000002)  /* Time stamp fine or coarse update */
#define ETH_PTPTSCR_TSE      ((unsigned long)0x00000001)  /* Time stamp enable */

/* Bit definition for Ethernet PTP Sub-Second Increment Register */
#define ETH_PTPSSIR_STSSI    ((unsigned long)0x000000FF)  /* System time Sub-second increment value */

/* Bit definition for Ethernet PTP Time Stamp High Register */
#define ETH_PTPTSHR_STS      ((unsigned long)0xFFFFFFFF)  /* System Time second */

/* Bit definition for Ethernet PTP Time Stamp Low Register */
#define ETH_PTPTSLR_STPNS    ((unsigned long)0x80000000)  /* System Time Positive or negative time */
#define ETH_PTPTSLR_STSS     ((unsigned long)0x7FFFFFFF)  /* System Time sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp High Update Register */
#define ETH_PTPTSHUR_TSUS    ((unsigned long)0xFFFFFFFF)  /* Time stamp update seconds */

/* Bit definition for Ethernet PTP Time Stamp Low Update Register */
#define ETH_PTPTSLUR_TSUPNS  ((unsigned long)0x80000000)  /* Time stamp update Positive or negative time */
#define ETH_PTPTSLUR_TSUSS   ((unsigned long)0x7FFFFFFF)  /* Time stamp update sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp Addend Register */
#define ETH_PTPTSAR_TSA      ((unsigned long)0xFFFFFFFF)  /* Time stamp addend */

/* Bit definition for Ethernet PTP Target Time High Register */
#define ETH_PTPTTHR_TTSH     ((unsigned long)0xFFFFFFFF)  /* Target time stamp high */

/* Bit definition for Ethernet PTP Target Time Low Register */
#define ETH_PTPTTLR_TTSL     ((unsigned long)0xFFFFFFFF)  /* Target time stamp low */

/******************************************************************************/
/*                 Ethernet DMA Registers bits definition                     */
/******************************************************************************/

/* Bit definition for Ethernet DMA Bus Mode Register */
#define ETH_DMABMR_AAB       ((unsigned long)0x02000000)  /* Address-Aligned beats */
#define ETH_DMABMR_FPM        ((unsigned long)0x01000000)  /* 4xPBL mode */
#define ETH_DMABMR_USP       ((unsigned long)0x00800000)  /* Use separate PBL */
#define ETH_DMABMR_RDP       ((unsigned long)0x007E0000)  /* RxDMA PBL */
  #define ETH_DMABMR_RDP_1Beat    ((unsigned long)0x00020000)  /* maximum number of beats to be transferred in one RxDMA transaction is 1 */
  #define ETH_DMABMR_RDP_2Beat    ((unsigned long)0x00040000)  /* maximum number of beats to be transferred in one RxDMA transaction is 2 */
  #define ETH_DMABMR_RDP_4Beat    ((unsigned long)0x00080000)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
  #define ETH_DMABMR_RDP_8Beat    ((unsigned long)0x00100000)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
  #define ETH_DMABMR_RDP_16Beat   ((unsigned long)0x00200000)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
  #define ETH_DMABMR_RDP_32Beat   ((unsigned long)0x00400000)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
  #define ETH_DMABMR_RDP_4xPBL_4Beat   ((unsigned long)0x01020000)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
  #define ETH_DMABMR_RDP_4xPBL_8Beat   ((unsigned long)0x01040000)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
  #define ETH_DMABMR_RDP_4xPBL_16Beat  ((unsigned long)0x01080000)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
  #define ETH_DMABMR_RDP_4xPBL_32Beat  ((unsigned long)0x01100000)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
  #define ETH_DMABMR_RDP_4xPBL_64Beat  ((unsigned long)0x01200000)  /* maximum number of beats to be transferred in one RxDMA transaction is 64 */
  #define ETH_DMABMR_RDP_4xPBL_128Beat ((unsigned long)0x01400000)  /* maximum number of beats to be transferred in one RxDMA transaction is 128 */
#define ETH_DMABMR_FB        ((unsigned long)0x00010000)  /* Fixed Burst */
#define ETH_DMABMR_RTPR      ((unsigned long)0x0000C000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_1_1     ((unsigned long)0x00000000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_2_1     ((unsigned long)0x00004000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_3_1     ((unsigned long)0x00008000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_4_1     ((unsigned long)0x0000C000)  /* Rx Tx priority ratio */
#define ETH_DMABMR_PBL    ((unsigned long)0x00003F00)  /* Programmable burst length */
  #define ETH_DMABMR_PBL_1Beat    ((unsigned long)0x00000100)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 1 */
  #define ETH_DMABMR_PBL_2Beat    ((unsigned long)0x00000200)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 2 */
  #define ETH_DMABMR_PBL_4Beat    ((unsigned long)0x00000400)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
  #define ETH_DMABMR_PBL_8Beat    ((unsigned long)0x00000800)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
  #define ETH_DMABMR_PBL_16Beat   ((unsigned long)0x00001000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
  #define ETH_DMABMR_PBL_32Beat   ((unsigned long)0x00002000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
  #define ETH_DMABMR_PBL_4xPBL_4Beat   ((unsigned long)0x01000100)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
  #define ETH_DMABMR_PBL_4xPBL_8Beat   ((unsigned long)0x01000200)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
  #define ETH_DMABMR_PBL_4xPBL_16Beat  ((unsigned long)0x01000400)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
  #define ETH_DMABMR_PBL_4xPBL_32Beat  ((unsigned long)0x01000800)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
  #define ETH_DMABMR_PBL_4xPBL_64Beat  ((unsigned long)0x01001000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 64 */
  #define ETH_DMABMR_PBL_4xPBL_128Beat ((unsigned long)0x01002000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 128 */
#define ETH_DMABMR_DSL       ((unsigned long)0x0000007C)  /* Descriptor Skip Length */
#define ETH_DMABMR_DA        ((unsigned long)0x00000002)  /* DMA arbitration scheme */
#define ETH_DMABMR_SR        ((unsigned long)0x00000001)  /* Software reset */

/* Bit definition for Ethernet DMA Transmit Poll Demand Register */
#define ETH_DMATPDR_TPD      ((unsigned long)0xFFFFFFFF)  /* Transmit poll demand */

/* Bit definition for Ethernet DMA Receive Poll Demand Register */
#define ETH_DMARPDR_RPD      ((unsigned long)0xFFFFFFFF)  /* Receive poll demand  */

/* Bit definition for Ethernet DMA Receive Descriptor List Address Register */
#define ETH_DMARDLAR_SRL     ((unsigned long)0xFFFFFFFF)  /* Start of receive list */

/* Bit definition for Ethernet DMA Transmit Descriptor List Address Register */
#define ETH_DMATDLAR_STL     ((unsigned long)0xFFFFFFFF)  /* Start of transmit list */

/* Bit definition for Ethernet DMA Status Register */
#define ETH_DMASR_TSTS       ((unsigned long)0x20000000)  /* Time-stamp trigger status */
#define ETH_DMASR_PMTS       ((unsigned long)0x10000000)  /* PMT status */
#define ETH_DMASR_MMCS       ((unsigned long)0x08000000)  /* MMC status */
#define ETH_DMASR_EBS        ((unsigned long)0x03800000)  /* Error bits status */
  /* combination with EBS[2:0] for GetFlagStatus function */
  #define ETH_DMASR_EBS_DescAccess      ((unsigned long)0x02000000)  /* Error bits 0-data buffer, 1-desc. access */
  #define ETH_DMASR_EBS_ReadTransf      ((unsigned long)0x01000000)  /* Error bits 0-write trnsf, 1-read transfr */
  #define ETH_DMASR_EBS_DataTransfTx    ((unsigned long)0x00800000)  /* Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMASR_TPS         ((unsigned long)0x00700000)  /* Transmit process state */
  #define ETH_DMASR_TPS_Stopped         ((unsigned long)0x00000000)  /* Stopped - Reset or Stop Tx Command issued  */
  #define ETH_DMASR_TPS_Fetching        ((unsigned long)0x00100000)  /* Running - fetching the Tx descriptor */
  #define ETH_DMASR_TPS_Waiting         ((unsigned long)0x00200000)  /* Running - waiting for status */
  #define ETH_DMASR_TPS_Reading         ((unsigned long)0x00300000)  /* Running - reading the data from host memory */
  #define ETH_DMASR_TPS_Suspended       ((unsigned long)0x00600000)  /* Suspended - Tx Descriptor unavailabe */
  #define ETH_DMASR_TPS_Closing         ((unsigned long)0x00700000)  /* Running - closing Rx descriptor */
#define ETH_DMASR_RPS         ((unsigned long)0x000E0000)  /* Receive process state */
  #define ETH_DMASR_RPS_Stopped         ((unsigned long)0x00000000)  /* Stopped - Reset or Stop Rx Command issued */
  #define ETH_DMASR_RPS_Fetching        ((unsigned long)0x00020000)  /* Running - fetching the Rx descriptor */
  #define ETH_DMASR_RPS_Waiting         ((unsigned long)0x00060000)  /* Running - waiting for packet */
  #define ETH_DMASR_RPS_Suspended       ((unsigned long)0x00080000)  /* Suspended - Rx Descriptor unavailable */
  #define ETH_DMASR_RPS_Closing         ((unsigned long)0x000A0000)  /* Running - closing descriptor */
  #define ETH_DMASR_RPS_Queuing         ((unsigned long)0x000E0000)  /* Running - queuing the recieve frame into host memory */
#define ETH_DMASR_NIS        ((unsigned long)0x00010000)  /* Normal interrupt summary */
#define ETH_DMASR_AIS        ((unsigned long)0x00008000)  /* Abnormal interrupt summary */
#define ETH_DMASR_ERS        ((unsigned long)0x00004000)  /* Early receive status */
#define ETH_DMASR_FBES       ((unsigned long)0x00002000)  /* Fatal bus error status */
#define ETH_DMASR_ETS        ((unsigned long)0x00000400)  /* Early transmit status */
#define ETH_DMASR_RWTS       ((unsigned long)0x00000200)  /* Receive watchdog timeout status */
#define ETH_DMASR_RPSS       ((unsigned long)0x00000100)  /* Receive process stopped status */
#define ETH_DMASR_RBUS       ((unsigned long)0x00000080)  /* Receive buffer unavailable status */
#define ETH_DMASR_RS         ((unsigned long)0x00000040)  /* Receive status */
#define ETH_DMASR_TUS        ((unsigned long)0x00000020)  /* Transmit underflow status */
#define ETH_DMASR_ROS        ((unsigned long)0x00000010)  /* Receive overflow status */
#define ETH_DMASR_TJTS       ((unsigned long)0x00000008)  /* Transmit jabber timeout status */
#define ETH_DMASR_TBUS       ((unsigned long)0x00000004)  /* Transmit buffer unavailable status */
#define ETH_DMASR_TPSS       ((unsigned long)0x00000002)  /* Transmit process stopped status */
#define ETH_DMASR_TS         ((unsigned long)0x00000001)  /* Transmit status */

/* Bit definition for Ethernet DMA Operation Mode Register */
#define ETH_DMAOMR_DTCEFD    ((unsigned long)0x04000000)  /* Disable Dropping of TCP/IP checksum error frames */
#define ETH_DMAOMR_RSF       ((unsigned long)0x02000000)  /* Receive store and forward */
#define ETH_DMAOMR_DFRF      ((unsigned long)0x01000000)  /* Disable flushing of received frames */
#define ETH_DMAOMR_TSF       ((unsigned long)0x00200000)  /* Transmit store and forward */
#define ETH_DMAOMR_FTF       ((unsigned long)0x00100000)  /* Flush transmit FIFO */
#define ETH_DMAOMR_TTC       ((unsigned long)0x0001C000)  /* Transmit threshold control */
  #define ETH_DMAOMR_TTC_64Bytes       ((unsigned long)0x00000000)  /* threshold level of the MTL Transmit FIFO is 64 Bytes */
  #define ETH_DMAOMR_TTC_128Bytes      ((unsigned long)0x00004000)  /* threshold level of the MTL Transmit FIFO is 128 Bytes */
  #define ETH_DMAOMR_TTC_192Bytes      ((unsigned long)0x00008000)  /* threshold level of the MTL Transmit FIFO is 192 Bytes */
  #define ETH_DMAOMR_TTC_256Bytes      ((unsigned long)0x0000C000)  /* threshold level of the MTL Transmit FIFO is 256 Bytes */
  #define ETH_DMAOMR_TTC_40Bytes       ((unsigned long)0x00010000)  /* threshold level of the MTL Transmit FIFO is 40 Bytes */
  #define ETH_DMAOMR_TTC_32Bytes       ((unsigned long)0x00014000)  /* threshold level of the MTL Transmit FIFO is 32 Bytes */
  #define ETH_DMAOMR_TTC_24Bytes       ((unsigned long)0x00018000)  /* threshold level of the MTL Transmit FIFO is 24 Bytes */
  #define ETH_DMAOMR_TTC_16Bytes       ((unsigned long)0x0001C000)  /* threshold level of the MTL Transmit FIFO is 16 Bytes */
#define ETH_DMAOMR_ST        ((unsigned long)0x00002000)  /* Start/stop transmission command */
#define ETH_DMAOMR_FEF       ((unsigned long)0x00000080)  /* Forward error frames */
#define ETH_DMAOMR_FUGF      ((unsigned long)0x00000040)  /* Forward undersized good frames */
#define ETH_DMAOMR_RTC       ((unsigned long)0x00000018)  /* receive threshold control */
  #define ETH_DMAOMR_RTC_64Bytes       ((unsigned long)0x00000000)  /* threshold level of the MTL Receive FIFO is 64 Bytes */
  #define ETH_DMAOMR_RTC_32Bytes       ((unsigned long)0x00000008)  /* threshold level of the MTL Receive FIFO is 32 Bytes */
  #define ETH_DMAOMR_RTC_96Bytes       ((unsigned long)0x00000010)  /* threshold level of the MTL Receive FIFO is 96 Bytes */
  #define ETH_DMAOMR_RTC_128Bytes      ((unsigned long)0x00000018)  /* threshold level of the MTL Receive FIFO is 128 Bytes */
#define ETH_DMAOMR_OSF       ((unsigned long)0x00000004)  /* operate on second frame */
#define ETH_DMAOMR_SR        ((unsigned long)0x00000002)  /* Start/stop receive */

/* Bit definition for Ethernet DMA Interrupt Enable Register */
#define ETH_DMAIER_NISE      ((unsigned long)0x00010000)  /* Normal interrupt summary enable */
#define ETH_DMAIER_AISE      ((unsigned long)0x00008000)  /* Abnormal interrupt summary enable */
#define ETH_DMAIER_ERIE      ((unsigned long)0x00004000)  /* Early receive interrupt enable */
#define ETH_DMAIER_FBEIE     ((unsigned long)0x00002000)  /* Fatal bus error interrupt enable */
#define ETH_DMAIER_ETIE      ((unsigned long)0x00000400)  /* Early transmit interrupt enable */
#define ETH_DMAIER_RWTIE     ((unsigned long)0x00000200)  /* Receive watchdog timeout interrupt enable */
#define ETH_DMAIER_RPSIE     ((unsigned long)0x00000100)  /* Receive process stopped interrupt enable */
#define ETH_DMAIER_RBUIE     ((unsigned long)0x00000080)  /* Receive buffer unavailable interrupt enable */
#define ETH_DMAIER_RIE       ((unsigned long)0x00000040)  /* Receive interrupt enable */
#define ETH_DMAIER_TUIE      ((unsigned long)0x00000020)  /* Transmit Underflow interrupt enable */
#define ETH_DMAIER_ROIE      ((unsigned long)0x00000010)  /* Receive Overflow interrupt enable */
#define ETH_DMAIER_TJTIE     ((unsigned long)0x00000008)  /* Transmit jabber timeout interrupt enable */
#define ETH_DMAIER_TBUIE     ((unsigned long)0x00000004)  /* Transmit buffer unavailable interrupt enable */
#define ETH_DMAIER_TPSIE     ((unsigned long)0x00000002)  /* Transmit process stopped interrupt enable */
#define ETH_DMAIER_TIE       ((unsigned long)0x00000001)  /* Transmit interrupt enable */

/* Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register */
#define ETH_DMAMFBOCR_OFOC   ((unsigned long)0x10000000)  /* Overflow bit for FIFO overflow counter */
#define ETH_DMAMFBOCR_MFA    ((unsigned long)0x0FFE0000)  /* Number of frames missed by the application */
#define ETH_DMAMFBOCR_OMFC   ((unsigned long)0x00010000)  /* Overflow bit for missed frame counter */
#define ETH_DMAMFBOCR_MFC    ((unsigned long)0x0000FFFF)  /* Number of frames missed by the controller */

/* Bit definition for Ethernet DMA Current Host Transmit Descriptor Register */
#define ETH_DMACHTDR_HTDAP   ((unsigned long)0xFFFFFFFF)  /* Host transmit descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Descriptor Register */
#define ETH_DMACHRDR_HRDAP   ((unsigned long)0xFFFFFFFF)  /* Host receive descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register */
#define ETH_DMACHTBAR_HTBAP  ((unsigned long)0xFFFFFFFF)  /* Host transmit buffer address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Buffer Address Register */
#define ETH_DMACHRBAR_HRBAP  ((unsigned long)0xFFFFFFFF)  /* Host receive buffer address pointer */
#endif /* STM32F10X_CL */

/**
  * @}
  */

 /**
  * @}
  */

#ifdef USE_STDPERIPH_DRIVER
  #include "stm32f10x_conf.h"
#endif

/** @addtogroup Exported_macro
  * @{
  */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_H */

/**
  * @}
  */

  /**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
