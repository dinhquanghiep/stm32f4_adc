/**
  ******************************************************************************
  * @file    main.c
  * @author  hiepdq
  * @version xxx
  * @date    06.09.2018
  * @brief   This file provides main firmware functions for MCU 
  *          Try to using all peripheral and standard coding style           
 ===============================================================================      
                       ##### How to use this driver #####
 ===============================================================================
  
  ******************************************************************************
  * @attention
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rng.h>
#include <misc.h>
#include "button.h"

#if defined (__GNUC__)
#include <malloc.h>
#elif defined (__ICCARM__)

#endif

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Config LED pin */
#define LED_GREEN   GPIO_Pin_12
#define LED_ORANGE  GPIO_Pin_13
#define LED_RED     GPIO_Pin_14
#define LED_BLUE    GPIO_Pin_15

/* Config for ADC pin */
#define ADC1_CHANNEL3 GPIO_Pin_3
#define ADC1_CHANNEL4 GPIO_Pin_4
#define ADC2_CHANNEL5 GPIO_Pin_5

/* Define PII value base on current value from system */

/* Private typedef -----------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
uint32_t curr_time = 0;

/* Private variables ---------------------------------------------------------*/
// static volatile const uint8_t
static volatile uint16_t myADC1_value[40] = {0};
static volatile uint16_t myADC2_value[20] = {0};
static float ADC1_IN3_avg = 0;
static float ADC1_IN4_avg = 0;
static float ADC2_avg = 0;
static Button_t g_user_button;
static uint32_t random_num = 0;

/* Private function prototypes -----------------------------------------------*/
static void rcc_config(void);
static void delay(uint32_t ms);
static void increase_curr_time(void);
static void gpio_config(void);
static void adc_common_config(void);
static void adc1_config(void);
static void adc2_config(void);
static void dma_config(void);
static void nvic_config(void);
static void rng_config(void);
/* Public functions ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @brief  Get the systick timer clock
  *         abcxyz.
  * @param  xxx1: 
  *         xxx1abc.
  * @param  xxx2: 
  *         xxx2abc.
  * @param  xxx3:
  *         xxx3abc.
  * 
  * @Note   yyy1:
  *         yyy1abc.
  * @Note   yyy2:
  *         yyy2abc.
  * @Note   yyy3: .
  * 
  * @retval None
  */

/** @brief  Config the parameters for RCC
  * @param  None
  * 
  * @retval None
  */
static void rcc_config(void) {
  /* Config for Clock use HSE */
  /* 168Mhz, PLL*/
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);

  const uint16_t MAX_TIMEOUT = 0xFFFF;
  uint16_t temp = MAX_TIMEOUT;
  while (RCC_WaitForHSEStartUp() == ERROR) {
    --temp;
  }
  /* Config for Sysclk (base on HSE or HSI) */
  if (temp != 0) {
    RCC_PLLConfig(RCC_PLLSource_HSE, 4, 168, 2, 4);
    RCC_PLLCmd(ENABLE);
    RCC_ClockSecuritySystemCmd(DISABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    RCC_HSICmd(DISABLE);
    /* Print error cannot startup HSE */
  } else {
    RCC_PLLConfig(RCC_PLLSource_HSI, 10, 168, 2, 4);
    RCC_PLLCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  }

  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div4);
  RCC_PCLK2Config(RCC_HCLK_Div2);

  /* Config clock for system timer */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
  SystemCoreClockUpdate();
}

/** @brief  Delay miniSecond.
  * @param  ms: the time to delay, from 1 to 65535
  * 
  * @Note   when call this Func., the CPU cannot do others things
  * 
  * @retval None
  */
static void delay(uint32_t ms) {
  uint32_t __time = (uint32_t)56000 * ms;
  while (--__time) {
    __ASM("nop");
  }
}

/** @brief  To increase global variable curr_time every 1ms
  * @param  None
  * 
  * @retval None
  */
static void increase_curr_time(void) {
  delay(1);
  ++curr_time;
}

/** @brief  config the GPIO for LED, Button,...
  * @param  None
  * 
  * @retval None
  */
static void gpio_config(void) {
  /* Config clock for GPIO */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Config pin for LED */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = LED_BLUE | LED_GREEN | LED_ORANGE | LED_RED;

  /* không cần cấu hình Pull up/down vì mặc định luôn được kết nối */
  // GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Config pin for User Button */
  g_user_button.GPIO_Pin = GPIO_Pin_0;
  g_user_button.GPIOx = GPIOA;
  g_user_button.bt_state = RELEASE;

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Pin = g_user_button.GPIO_Pin;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Config pin for ADC */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Pin = ADC1_CHANNEL3 | ADC1_CHANNEL4 | ADC2_CHANNEL5;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_RED, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_GREEN, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_ORANGE, Bit_RESET);
}

/** @brief  Config the parameters for ADC common
  * @param  None
  * 
  * @retval None
  */
static void adc_common_config(void) {
  ADC_DeInit();
  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
  /** Xung này là cấp cho ADC để chuyển đổi, được lấy từ APB2
    * Xung ABP2 hiện tại là 84Mhz, theo DS, với VAAD = 2.4 - 3.6V thì
    * xung cho ADC Fmax = 36Mhz, do đó hiệu quả nhất là Div4
    */
  ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStruct);
}

/** @brief  Config the parameters for ADC1 channel 3 and 4, use DMA
  * @param  None
  * 
  * @retval None1/
  */
static void adc1_config(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* config the port A, PA3 and PA4 as Analog mode */

  ADC_InitTypeDef ADC_InitStruct;
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStruct.ADC_ScanConvMode = ENABLE;
  ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  /* 2 channels 3 and 4 */
  ADC_InitStruct.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStruct);
  /* Rank 1 for channel 3 and Rank 2 for channel 4 */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_15Cycles);

  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);

  ADC_Cmd(ADC1, ENABLE);
}

/** @brief  Config the parameters for ADC2 channel 5 and use DMA
  * @param  None
  * 
  * @retval None1/
  */
static void adc2_config(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  /* config the port A, PA3 and PA4 as Analog mode */

  ADC_InitTypeDef ADC_InitStruct;
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStruct.ADC_ScanConvMode = DISABLE;
  ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  /* 2 channels 3 and 4 */
  ADC_InitStruct.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStruct);
  /* Rank 1 for channel 3 and Rank 2 for channel 4 */
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_15Cycles);

  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  ADC_DMACmd(ADC2, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
}

/** @brief  Config DMA for ADC1
  * @param  None
  * 
  * @retval None
  */
static void dma_config(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  /* DeInit before Init */
  DMA_DeInit(DMA2_Stream4);
  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_Channel = DMA_Channel_0;
  DMA_InitStruct.DMA_PeripheralBaseAddr = ADC1_BASE + 0x004C;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)myADC1_value;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = 40;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  // DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStruct);
  // DMA_PeriphIncOffsetSizeConfig(DMA2_Stream4, DMA_PINCOS_WordAligned);
  DMA_Cmd(DMA2_Stream4, ENABLE);

  DMA_DeInit(DMA2_Stream3);
  DMA_InitStruct.DMA_Channel = DMA_Channel_1;
  DMA_InitStruct.DMA_PeripheralBaseAddr = ADC2_BASE + 0x004C;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)myADC2_value;
  // DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = 20;
  // DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  // DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  // DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  // DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  // DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  // DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
  // DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  // DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  // DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  // DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream3, &DMA_InitStruct);
  DMA_Cmd(DMA2_Stream3, ENABLE);

  DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
}

/** @brief  Config NVIC for DMA
  * @param  None
  * 
  * @retval None
  */
static void nvic_config(void) {
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

/** @brief  Config Random number generator
  * @param  None
  * 
  * @retval None
  */
static void rng_config(void) {
  RNG_DeInit();
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
  RNG_Cmd(ENABLE);
}

/* Main source ---------------------------------------------------------------*/
int main(void) {
  rcc_config();
  gpio_config();
  dma_config();
  adc_common_config();
  adc1_config();
  adc2_config();
  nvic_config();
  rng_config();
  ADC_SoftwareStartConv(ADC1);
  ADC_SoftwareStartConv(ADC2);
  delay(10);
  while (1) {
    /* Do nothing here */
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) != RESET) {
      random_num = RNG_GetRandomNumber();
    }
    if (random_num > 0x89545236) {
      GPIO_WriteBit(GPIOD, LED_ORANGE, Bit_SET);
    } else if (random_num < 0x12545236) {
      GPIO_WriteBit(GPIOD, LED_ORANGE, Bit_RESET);
    }
    delay(1000);
  }
  return 0;
}

/**
  * @brief  This function handles EXTI0_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void) {
  if (EXTI_GetFlagStatus(EXTI_Line7) != RESET) {
      GPIO_ToggleBits(GPIOD, LED_RED);
    EXTI_ClearFlag(EXTI_Line7);
  }
}

/**
  * @brief  This function handles DMA2_Stream3_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void) {
  DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);

  uint32_t tmp_value = 0;
  for (uint8_t tmp = 0; tmp < 40; tmp += 2) {
    tmp_value += myADC1_value[tmp];
  }
  ADC1_IN3_avg = (float)tmp_value / 20;

  tmp_value = 0;
  for (uint8_t tmp = 1; tmp < 40; tmp += 2) {
    tmp_value += myADC1_value[tmp];
  }
  ADC1_IN4_avg = (float)tmp_value / 20;

  if (ADC1_IN3_avg > 2000) {
    GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);
  } else {
    GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);
  }
}

/**
  * @brief  This function handles DMA2_Stream4_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream4_IRQHandler(void) {
  DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
  uint32_t tmp_value = 0;
  for (uint8_t tmp = 0; tmp < 20; tmp++) {
    tmp_value += myADC2_value[tmp];
  }
}

