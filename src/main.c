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
#include <misc.h>
#include <stm32f4xx_gpio.h>
#include "button.h"
#include <stm32f4xx_dma.h>
#include <stm32f4xx_adc.h>
#include <string.h>
#include <stdlib.h>
#include <arm_math.h>
#include <math.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>

#if defined (__GNUC__)
#include <malloc.h>
#elif defined (__ICCARM__)

#endif

/* Private macro -------------------------------------------------------------*/
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define SYSTEMCALLBACK(fnCallback) void (*fnCallback) (uint32_t)

/* Private define ------------------------------------------------------------*/
/* Config LED pin */
#define LED_GREEN   GPIO_Pin_12
#define LED_ORANGE  GPIO_Pin_13
#define LED_RED     GPIO_Pin_14
#define LED_BLUE    GPIO_Pin_15

/* Config for ADC pin */
#define ADC_TEST    GPIO_Pin_3
#define ADC_TEST1   GPIO_Pin_4

/* Define PII value base on current value from system */

/* Private typedef -----------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
uint32_t curr_time = 0;

/* Private variables ---------------------------------------------------------*/
// static volatile const uint8_t
static volatile uint16_t myADC_value[80] = {0};
static uint32_t ADC_avg = 0;
static volatile uint32_t g_test32 = 0;
static volatile uint32_t kq = 0;
static uint32_t g_mang_so_nguyen[100] = {50};
static char g_chuoi[100] = "";
static int32_t g_test_sign = 0;
static char *gp_my_name = "Dinh Quang Hiep";
static uint32_t **gpp_arr_so;
static uint32_t *gp_arr_so;
static uint32_t *g_arr_so = (uint32_t []){10, 20, 30, 40, 50};
static Button_t g_user_button;
static volatile uint32_t ngatngoaiA = 0;
static volatile uint32_t ngatngoaiD = 0;

/* Private function prototypes -----------------------------------------------*/
static void config_rcc(void);
static uint32_t get_systick_freq(void);
static void delay(uint32_t ms);
static void increase_curr_time(void);
static void config_gpio(void);
static void config_adc(void);
static void config_dma(void);
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
uint32_t get_systick_freq(void) {
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  if ((SysTick->CTRL & SysTick_CLKSource_HCLK) == SysTick_CLKSource_HCLK) {
    return RCC_Clocks.HCLK_Frequency;
  } else {
    return RCC_Clocks.HCLK_Frequency / 8;
  }
}

/** @brief  Config the parameter for RCC
  * @param  None
  * 
  * @retval None
  */
static void config_rcc(void) {
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
    __ASM ("nop");
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
static void config_gpio(void) {
  /* Config clock for GPIO */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

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
  GPIO_InitStruct.GPIO_Pin = ADC_TEST | ADC_TEST1;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_RED, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_GREEN, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_ORANGE, Bit_RESET);
}

/** @brief  Config ADC for ADC1 channel 3 and use DMA
  * @param  None
  * 
  * @retval None
  */
static void config_adc(void) {
  
  /** config the port A, PA3
    * Refer to GPIO config */
  
  /* Config clock for ADC perripheral */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Phần chung cho cả 3 bộ ADC */
  ADC_DeInit();
  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  /* Chế độ độc lập là mạnh thằng nào thằng đó chuyển đổi */
  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
  /** Xung này là cấp cho ADC để chuyển đổi, được lấy từ APB2
    * Xung ABP2 hiện tại là 84Mhz, theo DS, với VAAD = 2.4 - 3.6V thì
    * xung cho ADC Fmax = 36Mhz, do đó hiệu quả nhất là Div4
    */
  ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  /** Khoảng thời gian giữa 2 lần lấy mẫu, phần này chỉ ảnh hưởng
    * nếu thực hiện lấy mẫu liên tục trên 1 kênh
    * */
  ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStruct);

  /* phần riêng cho từng bộ ADC */
  /* Config for ADC PA3, continue and DMA */
  ADC_InitTypeDef ADC_InitStruct;
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  /* Chế độ này sẽ dùng khi có nhiều hơn 1 kênh cần chuyển đổi trên 1 bộ ADC*/
  ADC_InitStruct.ADC_ScanConvMode = ENABLE;
  /* Chuyển đôi liên tục, quay vòng */
  ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  /* 2 kênh */
  ADC_InitStruct.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStruct);
  /* Check Rank, cycle time */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_15Cycles);
  /* can tin hieu ben duoi de DAM va ADC hoat dong lien tuc*/
  /* O che do nao thi ADC van hoat dong binh thuong khong bi anh huong */
  /* nhung neu tat di thi DMA khong hoat dong */
  /* Mot khi da bat thi chỉ chuyển đổi khi nhận tín hiệu lấy xong dữ liệu của DMA*/
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  
  // ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
}

/** @brief  Config DMA for ADC1
  * @param  None
  * 
  * @retval None
  */
static void config_nvic(void) {

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn; 
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

/** @brief  Config DMA for ADC1
  * @param  None
  * 
  * @retval None
  */
static void config_dma(void) {
  /* Config the RCC-clock for DMA */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  /* DeInit before Init */
  DMA_DeInit(DMA2_Stream4);
  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_Channel = DMA_Channel_0;
  DMA_InitStruct.DMA_PeripheralBaseAddr = ADC1_BASE + 0x004C;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)myADC_value;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = 80;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  // Che do normal chi hoat dong 1 lan, muon lien tuc can dung che do vong
  // DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  // DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStruct);
  // DMA_PeriphIncOffsetSizeConfig(DMA2_Stream4, DMA_PINCOS_WordAligned);
  DMA_Cmd(DMA2_Stream4, ENABLE);
  // DMA_StructInit(&DMA_InitStruct);
}

/* Main source ---------------------------------------------------------------*/
int main(void) {
  
  config_rcc();
  config_gpio();
  config_dma();
  config_adc();
  // config_nvic();
  ADC_SoftwareStartConv(ADC1);
  delay(10);
  while (1) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
    delay(200);
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);
    delay(200);
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);
    delay(200);
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
    delay(200);
    kq = 1 / (g_test32 - kq);
    if (kq != 0) {
      while (1) { /* infinity loop*/ };
    }
  }
  // uint32_t 
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
  * @brief  This function handles ADC_IQRHandler interrupt request.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void) {
  g_arr_so++;
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}