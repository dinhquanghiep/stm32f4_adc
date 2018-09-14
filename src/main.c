/**
  ******************************************************************************
  * @file    main.c
  * @author  hiepdq
  * @version xxx
  * @date    14.09.2018
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
#include <stm32f4xx_adc.h>
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
static float Voltage = 0.0;
static Button_t g_user_button;

/* Private function prototypes -----------------------------------------------*/
static void rcc_config(void);
static void delay(uint32_t ms);
static void increase_curr_time(void);
static void gpio_config(void);
static void adc_common_config(void);
static void adc1_config(void);
/* Public functions ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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
  GPIO_InitStruct.GPIO_Pin = ADC_TEST | ADC_TEST1;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_RED, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_GREEN, Bit_RESET);
  GPIO_WriteBit(GPIOD, LED_ORANGE, Bit_RESET);
}

/** @brief  Config ADC common parameters for all ADCx
  * @param  None
  * 
  * @retval None
  */
static void adc_common_config(void) {
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
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

/** @brief  Config the parameters for ADC1
  *         2 Channels IN3 and IN4
  * @param  None
  * 
  * @retval None
  */
static void adc1_config(void) {
  
  /** config the port A, PA3 and PA4 analog mode
    * Refer to GPIO config */
  
  ADC_InitTypeDef ADC_InitStruct;
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStruct.ADC_ScanConvMode = DISABLE;
  ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStruct);
  /** Check Rank này là dùng khi dùng nhiều kênh trên 1 bộ ADC
    * đặt vào Rank để chạy luôn phiên
   */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
  ADC_Cmd(ADC1, ENABLE);
}

/* Main source ---------------------------------------------------------------*/
int main(void) {
  
  rcc_config();
  gpio_config();
  adc_common_config();
  adc1_config();
  delay(10);
  while (1) {
    for (uint8_t tmp = 0; tmp < 32; ++tmp) {
      ADC_SoftwareStartConv(ADC1);
      while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        /* waiting for ADC convert */
      }
      ADC_avg += ADC_GetConversionValue(ADC1);
      ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    }
    Voltage = 3.3 * (ADC_avg >> 5) / 4095;
    ADC_avg = 0;
    if (Voltage > 1.5) {
      GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);
    } else {
      GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);
    }
  }
  return 0;
}
