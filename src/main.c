#include <main.h>

/* Config LED pin */
#define LED_GREEN   GPIO_Pin_12
#define LED_ORANGE  GPIO_Pin_13
#define LED_RED     GPIO_Pin_14
#define LED_BLUE    GPIO_Pin_15

uint32_t GetSystickFreq() {
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  if ((SysTick->CTRL & SysTick_CLKSource_HCLK) == SysTick_CLKSource_HCLK) {
    return RCC_Clocks.HCLK_Frequency;
  } else {
    return RCC_Clocks.HCLK_Frequency/8;
  }
}

int main(void) {
  /* Config for Clock use HSE */
  /* 168Mhz, PLL*/
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  {
    uint16_t temp = 0xFFFF;
    while (RCC_WaitForHSEStartUp() == ERROR) {
      --temp;
    }
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
  }
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div4);
  RCC_PCLK2Config(RCC_HCLK_Div2);
  /* Config clock for system timer */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
  SystemCoreClockUpdate();

  return 0;
}