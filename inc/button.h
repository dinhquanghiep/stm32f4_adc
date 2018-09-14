/*
 * Button.h
 *
 *  Created on: May 15, 2018
 *      Author: dinhq
 */

#ifndef LIBRARIES_BUTTON_H_
#define LIBRARIES_BUTTON_H_

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>

typedef enum {RELEASE = Bit_RESET, WAIT_DEBOUNCE = 10, PRESS = Bit_SET} Button_state_t;   //

typedef struct {
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin;
  uint8_t bt_state;
  uint32_t bt_press_time;
} Button_t;

Button_state_t getBTStatus(Button_t *buton, uint16_t delay_time_ms);
Button_state_t getBTStatus_old(Button_t *buton, uint16_t delay_time_ms);
#endif /* LIBRARIES_BUTTON_H_ */
