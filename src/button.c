/*
 * button.cpp
 *
 *  Created on: May 15, 2018
 *      Author: dinhq
 */

#include <button.h>

extern uint32_t curr_time;

Button_state_t getBTStatus(Button_t *button, uint16_t delay_time_ms) {

  /*
   * Giai phap duoi nay ro rang de phan tich nhung lai ton dung luong code hon
   * Toc do chay cung cham hon ban old nhung khoang cham nho co the bo qua
   * Giai phap nay giai quyet duoc van de delay(20) o truong hop doi debounce
  */
  switch (button->bt_state) {
    case RELEASE:
      if (GPIO_ReadInputDataBit(button->GPIOx, button->GPIO_Pin) == Bit_SET) {
        button->bt_state = WAIT_DEBOUNCE;
        button->bt_press_time = curr_time;
      }
      return RELEASE;
      break;
    case WAIT_DEBOUNCE:
      if (curr_time - button->bt_press_time > 20) {
        if (GPIO_ReadInputDataBit(button->GPIOx, button->GPIO_Pin) == Bit_SET) {
          button->bt_state = PRESS;
          button->bt_press_time = curr_time;
          return PRESS;
        } else {
          button->bt_state = RELEASE;
          return RELEASE;
        }
      } else {
        return RELEASE;
      }
      break;
    case PRESS:
      if (GPIO_ReadInputDataBit(button->GPIOx, button->GPIO_Pin) == Bit_SET) {
        if (curr_time - button->bt_press_time > delay_time_ms) {
          button->bt_press_time = curr_time;
          return PRESS;
        } else {
          return RELEASE;
        }
      } else {
        button->bt_state = RELEASE;
        return RELEASE;
      }
  }
}
#if 0
Button_State getBTStatus_old(Button *button, uint16_t delay_time) {
  /*
   * get the Button status (press/ release)_debounce
   * Return 1 for press frist or keep press over delay_time
   * Return 0 for release
   */
  /*
   van de gap phai o day neu dung ham nay don thuan cho nhieu nut nhan
   khi do cac nut nhan se hoat dong ko chinh xac vi static trong ham nay
   Giai phap la moi nut nhan se co mot bien dem thoi gian rieng
   ==> class, hoac struct trong truong hop nay mang lai hieu qua
   */
//  static int bt_state = 0;
//  static long bt_press_time = 0;
  if (LOW == digitalRead(button->pin)) {
    if (button->bt_state) {
      if (curr_time > delay_time + button->bt_press_time) {
        button->bt_press_time = curr_time;
        return 1;
      } else {
        return 0;
      }
    } else {
      delay(20);
      if (LOW == digitalRead(button->pin)) {
        button->bt_state = 1;
        button->bt_press_time = curr_time;
        return 1;
      } else {
        return 0;
      }
    }
  } else {
    button->bt_state = 0;
    return 0;
  }
}
#endif
