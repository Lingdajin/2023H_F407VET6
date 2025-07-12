/*
 * key.c
 *
 *  Created on: Jun 4, 2025
 *      Author: 1
 */

#include "key.h"


uint8_t key_scan() {
	uint8_t key_val_WK = HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin);
	uint8_t key_val_KEY0 = HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
	uint8_t key_val_KEY1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
	uint8_t key_return = 0;


	switch(key_state) {
	case KEY_IDLE:
		if(key_val_WK == KEY_PRESSED_WK || key_val_KEY0 == KEY_PRESSED_KEY || key_val_KEY1 == KEY_PRESSED_KEY) {
			key_state = KEY_DEBOUNCE;
			debounce_count = 0;
		}
		break;
	case KEY_DEBOUNCE:
		if(key_val_WK == KEY_PRESSED_WK || key_val_KEY0 == KEY_PRESSED_KEY || key_val_KEY1 == KEY_PRESSED_KEY) {
			if(debounce_count >= DEBOUNCE_TIME) {
				key_state = KEY_PRESSED_OK;
				if(key_val_WK == KEY_PRESSED_WK) {
					// key_event = WKUP_EVENT;	//WK按键按下
					return WKUP_EVENT;
				}
				else if(key_val_KEY0 == KEY_PRESSED_KEY) {
					// key_event = KEY0_EVENT;	//KEY0按键按下
					return KEY0_EVENT;
				}
				else if(key_val_KEY1 == KEY_PRESSED_KEY) {
					// key_event = KEY1_EVENT;	//KEY1按键按下
					return KEY1_EVENT;
				}

				// HAL_TIM_Base_Stop_IT(&htim5);
			}
			else {
				//HAL_TIM_Base_Start_IT(&htim5);
			}
		}
		else {
			//抖动或松开
			key_state = KEY_IDLE;
		}
		break;
	case KEY_PRESSED_OK:
		if(key_val_WK == KEY_RELEASE_WK && key_val_KEY0 == KEY_RELEASE_KEY && key_val_KEY1 && KEY_RELEASE_KEY) {
			key_state = KEY_IDLE;
		}
		break;
	default:
		key_state = KEY_IDLE;
		break;
	}
	// if(key_event) {
	// 	//有按键按下事件
	// 	key_return = key_event;	//返回按键事件
	// 	key_event = 0;
	// 	return key_return;
	// }
	//没有按键按下
	return 0;
}
