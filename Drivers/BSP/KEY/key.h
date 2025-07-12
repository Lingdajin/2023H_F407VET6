/*
 * key.h
 *
 *  Created on: Jun 4, 2025
 *      Author: 1
 */

#ifndef BSP_KEY_KEY_H_
#define BSP_KEY_KEY_H_

#include "main.h"
#include "tim.h"

#define DEBOUNCE_TIME	10	//消抖延时ms
#define KEY_PRESSED_WK	1	//WK按键按下
#define KEY_RELEASE_WK 0	//WK按键释放
#define KEY_PRESSED_KEY	0	//KEY0、KEY1按键按下
#define KEY_RELEASE_KEY	1	//KEY0、KEY1按键按下

#define KEY_NONE	0	//没有按键按下
#define WKUP_EVENT	1	//WK_UP按键事件
#define KEY0_EVENT	2	//KEY0按键事件
#define KEY1_EVENT	3	//KEY1按键事件


typedef enum {
	KEY_IDLE,
	KEY_DEBOUNCE,
	KEY_PRESSED_OK
}KeyState_t;

static KeyState_t key_state = KEY_IDLE;
extern uint8_t debounce_count;
static uint8_t key_event = 0;	//置1表示检测到一次按下事件

uint8_t key_scan();

#endif /* BSP_KEY_KEY_H_ */
