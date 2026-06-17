#ifndef __ROUTE_MENU_PORT_H
#define __ROUTE_MENU_PORT_H

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include "oled.h"

// 外接四按键，低电平按下
#define KEY_EXT1_GPIO_Port   GPIOC
#define KEY_EXT1_Pin         GPIO_PIN_0   // 上

#define KEY_EXT2_GPIO_Port   GPIOC
#define KEY_EXT2_Pin         GPIO_PIN_1   // 下

#define KEY_EXT3_GPIO_Port   GPIOC
#define KEY_EXT3_Pin         GPIO_PIN_2   // 确认

#define KEY_EXT4_GPIO_Port   GPIOC
#define KEY_EXT4_Pin         GPIO_PIN_3   // 返回

void OLED_Menu_Init(void);
void OLED_Menu_Clear(void);
void OLED_Menu_ShowLine(uint8_t line, const char *str);
void OLED_Menu_Refresh(void);

#endif
