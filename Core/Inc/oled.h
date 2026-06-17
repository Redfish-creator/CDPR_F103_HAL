#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>

#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8

// HAL I2C 设备地址传 8bit 地址
#define OLED_I2C_ADDR   0x78   // 0x3C << 1

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);

void OLED_ShowChar(uint8_t x, uint8_t page, char ch);
void OLED_ShowString(uint8_t x, uint8_t page, const char *str);
void OLED_ShowLine(uint8_t line, const char *str);

#endif
