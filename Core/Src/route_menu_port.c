#include "route_menu_port.h"

void OLED_Menu_Init(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_Refresh();
}

void OLED_Menu_Clear(void)
{
    OLED_Clear();
}

void OLED_Menu_ShowLine(uint8_t line, const char *str)
{
    OLED_ShowLine(line, str);
}

void OLED_Menu_Refresh(void)
{
    OLED_Refresh();
}
