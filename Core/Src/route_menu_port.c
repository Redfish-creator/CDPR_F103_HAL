#include "route_menu_port.h"

static uint8_t s_oled_menu_initialized = 0U;

void OLED_Menu_Init(void)
{
    if (s_oled_menu_initialized == 0U)
    {
        OLED_Init();
        s_oled_menu_initialized = 1U;
    }
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

void OLED_Menu_Service(void)
{
    OLED_Service();
}
