#include "route_menu.h"
#include "route_menu_port.h"
#include <stdio.h>
#include <string.h>

typedef enum
{
    MENU_STATE_EDIT = 0,
    MENU_STATE_READY
} MenuState_t;

typedef struct
{
    uint8_t stable_level;
    uint8_t last_sample;
    uint8_t debounce_cnt;
    uint8_t pressed_event;
} KeyFilter_t;

static RouteOrder_t g_route;
static MenuState_t g_menu_state = MENU_STATE_EDIT;

/* 当前正在编辑第几步：0~4 */
static uint8_t g_edit_step = 0;
/* 当前候选点：0~4 */
static uint8_t g_cursor = 0;

static uint8_t g_need_refresh = 1;

static KeyFilter_t g_key_up;
static KeyFilter_t g_key_down;
static KeyFilter_t g_key_ok;
static KeyFilter_t g_key_back;

static const char *g_point_name[ROUTE_POINT_NUM] =
{
    "P1", "P2", "P3", "P4", "P5"
};

/* ========== 按键底层 ========== */

static uint8_t Key_ReadRaw(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0; // 低电平按下
}

static void Key_Update(KeyFilter_t *kf, uint8_t raw_pressed)
{
    if (raw_pressed == kf->last_sample)
    {
        if (kf->debounce_cnt < 3)
        {
            kf->debounce_cnt++;
        }
    }
    else
    {
        kf->debounce_cnt = 0;
        kf->last_sample = raw_pressed;
    }

    if (kf->debounce_cnt >= 2)
    {
        if (kf->stable_level != raw_pressed)
        {
            kf->stable_level = raw_pressed;
            if (raw_pressed == 1)
            {
                kf->pressed_event = 1;
            }
        }
    }
}

static uint8_t Key_GetPressedEvent(KeyFilter_t *kf)
{
    if (kf->pressed_event)
    {
        kf->pressed_event = 0;
        return 1;
    }
    return 0;
}

/* ========== 路由数据逻辑 ========== */

static uint8_t Step_IsFilled(uint8_t step)
{
    if (step >= ROUTE_POINT_NUM)
    {
        return 0;
    }
    return (g_route.order[step] != 0xFF) ? 1 : 0;
}

static void Update_Count_And_Ready(void)
{
    uint8_t i;
    g_route.count = 0;

    for (i = 0; i < ROUTE_POINT_NUM; i++)
    {
        if (g_route.order[i] != 0xFF)
        {
            g_route.count++;
        }
    }

    if (g_route.count >= ROUTE_POINT_NUM)
    {
        g_route.ready = 1;
        g_menu_state = MENU_STATE_READY;
    }
    else
    {
        g_route.ready = 0;
        g_menu_state = MENU_STATE_EDIT;
    }
}

static uint8_t Point_UsedBeforeStep(uint8_t point, uint8_t step_exclusive)
{
    uint8_t i;
    for (i = 0; i < step_exclusive; i++)
    {
        if (g_route.order[i] == point)
        {
            return 1;
        }
    }
    return 0;
}

static uint8_t Find_Next_Selectable(uint8_t cur, uint8_t step)
{
    uint8_t i;
    for (i = 1; i <= ROUTE_POINT_NUM; i++)
    {
        uint8_t idx = (cur + i) % ROUTE_POINT_NUM;
        if (!Point_UsedBeforeStep(idx, step))
        {
            return idx;
        }
    }
    return cur;
}

static uint8_t Find_Prev_Selectable(uint8_t cur, uint8_t step)
{
    uint8_t i;
    for (i = 1; i <= ROUTE_POINT_NUM; i++)
    {
        uint8_t idx = (cur + ROUTE_POINT_NUM - i) % ROUTE_POINT_NUM;
        if (!Point_UsedBeforeStep(idx, step))
        {
            return idx;
        }
    }
    return cur;
}

/*
 * 清空从 step 开始到最后的所有步骤
 * 用于“返回重选并覆盖后续内容”
 */
static void Clear_FromStep(uint8_t step)
{
    uint8_t i;
    for (i = step; i < ROUTE_POINT_NUM; i++)
    {
        g_route.order[i] = 0xFF;
    }
    Update_Count_And_Ready();
}

static void Save_Current_Step(void)
{
    /* 只检查前面的步骤，后面的已经被清空了 */
    if (Point_UsedBeforeStep(g_cursor, g_edit_step))
    {
        return;
    }

    g_route.order[g_edit_step] = g_cursor;
    Update_Count_And_Ready();

    if (g_menu_state == MENU_STATE_EDIT)
    {
        if (g_edit_step < ROUTE_POINT_NUM - 1)
        {
            g_edit_step++;

            if (Step_IsFilled(g_edit_step))
            {
                g_cursor = g_route.order[g_edit_step];
            }
            else
            {
                g_cursor = Find_Next_Selectable(g_cursor, g_edit_step);
            }
        }
    }
}

/*
 * 返回并允许覆盖：
 * - READY 状态：回到最后一步，并清空最后一步
 * - EDIT 状态：回到上一步，并清空这一歩及其后面的所有步骤
 */
static void Step_Backward(void)
{
    if (g_route.count == 0)
    {
        g_edit_step = 0;
        g_cursor = 0;
        g_menu_state = MENU_STATE_EDIT;
        g_route.ready = 0;
        return;
    }

    if (g_menu_state == MENU_STATE_READY)
    {
        g_menu_state = MENU_STATE_EDIT;
        g_route.ready = 0;

        g_edit_step = ROUTE_POINT_NUM - 1;
        Clear_FromStep(g_edit_step);
    }
    else
    {
        if (g_edit_step > 0)
        {
            g_edit_step--;
        }
        Clear_FromStep(g_edit_step);
    }

    if (g_edit_step > 0 && Step_IsFilled(g_edit_step))
    {
        g_cursor = g_route.order[g_edit_step];
    }
    else
    {
        g_cursor = Find_Next_Selectable(0, g_edit_step);
    }
}

/* ========== OLED 显示 ========== */

static void RouteMenu_Draw(void)
{
    char line[24];

    OLED_Menu_Clear();

    OLED_Menu_ShowLine(0, "SET 5-POINT ORDER");

    if (g_menu_state == MENU_STATE_EDIT)
    {
        snprintf(line, sizeof(line), "EDIT STEP:%d/5", g_edit_step + 1);
        OLED_Menu_ShowLine(1, line);

        snprintf(line, sizeof(line), "CUR:%s", g_point_name[g_cursor]);
        OLED_Menu_ShowLine(2, line);

        OLED_Menu_ShowLine(3, "UP/DN: SELECT");
        OLED_Menu_ShowLine(4, "OK   : SAVE");
        OLED_Menu_ShowLine(5, "BACK : RE-EDIT");
    }
    else
    {
        OLED_Menu_ShowLine(1, "ORDER READY");
        OLED_Menu_ShowLine(2, "KEY0 TO START");
        OLED_Menu_ShowLine(3, "BACK TO MODIFY");
    }

    snprintf(line, sizeof(line), "1:%s 2:%s",
             Step_IsFilled(0) ? g_point_name[g_route.order[0]] : "--",
             Step_IsFilled(1) ? g_point_name[g_route.order[1]] : "--");
    OLED_Menu_ShowLine(6, line);

    snprintf(line, sizeof(line), "3:%s 4:%s 5:%s",
             Step_IsFilled(2) ? g_point_name[g_route.order[2]] : "--",
             Step_IsFilled(3) ? g_point_name[g_route.order[3]] : "--",
             Step_IsFilled(4) ? g_point_name[g_route.order[4]] : "--");
    OLED_Menu_ShowLine(7, line);

    OLED_Menu_Refresh();
}

/* ========== 对外接口 ========== */

void RouteMenu_Init(void)
{
    uint8_t i;

    memset(&g_route, 0, sizeof(g_route));
    for (i = 0; i < ROUTE_POINT_NUM; i++)
    {
        g_route.order[i] = 0xFF;
    }

    memset(&g_key_up, 0, sizeof(g_key_up));
    memset(&g_key_down, 0, sizeof(g_key_down));
    memset(&g_key_ok, 0, sizeof(g_key_ok));
    memset(&g_key_back, 0, sizeof(g_key_back));

    g_menu_state = MENU_STATE_EDIT;
    g_edit_step = 0;
    g_cursor = 0;
    g_need_refresh = 1;

    OLED_Menu_Init();
    RouteMenu_Draw();
}

void RouteMenu_Task_10ms(void)
{
    Key_Update(&g_key_up,   Key_ReadRaw(KEY_EXT1_GPIO_Port, KEY_EXT1_Pin));
    Key_Update(&g_key_down, Key_ReadRaw(KEY_EXT2_GPIO_Port, KEY_EXT2_Pin));
    Key_Update(&g_key_ok,   Key_ReadRaw(KEY_EXT3_GPIO_Port, KEY_EXT3_Pin));
    Key_Update(&g_key_back, Key_ReadRaw(KEY_EXT4_GPIO_Port, KEY_EXT4_Pin));

    if (g_menu_state == MENU_STATE_EDIT)
    {
        if (Key_GetPressedEvent(&g_key_up))
        {
            g_cursor = Find_Prev_Selectable(g_cursor, g_edit_step);
            g_need_refresh = 1;
        }

        if (Key_GetPressedEvent(&g_key_down))
        {
            g_cursor = Find_Next_Selectable(g_cursor, g_edit_step);
            g_need_refresh = 1;
        }

        if (Key_GetPressedEvent(&g_key_ok))
        {
            Save_Current_Step();
            g_need_refresh = 1;
        }

        if (Key_GetPressedEvent(&g_key_back))
        {
            Step_Backward();
            g_need_refresh = 1;
        }
    }
    else
    {
        if (Key_GetPressedEvent(&g_key_back))
        {
            Step_Backward();
            g_need_refresh = 1;
        }
    }

    if (g_need_refresh)
    {
        g_need_refresh = 0;
        RouteMenu_Draw();
    }
}

uint8_t RouteMenu_IsReady(void)
{
    return g_route.ready;
}

void RouteMenu_GetOrder(RouteOrder_t *dst)
{
    if (dst != NULL)
    {
        *dst = g_route;
    }
}

void RouteMenu_Reset(void)
{
    uint8_t i;

    memset(&g_route, 0, sizeof(g_route));
    for (i = 0; i < ROUTE_POINT_NUM; i++)
    {
        g_route.order[i] = 0xFF;
    }

    g_menu_state = MENU_STATE_EDIT;
    g_edit_step = 0;
    g_cursor = 0;
    g_need_refresh = 1;

    RouteMenu_Draw();
}
