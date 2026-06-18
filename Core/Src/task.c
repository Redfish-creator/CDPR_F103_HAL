/* ============================================================
 *  task.c — 应用层: OLED 选点菜单 + 协调运动巡检 + 实时坐标显示
 *
 *  复用现成模块:
 *    route_menu  : 在 OLED 上选 5 个点的巡检顺序 (KEY_EXT1..4)
 *    motion      : 协调运动 motion_move_to(x,y) (已修好,不会过头)
 *    kinematics  : 正解 kin_fk() (4 段绳长 -> 吊舱 x,y)
 *
 *  流程:
 *    上电 -> 菜单选好 5 点顺序 -> 把吊舱摆到中心 -> 按 KEY0 启动
 *    -> 在中心清零 -> 依次 move_to 各点 -> 最后回中心
 *    每到一个点(电机停稳后)读编码器 -> 正解 -> OLED 显示当前坐标
 *
 *  重要约定: 只在电机停稳后读总线/编码器,绝不在运动中读
 *           (motion_move_to 返回时电机已停,所以在它之后读是安全的)
 * ============================================================ */
#include "task.h"
#include "motion.h"
#include "kinematics.h"
#include "motor.h"
#include "oled.h"
#include "route_menu.h"
#include "main.h"          /* HAL + GPIO 宏 */
#include <stdio.h>
#include "vision.h" 


/* ============================================================
 *  >>> 必改 <<<  5 个橙色圆的实测坐标 (单位 cm, 原点=板子中心)
 *  下标 = 菜单里的 P1..P5。下面只是占位:P1~P4 四角, P5 中心。
 *  改成你板子上 5 个圆的真实坐标; 注意每个点都要落在 KIN_XY_LIMIT 之内,
 *  否则 motion_move_to 会打印 REJECT 并跳过。
 * ============================================================ */
static const float PT_X[ROUTE_POINT_NUM] = { -15.0f, +15.0f, +15.0f, -15.0f,  0.0f };
static const float PT_Y[ROUTE_POINT_NUM] = { +15.0f, +15.0f, -15.0f, -15.0f,  0.0f };
/* ============================================================ */

#define HOME_OK_CM          2.0f
#define HOME_STEP_MAX_CM    5.0f
#define HOME_MAX_ITER       10
#define HOME_VISION_AGE_MS  500

typedef enum { TS_MENU = 0, TS_DONE } TaskState_t;

static TaskState_t s_state          = TS_MENU;
static uint32_t    s_last_menu_tick = 0;
static uint8_t     s_key0_last      = 0;

/* ---------- KEY0 = PC5, 低电平按下, 用作"启动 / 再来一次" ---------- */
static uint8_t Key0_Raw(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) ? 1 : 0;
}

/* 上升沿(按下瞬间)触发一次 */
static uint8_t Key0_Edge(void)
{
    uint8_t raw  = Key0_Raw();
    uint8_t edge = (raw && !s_key0_last) ? 1 : 0;
    s_key0_last = raw;
    return edge;
}

/* ---------- 读 4 个编码器 -> 正解 -> OLED 显示当前坐标 ----------
 * CW(编码器读数增大)=收线=绳变短, 所以:
 *   当前绳长 = home 处绳长 - (编码器读数 / 每厘米计数)
 * 只在电机停稳后调用。 */
static void Task_ShowPos(const char *tag, uint8_t step, uint8_t total)
{
    int32_t c[4];
    float   L[4], x = 0.0f, y = 0.0f;
    char    line[22];
    uint8_t ok = 1;

    for (uint8_t id = 1; id <= 4; id++) {
        if (motor_read_position(id, &c[id - 1]) != MB_OK) ok = 0;
        HAL_Delay(15);
    }
    for (int i = 0; i < 4; i++)
        L[i] = kin_home_len(i) - (float)c[i] / KIN_COUNTS_PER_CM;

    if (!ok || !kin_fk(L, &x, &y))
        motion_get_pos(&x, &y);          /* 读失败就退回用模型坐标 */

    OLED_Clear();
    OLED_ShowLine(0, (char *)tag);
    if (total) {
        snprintf(line, sizeof(line), "STEP:%d/%d", step, total);
        OLED_ShowLine(1, line);
    }
    snprintf(line, sizeof(line), "X:%6.1f", (double)x); OLED_ShowLine(3, line);
    snprintf(line, sizeof(line), "Y:%6.1f", (double)y); OLED_ShowLine(4, line);
    OLED_Refresh();
}

/* ---------- 执行一趟巡检(阻塞) ----------
 * 前提: 启动前把吊舱摆到中心, 按 KEY0 时在中心清零。 */
static void Task_RunPatrol(const RouteOrder_t *r)
{
    OLED_Clear(); OLED_ShowLine(0, "HOMING..."); OLED_Refresh();
    if (motion_set_home() != MB_OK) {
        OLED_Clear(); OLED_ShowLine(0, "HOME ZERO FAIL"); OLED_Refresh();
        return;
    }
    HAL_Delay(300);

    for (uint8_t i = 0; i < r->count; i++) {
        uint8_t p = r->order[i];
        if (p >= ROUTE_POINT_NUM) continue;
        if (motion_move_to(PT_X[p], PT_Y[p]) != MB_OK) {
            OLED_Clear(); OLED_ShowLine(0, "MOVE FAIL"); OLED_ShowLine(2, "PATROL ABORT"); OLED_Refresh();
            return;
        }
        Task_ShowPos("PATROL", (uint8_t)(i + 1), r->count);
        HAL_Delay(600);
    }
    if (motion_move_to(0.0f, 0.0f) != MB_OK) { OLED_Clear(); OLED_ShowLine(0, "HOME FAIL"); OLED_Refresh(); return; }
    Task_ShowPos("RETURN HOME", 0, 0);
}

/* ============================ 对外接口 ============================ */

void Task_Init(void)
{
    /* 注意: 调用前 main 里必须已经做了
     *   MX_GPIO_Init()  (按键 KEY_EXT1..4 + KEY0/PC5 输入)
     *   MX_I2C1_Init()  (OLED)
     *   USART3 初始化   (电机 Modbus) */
    motion_init();
    motion_enable_all(1);
    HAL_Delay(200);

    RouteMenu_Init();                    /* 内部会 OLED_Init 并画出选点菜单 */

    s_state          = TS_MENU;
    s_last_menu_tick = HAL_GetTick();
    s_key0_last      = Key0_Raw();       /* 记下当前电平, 避免开机误触发 */
}

void Task_Loop(void)
{
    switch (s_state) {

    case TS_MENU:
        /* 菜单按键 10ms 扫描一次 */
        if (HAL_GetTick() - s_last_menu_tick >= 10) {
            s_last_menu_tick = HAL_GetTick();
            RouteMenu_Task_10ms();
        }
        /* 选满 5 点 (READY) 且按下 KEY0 -> 启动巡检 */
        if (RouteMenu_IsReady() && Key0_Edge()) {
            RouteOrder_t r;
            RouteMenu_GetOrder(&r);

            Task_RunPatrol(&r);

            OLED_Clear();
            OLED_ShowLine(0, "PATROL DONE");
            OLED_ShowLine(2, "KEY0: AGAIN");
            OLED_Refresh();
            s_state = TS_DONE;
        }
        break;

    case TS_DONE:
        /* 再按一次 KEY0 -> 重新选点 */
        if (Key0_Edge()) {
            RouteMenu_Reset();
            s_state          = TS_MENU;
            s_last_menu_tick = HAL_GetTick();
        }
        break;

    default:
        s_state = TS_MENU;
        break;
    }
}

static uint8_t Task_GetVisionLaser(float *x, float *y)
{
    if (!vision_is_online(HOME_VISION_AGE_MS)) return 0;   /* 彻底掉线 */
    return vision_read_filtered(x, y);                     /* 多帧中值 */
}