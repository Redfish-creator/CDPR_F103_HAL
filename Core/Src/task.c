/* ============================================================
 * task.c - application layer
 *
 * Flow:
 *   power on -> KEY0 visual homing -> set motor zero at real center
 *            -> route menu -> patrol -> return center
 *
 * MaixCAM coordinates are assumed to be aligned with the board:
 *   origin = board center, unit = cm, +x right, +y up.
 * ============================================================ */
#include "task.h"
#include "motion.h"
#include "kinematics.h"
#include "motor.h"
#include "oled.h"
#include "route_menu.h"
#include "main.h"
#include "vision.h"
#include <math.h>
#include <stdio.h>

/* Measured orange circle coordinates, unit: cm, origin: board center.
 * P1..P5 in route_menu map to index 0..4 here.
 * These are placeholders and must be replaced by real measured values. */
static const float PT_X[ROUTE_POINT_NUM] = { -15.0f, +15.0f, +15.0f, -15.0f,  0.0f };
static const float PT_Y[ROUTE_POINT_NUM] = { +15.0f, +15.0f, -15.0f, -15.0f,  0.0f };

#define HOME_OK_CM          2.0f
#define HOME_STEP_MAX_CM    5.0f
#define HOME_MAX_ITER       10U
#define HOME_VISION_AGE_MS  500U

#define FIRE_RECORD_COUNT       2U
#define FIRE_BEEP_ON_MS       350U
#define FIRE_BEEP_OFF_MS      250U
#define FIRE_BEEP_REPEAT        3U
#define FIRE_REARM_CLEAR_MS  3000U

/* PB1 buzzer. Default assumes active-low buzzer module. */
#ifndef BUZZER_Pin
#define BUZZER_Pin        GPIO_PIN_1
#define BUZZER_GPIO_Port  GPIOB
#endif
#define BUZZER_ON_LEVEL   GPIO_PIN_RESET
#define BUZZER_OFF_LEVEL  GPIO_PIN_SET

typedef enum {
    TS_HOME = 0,
    TS_MENU,
    TS_DONE
} TaskState_t;

typedef enum {
    FIRE_ALARM_IDLE = 0,
    FIRE_ALARM_ON,
    FIRE_ALARM_OFF
} FireAlarmState_t;

typedef struct {
    uint8_t  valid;
    float    x;
    float    y;
    uint32_t tick_ms;
} FireRecord_t;

static TaskState_t s_state = TS_HOME;
static uint32_t s_last_menu_tick = 0;
static uint8_t s_key0_last = 0;

static FireRecord_t s_fire_records[FIRE_RECORD_COUNT];
static uint8_t s_fire_next = 0;
static uint8_t s_fire_latched = 0;
static FireAlarmState_t s_fire_state = FIRE_ALARM_IDLE;
static uint32_t s_fire_state_tick = 0;
static uint8_t s_fire_beep_count = 0;

static uint8_t Key0_Raw(void);
static uint8_t Key0_Edge(void);
static void Task_ShowHomePrompt(void);
static void Task_ShowPos(const char *tag, uint8_t step, uint8_t total);
static uint8_t Task_GetVisionLaser(float *x, float *y);
static uint8_t Task_ReturnCenter_ByVision(void);
static uint8_t Task_RunPatrol(const RouteOrder_t *r);
static void Task_DelayService(uint32_t delay_ms);
static void Task_BuzzerSet(uint8_t on);
static void Task_FireService(void);
static void Task_RecordFire(float x, float y);
static void Task_DrawFireRecords(void);
static int Task_RoundToInt(float value);

static uint8_t Key0_Raw(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint8_t Key0_Edge(void)
{
    uint8_t raw = Key0_Raw();
    uint8_t edge = (raw != 0U && s_key0_last == 0U) ? 1U : 0U;
    s_key0_last = raw;
    return edge;
}

static int Task_RoundToInt(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

static void Task_BuzzerSet(uint8_t on)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                      (on != 0U) ? BUZZER_ON_LEVEL : BUZZER_OFF_LEVEL);
}

static void Task_RecordFire(float x, float y)
{
    FireRecord_t *rec = &s_fire_records[s_fire_next];

    rec->valid = 1U;
    rec->x = x;
    rec->y = y;
    rec->tick_ms = HAL_GetTick();
    s_fire_next = (uint8_t)((s_fire_next + 1U) % FIRE_RECORD_COUNT);

    printf("FIRE record x=%.1f y=%.1f tick=%lu\r\n",
           (double)x, (double)y, (unsigned long)rec->tick_ms);
}

static void Task_FireService(void)
{
    uint32_t now = HAL_GetTick();

    if (g_vision.fire_fresh != 0U) {
        float fx = g_vision.fire_x;
        float fy = g_vision.fire_y;
        g_vision.fire_fresh = 0U;

        if (s_fire_latched == 0U) {
            s_fire_latched = 1U;
            Task_RecordFire(fx, fy);
            s_fire_state = FIRE_ALARM_ON;
            s_fire_state_tick = now;
            s_fire_beep_count = 0U;
            Task_BuzzerSet(1U);
        }
    }

    if ((s_fire_latched != 0U) &&
        (g_vision.fire_ms != 0U) &&
        ((now - g_vision.fire_ms) >= FIRE_REARM_CLEAR_MS)) {
        s_fire_latched = 0U;
    }

    switch (s_fire_state) {
    case FIRE_ALARM_ON:
        if ((now - s_fire_state_tick) >= FIRE_BEEP_ON_MS) {
            Task_BuzzerSet(0U);
            s_fire_beep_count++;
            s_fire_state_tick = now;
            s_fire_state = (s_fire_beep_count >= FIRE_BEEP_REPEAT) ?
                           FIRE_ALARM_IDLE : FIRE_ALARM_OFF;
        }
        break;

    case FIRE_ALARM_OFF:
        if ((now - s_fire_state_tick) >= FIRE_BEEP_OFF_MS) {
            Task_BuzzerSet(1U);
            s_fire_state_tick = now;
            s_fire_state = FIRE_ALARM_ON;
        }
        break;

    case FIRE_ALARM_IDLE:
    default:
        Task_BuzzerSet(0U);
        break;
    }
}

static void Task_DelayService(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < delay_ms) {
        vision_task();
        Task_FireService();
        HAL_Delay(5);
    }
}

static void Task_DrawFireRecords(void)
{
    char line[22];

    for (uint8_t i = 0; i < FIRE_RECORD_COUNT; i++) {
        if (s_fire_records[i].valid != 0U) {
            snprintf(line, sizeof(line), "F%d X:%4d Y:%4d",
                     (int)(i + 1U),
                     Task_RoundToInt(s_fire_records[i].x),
                     Task_RoundToInt(s_fire_records[i].y));
        } else {
            snprintf(line, sizeof(line), "F%d --", (int)(i + 1U));
        }
        OLED_ShowLine((uint8_t)(6U + i), line);
    }
}

static void Task_ShowHomePrompt(void)
{
    OLED_Clear();
    OLED_ShowLine(0, "HOME REQUIRED");
    OLED_ShowLine(2, "KEY0: HOME");
    OLED_ShowLine(4, "CAM READY FIRST");
    Task_DrawFireRecords();
    OLED_Refresh();
}

static uint8_t Task_ReadEncoderPos(float *x, float *y)
{
    int32_t c[4];
    float L[4];

    for (uint8_t id = 1; id <= 4; id++) {
        if (motor_read_position(id, &c[id - 1U]) != MB_OK) {
            return 0U;
        }
        HAL_Delay(15);
    }

    for (int i = 0; i < 4; i++) {
        L[i] = kin_home_len(i) - (float)c[i] / KIN_COUNTS_PER_CM;
    }

    return kin_fk(L, x, y);
}

static void Task_ShowPos(const char *tag, uint8_t step, uint8_t total)
{
    float x = 0.0f;
    float y = 0.0f;
    char line[22];
    uint8_t ok = 0U;

    if (vision_is_online(HOME_VISION_AGE_MS) != 0U) {
        ok = vision_read_filtered(&x, &y);
    }

    if (ok == 0U) {
        ok = Task_ReadEncoderPos(&x, &y);
    }

    if (ok == 0U) {
        motion_get_pos(&x, &y);
    }

    OLED_Clear();
    OLED_ShowLine(0, tag);
    if (total != 0U) {
        snprintf(line, sizeof(line), "STEP:%d/%d", step, total);
        OLED_ShowLine(1, line);
    }
    snprintf(line, sizeof(line), "X:%6.1f", (double)x);
    OLED_ShowLine(3, line);
    snprintf(line, sizeof(line), "Y:%6.1f", (double)y);
    OLED_ShowLine(4, line);
    Task_DrawFireRecords();
    OLED_Refresh();
}

static uint8_t Task_GetVisionLaser(float *x, float *y)
{
    if (vision_is_online(HOME_VISION_AGE_MS) == 0U) {
        return 0U;
    }
    return vision_read_filtered(x, y);
}

static uint8_t Task_ReturnCenter_ByVision(void)
{
    char line[22];
    uint8_t centered = 0U;

    OLED_Clear();
    OLED_ShowLine(0, "HOMING BY CAM");
    OLED_Refresh();
    Task_DelayService(200);

    if (vision_is_online(HOME_VISION_AGE_MS) == 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "VISION LOST");
        OLED_ShowLine(2, "KEY0: RETRY");
        OLED_Refresh();
        printf("HOME abort: vision offline\r\n");
        return 0U;
    }

    for (uint8_t k = 0; k < HOME_MAX_ITER; k++) {
        float xm;
        float ym;
        float dist;
        float step;
        float xn;
        float yn;

        if (Task_GetVisionLaser(&xm, &ym) == 0U) {
            OLED_Clear();
            OLED_ShowLine(0, "VISION LOST");
            OLED_ShowLine(2, "KEY0: RETRY");
            OLED_Refresh();
            printf("HOME abort: filtered laser failed\r\n");
            return 0U;
        }

        dist = sqrtf(xm * xm + ym * ym);
        printf("HOME k=%d x=%.2f y=%.2f d=%.2f\r\n",
               (int)k, (double)xm, (double)ym, (double)dist);

        snprintf(line, sizeof(line), "D:%5.1f K:%d", (double)dist, (int)k);
        OLED_ShowLine(2, line);
        OLED_Refresh();

        if (dist <= HOME_OK_CM) {
            centered = 1U;
            break;
        }

        if (kin_in_range(xm, ym) == 0U) {
            OLED_Clear();
            OLED_ShowLine(0, "HOME RANGE");
            OLED_ShowLine(2, "MOVE MANUAL");
            OLED_Refresh();
            printf("HOME abort: current point out of range\r\n");
            return 0U;
        }

        step = (dist < HOME_STEP_MAX_CM) ? dist : HOME_STEP_MAX_CM;
        xn = xm - (xm / dist) * step;
        yn = ym - (ym / dist) * step;

        if (motion_move_between_nohome(xm, ym, xn, yn) != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "HOME MOVE FAIL");
            OLED_ShowLine(2, "KEY0: RETRY");
            OLED_Refresh();
            return 0U;
        }

        Task_DelayService(100);
    }

    if (centered == 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "HOME FAIL");
        OLED_ShowLine(2, "NOT CENTERED");
        OLED_Refresh();
        printf("HOME abort: not centered\r\n");
        return 0U;
    }

    if (motion_set_home() != MB_OK) {
        OLED_Clear();
        OLED_ShowLine(0, "HOME ZERO FAIL");
        OLED_ShowLine(2, "KEY0: RETRY");
        OLED_Refresh();
        return 0U;
    }

    OLED_Clear();
    OLED_ShowLine(0, "HOME DONE");
    OLED_Refresh();
    printf("HOME done\r\n");
    Task_DelayService(600);
    return 1U;
}

static uint8_t Task_RunPatrol(const RouteOrder_t *r)
{
    if (r == NULL || r->count == 0U) {
        return 0U;
    }

    for (uint8_t i = 0; i < r->count; i++) {
        uint8_t p = r->order[i];
        uint8_t tuned;

        if (p >= ROUTE_POINT_NUM) {
            continue;
        }

        OLED_Clear();
        OLED_ShowLine(0, "MOVING");
        OLED_Refresh();

        if (motion_move_to(PT_X[p], PT_Y[p]) != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "MOVE FAIL");
            OLED_ShowLine(2, "PATROL ABORT");
            OLED_Refresh();
            return 0U;
        }

        tuned = fine_tune_to(PT_X[p], PT_Y[p]);
        Task_FireService();
        Task_ShowPos((tuned != 0U) ? "PATROL" : "PATROL RAW",
                     (uint8_t)(i + 1U), r->count);
        Task_DelayService(600);
    }

    if (motion_move_to(0.0f, 0.0f) != MB_OK) {
        OLED_Clear();
        OLED_ShowLine(0, "HOME FAIL");
        OLED_Refresh();
        return 0U;
    }
    (void)fine_tune_to(0.0f, 0.0f);
    Task_ShowPos("RETURN HOME", 0U, 0U);
    return 1U;
}

void Task_Init(void)
{
    motion_init();
    (void)motion_enable_all(1U);
    Task_BuzzerSet(0U);

    OLED_Init();
    s_state = TS_HOME;
    s_last_menu_tick = HAL_GetTick();
    s_key0_last = Key0_Raw();
    Task_ShowHomePrompt();

    printf("\r\nCDPR ready: press KEY0 to visual home\r\n");
}

void Task_Loop(void)
{
    Task_FireService();

    switch (s_state) {
    case TS_HOME:
        if (Key0_Edge() != 0U) {
            if (Task_ReturnCenter_ByVision() != 0U) {
                RouteMenu_Init();
                s_state = TS_MENU;
                s_last_menu_tick = HAL_GetTick();
                s_key0_last = Key0_Raw();
            } else {
                Task_DelayService(1000);
                Task_ShowHomePrompt();
            }
        }
        break;

    case TS_MENU:
        if ((HAL_GetTick() - s_last_menu_tick) >= 10U) {
            s_last_menu_tick = HAL_GetTick();
            RouteMenu_Task_10ms();
        }

        if (RouteMenu_IsReady() != 0U && Key0_Edge() != 0U) {
            RouteOrder_t r;
            uint8_t ok;

            RouteMenu_GetOrder(&r);
            ok = Task_RunPatrol(&r);

            if (ok != 0U) {
                OLED_Clear();
                OLED_ShowLine(0, "PATROL DONE");
                OLED_ShowLine(2, "KEY0: AGAIN");
                Task_DrawFireRecords();
                OLED_Refresh();
            } else {
                OLED_ShowLine(5, "KEY0: AGAIN");
                OLED_Refresh();
            }
            s_state = TS_DONE;
            s_key0_last = Key0_Raw();
        }
        break;

    case TS_DONE:
        if (Key0_Edge() != 0U) {
            s_state = TS_HOME;
            s_last_menu_tick = HAL_GetTick();
            Task_ShowHomePrompt();
            s_key0_last = Key0_Raw();
        }
        break;

    default:
        s_state = TS_HOME;
        Task_ShowHomePrompt();
        break;
    }
}
