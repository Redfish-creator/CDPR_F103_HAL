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
#include <string.h>

/* Measured orange circle coordinates, unit: cm, origin: board center.
 * P1..P5 in route_menu map to index 0..4 here.
 * These are placeholders and must be replaced by real measured values. */
static const float PT_X[ROUTE_POINT_NUM] = { -15.0f, +15.0f, +15.0f, -15.0f,  0.0f };
static const float PT_Y[ROUTE_POINT_NUM] = { +15.0f, +15.0f, -15.0f, -15.0f,  0.0f };

#ifndef TASK_BASIC1_ONLY
#define TASK_BASIC1_ONLY    1U    /* 1=基本要求(1): 只做视觉回中心, 不进入五点菜单 */
#endif

#ifndef TASK_MANUAL_ZERO_MODE
#define TASK_MANUAL_ZERO_MODE 1U  /* 1=手动记录中心->按键点动->按KEY0电机位置回零 */
#endif

#define HOME_OK_CM          0.6f
#define HOME_STEP_MAX_CM    0.8f
#define HOME_MAX_ITER       30U
#define HOME_VISION_AGE_MS  500U
#define HOME_MIN_IMPROVE_CM 0.08f
#define HOME_DIVERGE_CM     0.80f
#define HOME_MAX_STALE      3U
#define HOME_ROUGH_DONE_CM  12.0f
#define HOME_ROUGH_STEP_CM  0.6f
#define HOME_ROUGH_MAX_ITER 50U
#define HOME_FINE_DIRECT_CM HOME_ROUGH_DONE_CM
#define HOME_CENTER_REF_TOL_CM 3.0f
#define HOME_TOTAL_CORNER_CM 20.0f
#define HOME_NOHOME_CALC_LIMIT_CM 21.8f
#define HOME_RELAX_FINE_CM  3.0f
#define HOME_RELAX_TAKEUP_GAIN 0.65f
#define HOME_RELAX_PAYOUT_GAIN 1.20f

#define MANUAL_RETURN_STEP_CM     0.5f
#define MANUAL_RETURN_TOL_COUNTS  20L
#define MANUAL_RETURN_BASE_VMAX   600U
#define MANUAL_RETURN_MAX_SEG     80U
#define MANUAL_KEY_PERIOD_MS      10U
#define MANUAL_JOG_RUN_CM         4.0f
#define MANUAL_JOG_EDGE_RUN_CM    2.0f
#define MANUAL_JOG_CORNER_RUN_CM  1.0f
#define MANUAL_JOG_EDGE_CM        14.0f
#define MANUAL_JOG_CORNER_CM      17.0f
#define MANUAL_JOG_SAFE_LIMIT_CM  18.0f
#define MANUAL_JOG_TAKEUP_GAIN    1.04f
#define MANUAL_JOG_PAYOUT_GAIN    0.98f
#define MANUAL_JOG_GAP_MS         20U
#define MANUAL_JOG_X_UP_COMP      0.42f
#define MANUAL_JOG_OLED_MS        200U
#define MANUAL_AUTO_HOME_FAST_STEP_CM 2.0f
#define MANUAL_AUTO_HOME_FINE_STEP_CM 0.5f
#define MANUAL_AUTO_HOME_FINE_ZONE_CM 3.0f
#define MANUAL_AUTO_HOME_MAX_ITER 30U
#define MANUAL_AUTO_HOME_MAX_START_CM 16.0f
#define MANUAL_CENTER_TENSION_ENABLE 1U
#define MANUAL_CENTER_TENSION_COUNTS 40U
#define MANUAL_CENTER_TENSION_VMAX   60U
#define MANUAL_CENTER_TENSION_SETTLE_MS 300U
#define MANUAL_CENTER_TENSION_REHOME_STEP_CM 0.35f
#define MANUAL_CENTER_TENSION_REHOME_MAX_ITER 8U
#define MANUAL_VISION_LOG         1U
#define MANUAL_VISION_LIVE_MS     500U
#define MANUAL_VISION_STEP_SAMPLE 0U

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
    TS_DONE,
    TS_MANUAL_SET_ZERO,
    TS_MANUAL_JOG
} TaskState_t;

typedef enum {
    FIRE_ALARM_IDLE = 0,
    FIRE_ALARM_ON,
    FIRE_ALARM_OFF
} FireAlarmState_t;

typedef enum {
    MANUAL_JOG_DIR_NONE = 0,
    MANUAL_JOG_DIR_UP,
    MANUAL_JOG_DIR_DOWN,
    MANUAL_JOG_DIR_LEFT,
    MANUAL_JOG_DIR_RIGHT
} ManualJogDir_t;

typedef struct {
    uint8_t  valid;
    float    x;
    float    y;
    uint32_t tick_ms;
} FireRecord_t;

typedef struct {
    uint8_t stable_level;
    uint8_t last_sample;
    uint8_t debounce_cnt;
} ManualKeyFilter_t;

static TaskState_t s_state = TS_HOME;
static uint32_t s_last_menu_tick = 0;
static uint8_t s_key0_last = 0;

static FireRecord_t s_fire_records[FIRE_RECORD_COUNT];
static uint8_t s_fire_next = 0;
static uint8_t s_fire_latched = 0;
static FireAlarmState_t s_fire_state = FIRE_ALARM_IDLE;
static uint32_t s_fire_state_tick = 0;
static uint8_t s_fire_beep_count = 0;

static int32_t s_manual_zero_pos[4] = {0};
static uint8_t s_manual_zero_valid = 0U;
static float s_manual_x = 0.0f;
static float s_manual_y = 0.0f;
static uint32_t s_manual_key_tick = 0U;
static uint32_t s_manual_jog_tick = 0U;
static uint32_t s_manual_jog_done_tick = 0U;
static uint32_t s_manual_jog_run_start_tick = 0U;
static uint32_t s_manual_jog_run_duration_ms = 0U;
static uint32_t s_manual_oled_tick = 0U;
static uint32_t s_manual_vision_log_tick = 0U;
static ManualJogDir_t s_manual_jog_dir = MANUAL_JOG_DIR_NONE;
static uint8_t s_manual_jog_active = 0U;
static float s_manual_jog_start_x = 0.0f;
static float s_manual_jog_start_y = 0.0f;
static float s_manual_jog_target_x = 0.0f;
static float s_manual_jog_target_y = 0.0f;
static ManualJogDir_t s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;
static ManualKeyFilter_t s_manual_key_up;
static ManualKeyFilter_t s_manual_key_down;
static ManualKeyFilter_t s_manual_key_left;
static ManualKeyFilter_t s_manual_key_right;

static uint8_t Key0_Raw(void);
static uint8_t Key0_Edge(void);
static uint8_t Task_ReadMotorPositionRetry(uint8_t id, int32_t *pos);
static int32_t Task_AbsI32(int32_t value);
static void ManualKey_Update(ManualKeyFilter_t *kf, uint8_t raw_pressed);
static uint8_t ManualKey_IsPressed(const ManualKeyFilter_t *kf);
static void Task_ManualKeysInit(void);
static void Task_ManualKeysService(void);
static void Task_ShowManualZeroPrompt(void);
static void Task_ShowManualJog(void);
static void Task_ShowManualJogThrottled(void);
#if MANUAL_VISION_STEP_SAMPLE
static uint8_t Task_ManualVisionSample(const char *tag, float *x, float *y);
#endif
static void Task_ManualVisionLiveLog(const char *tag);
static mb_result_t Task_ManualCenterPretension(void);
static uint8_t Task_ManualRecenterAfterPretension(void);
static uint8_t Task_ManualAutoCenterAndRecord(uint8_t normalize_tension);
static const char *Task_ManualJogDirName(ManualJogDir_t dir);
static ManualJogDir_t Task_ManualReadJogDir(void);
static void Task_ClampManualJogPoint(float *x, float *y);
static float Task_ManualJogRunCm(void);
static void Task_ManualJogIntegrate(uint32_t now);
static void Task_ManualJogStop(void);
static void Task_ManualJogStopForce(void);
static uint8_t Task_ManualJogCommand(ManualJogDir_t dir, uint8_t force);
static mb_result_t Task_ManualSendMotorDelta(const int32_t delta[4]);
static uint8_t Task_ManualReturnZero(void);
static void Task_ManualZeroLoop(void);
static void Task_ShowHomePrompt(void);
static void Task_ShowPos(const char *tag, uint8_t step, uint8_t total);
static uint8_t Task_MatchKnownVisionCircle(float cx, float cy, float *ref_x, float *ref_y);
static void Task_ClampNoHomeCalcPoint(float *x, float *y);
static uint8_t Task_GetVisionGlobal(float *x, float *y, uint8_t *center_ref_seen);
static uint8_t Task_GetVisionLaser(float *x, float *y);
static uint8_t Task_RoughCenter_ByGlobal(void);
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

static uint8_t Task_ReadMotorPositionRetry(uint8_t id, int32_t *pos)
{
    for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
        mb_result_t r = motor_read_position(id, pos);
        if (r == MB_OK) {
            return 1U;
        }
        printf("MANUAL read m%d attempt=%d %s\r\n",
               (int)id, (int)attempt + 1, motor_result_str(r));
        HAL_Delay(40);
    }

    return 0U;
}

static int32_t Task_AbsI32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static void ManualKey_Update(ManualKeyFilter_t *kf, uint8_t raw_pressed)
{
    if (raw_pressed == kf->last_sample) {
        if (kf->debounce_cnt < 3U) {
            kf->debounce_cnt++;
        }
    } else {
        kf->debounce_cnt = 0U;
        kf->last_sample = raw_pressed;
    }

    if (kf->debounce_cnt >= 2U) {
        if (kf->stable_level != raw_pressed) {
            kf->stable_level = raw_pressed;
        }
    }
}

static uint8_t ManualKey_IsPressed(const ManualKeyFilter_t *kf)
{
    return (kf->stable_level != 0U) ? 1U : 0U;
}

static void Task_ManualKeysInit(void)
{
    memset(&s_manual_key_up, 0, sizeof(s_manual_key_up));
    memset(&s_manual_key_down, 0, sizeof(s_manual_key_down));
    memset(&s_manual_key_left, 0, sizeof(s_manual_key_left));
    memset(&s_manual_key_right, 0, sizeof(s_manual_key_right));
    s_manual_key_tick = HAL_GetTick();
    s_manual_jog_tick = s_manual_key_tick;
    s_manual_jog_done_tick = s_manual_key_tick;
    s_manual_jog_run_start_tick = s_manual_key_tick;
    s_manual_jog_run_duration_ms = 0U;
    s_manual_oled_tick = s_manual_key_tick;
    s_manual_jog_dir = MANUAL_JOG_DIR_NONE;
    s_manual_jog_active = 0U;
    s_manual_jog_start_x = 0.0f;
    s_manual_jog_start_y = 0.0f;
    s_manual_jog_target_x = 0.0f;
    s_manual_jog_target_y = 0.0f;
    s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;
}

static void Task_ManualKeysService(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_manual_key_tick) < MANUAL_KEY_PERIOD_MS) {
        return;
    }
    s_manual_key_tick = now;

    ManualKey_Update(&s_manual_key_up,
                     (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1U : 0U);
    ManualKey_Update(&s_manual_key_down,
                     (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1U : 0U);
    ManualKey_Update(&s_manual_key_left,
                     (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) ? 1U : 0U);
    ManualKey_Update(&s_manual_key_right,
                     (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1U : 0U);
}

static void Task_ShowManualZeroPrompt(void)
{
    OLED_Clear();
    OLED_ShowLine(0, "AUTO CENTER");
    OLED_ShowLine(2, "PLACE NEAR 0");
    OLED_ShowLine(4, "KEY0: HOME+ZERO");
    OLED_ShowLine(6, "THEN PC0-3 JOG");
    OLED_Refresh();
}

static void Task_ShowManualJog(void)
{
    char line[22];

    OLED_Clear();
    OLED_ShowLine(0, "MANUAL JOG");
    snprintf(line, sizeof(line), "X:%5.1f Y:%5.1f", (double)s_manual_x, (double)s_manual_y);
    OLED_ShowLine(1, line);
    OLED_ShowLine(2, "PC0 UP  PC1 DOWN");
    OLED_ShowLine(3, "PC2 LEFT PC3 RIGHT");
    OLED_ShowLine(5, "KEY0: RETURN 0");
    OLED_Refresh();
    s_manual_oled_tick = HAL_GetTick();
}

static void Task_ShowManualJogThrottled(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_manual_oled_tick) >= MANUAL_JOG_OLED_MS) {
        Task_ShowManualJog();
    }
}

#if MANUAL_VISION_STEP_SAMPLE
static uint8_t Task_ManualVisionSample(const char *tag, float *x, float *y)
{
    const char *label = (tag != NULL) ? tag : "sample";
    float vx = 0.0f;
    float vy = 0.0f;
    float circle_x = 0.0f;
    float circle_y = 0.0f;
    float ref_x = 0.0f;
    float ref_y = 0.0f;
    const char *src = "NONE";

    if (vision_is_online(HOME_VISION_AGE_MS) == 0U) {
        printf("MANUAL vision %s unavailable: offline valid_age=%lums\r\n",
               label, (unsigned long)(HAL_GetTick() - g_vision.last_valid_ms));
        return 0U;
    }

    if (vision_read_home_filtered(&vx, &vy) != 0U) {
        src = "HOME";
    } else if (vision_read_circle_error_with_ref_filtered(&vx, &vy,
                                                          &circle_x, &circle_y) != 0U) {
        src = "LASER-CIRCLE";
        if (Task_MatchKnownVisionCircle(circle_x, circle_y, &ref_x, &ref_y) != 0U &&
            (fabsf(ref_x) > 0.1f || fabsf(ref_y) > 0.1f)) {
            printf("MANUAL vision %s reject: circle ref=(%.1f,%.1f), need center for local log\r\n",
                   label, (double)ref_x, (double)ref_y);
            return 0U;
        }
    } else if (vision_read_filtered(&vx, &vy) != 0U) {
        src = "LASER";
    } else {
        printf("MANUAL vision %s unavailable: home=%lu laser=%lu circle=%lu\r\n",
               label,
               (unsigned long)g_vision.home_count,
               (unsigned long)g_vision.laser_count,
               (unsigned long)g_vision.circle_count);
        return 0U;
    }

    if (x != NULL) *x = vx;
    if (y != NULL) *y = vy;
    printf("MANUAL vision %s src=%s x=%.2f y=%.2f d=%.2f\r\n",
           label, src, (double)vx, (double)vy,
           (double)sqrtf(vx * vx + vy * vy));
    return 1U;
}
#endif

static void Task_ManualVisionLiveLog(const char *tag)
{
#if MANUAL_VISION_LOG
    uint32_t now = HAL_GetTick();
    const char *label = (tag != NULL) ? tag : "live";

    if ((now - s_manual_vision_log_tick) < MANUAL_VISION_LIVE_MS) {
        return;
    }
    s_manual_vision_log_tick = now;

    printf("MANUAL vision %s latest", label);
    if (g_vision.home_count != 0U) {
        printf(" HOME cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.home_count,
               (double)g_vision.home_x,
               (double)g_vision.home_y,
               (unsigned long)(now - g_vision.home_ms));
    }
    if (g_vision.laser_count != 0U) {
        printf(" LASER cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.laser_count,
               (double)g_vision.laser_x,
               (double)g_vision.laser_y,
               (unsigned long)(now - g_vision.laser_ms));
    }
    if (g_vision.circle_count != 0U) {
        printf(" CIRCLE cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.circle_count,
               (double)g_vision.circle_x,
               (double)g_vision.circle_y,
               (unsigned long)(now - g_vision.circle_ms));
    }
    if (g_vision.tilt_count != 0U) {
        printf(" TILT cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.tilt_count,
               (double)g_vision.tilt_x,
               (double)g_vision.tilt_y,
               (unsigned long)(now - g_vision.tilt_ms));
    }
    if (g_vision.laser_count != 0U && g_vision.circle_count != 0U) {
        printf(" err=(%.2f,%.2f)",
               (double)(g_vision.laser_x - g_vision.circle_x),
               (double)(g_vision.laser_y - g_vision.circle_y));
    }
    printf(" soft=(%.2f,%.2f)\r\n", (double)s_manual_x, (double)s_manual_y);
#else
    (void)tag;
#endif
}

static mb_result_t Task_ManualCenterPretension(void)
{
#if MANUAL_CENTER_TENSION_ENABLE
    printf("MANUAL tension normalize start count=%u vmax=%u\r\n",
           (unsigned int)MANUAL_CENTER_TENSION_COUNTS,
           (unsigned int)MANUAL_CENTER_TENSION_VMAX);

    for (uint8_t id = 1U; id <= 4U; id++) {
        uint8_t dir = motor_cable_direction(id, MOTOR_CABLE_TAKEUP);
        mb_result_t r = motor_move_pos(id,
                                       dir,
                                       MANUAL_CENTER_TENSION_VMAX,
                                       MANUAL_CENTER_TENSION_COUNTS,
                                       MB_MODE_REL_CUR,
                                       MB_SYNC_BUFFER);
        printf("  tension buf m%d dir=%s mag=%u: %s\r\n",
               (int)id,
               (dir == MB_DIR_CW) ? "CW" : "CCW",
               (unsigned int)MANUAL_CENTER_TENSION_COUNTS,
               motor_result_str(r));
        if (r != MB_OK) {
            for (uint8_t sid = 1U; sid <= 4U; sid++) {
                (void)motor_stop(sid, MB_SYNC_NOW);
                HAL_Delay(15);
            }
            return r;
        }
        HAL_Delay(25);
    }

    {
        mb_result_t tr = motor_sync_trigger();
        printf("  tension trigger: %s\r\n", motor_result_str(tr));
        if (tr != MB_OK) {
            for (uint8_t sid = 1U; sid <= 4U; sid++) {
                (void)motor_stop(sid, MB_SYNC_NOW);
                HAL_Delay(15);
            }
            return tr;
        }
    }

    Task_DelayService(MANUAL_CENTER_TENSION_SETTLE_MS);
#endif
    return MB_OK;
}

static uint8_t Task_ManualRecenterAfterPretension(void)
{
    for (uint8_t k = 0U; k < MANUAL_CENTER_TENSION_REHOME_MAX_ITER; k++) {
        float xm = 0.0f;
        float ym = 0.0f;
        float dist;
        float step;
        float xn;
        float yn;
        mb_result_t r;

        if (Task_GetVisionLaser(&xm, &ym) == 0U) {
            printf("MANUAL tension recenter abort: no local center error\r\n");
            return 0U;
        }

        dist = sqrtf(xm * xm + ym * ym);
        printf("MANUAL tension recenter k=%d x=%.2f y=%.2f d=%.2f\r\n",
               (int)k, (double)xm, (double)ym, (double)dist);
        if (dist <= HOME_OK_CM) {
            return 1U;
        }
        if (kin_in_range(xm, ym) == 0U) {
            printf("MANUAL tension recenter abort: point out of range\r\n");
            return 0U;
        }

        step = (dist < MANUAL_CENTER_TENSION_REHOME_STEP_CM) ?
               dist : MANUAL_CENTER_TENSION_REHOME_STEP_CM;
        xn = xm - (xm / dist) * step;
        yn = ym - (ym / dist) * step;
        printf("MANUAL tension recenter cmd k=%d target=(%.2f,%.2f) step=%.2f\r\n",
               (int)k, (double)xn, (double)yn, (double)step);

        r = motion_move_between_nohome(xm, ym, xn, yn);
        if (r != MB_OK) {
            printf("MANUAL tension recenter move fail: %s\r\n", motor_result_str(r));
            return 0U;
        }
        Task_DelayService(250);
    }

    printf("MANUAL tension recenter abort: not centered after normalize\r\n");
    return 0U;
}

static uint8_t Task_ManualAutoCenterAndRecord(uint8_t normalize_tension)
{
    char line[22];
    uint8_t centered = 0U;
    uint8_t move_fail_count = 0U;

    OLED_Clear();
    OLED_ShowLine(0, "CENTER BY CAM");
    OLED_ShowLine(2, "LOCAL CIRCLE");
    OLED_Refresh();
    Task_DelayService(200);

    if (vision_is_online(HOME_VISION_AGE_MS) == 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "VISION LOST");
        OLED_ShowLine(2, "KEY0: RETRY");
        OLED_Refresh();
        printf("MANUAL auto home abort: vision offline\r\n");
        return 0U;
    }

    for (uint8_t k = 0U; k < MANUAL_AUTO_HOME_MAX_ITER; k++) {
        float xm = 0.0f;
        float ym = 0.0f;
        float dist;
        float step;
        float xn;
        float yn;
        mb_result_t r;

        if (Task_GetVisionLaser(&xm, &ym) == 0U) {
            OLED_Clear();
            OLED_ShowLine(0, "NO CENTER CIRCLE");
            OLED_ShowLine(2, "PLACE NEAR 0");
            OLED_Refresh();
            printf("MANUAL auto home abort: no local center error\r\n");
            return 0U;
        }

        dist = sqrtf(xm * xm + ym * ym);
        printf("MANUAL auto home k=%d x=%.2f y=%.2f d=%.2f\r\n",
               (int)k, (double)xm, (double)ym, (double)dist);

        snprintf(line, sizeof(line), "D:%5.1f K:%d", (double)dist, (int)k);
        OLED_ShowLine(2, line);
        OLED_Refresh();

        if (dist <= HOME_OK_CM) {
            centered = 1U;
            break;
        }

        if (dist > MANUAL_AUTO_HOME_MAX_START_CM || kin_in_range(xm, ym) == 0U) {
            OLED_Clear();
            OLED_ShowLine(0, "TOO FAR");
            OLED_ShowLine(2, "PLACE NEAR 0");
            OLED_Refresh();
            printf("MANUAL auto home abort: d=%.2f too far, max=%.2f for local-only home\r\n",
                   (double)dist, (double)MANUAL_AUTO_HOME_MAX_START_CM);
            return 0U;
        }

        if (dist > MANUAL_AUTO_HOME_FINE_ZONE_CM) {
            step = dist - MANUAL_AUTO_HOME_FINE_ZONE_CM;
            if (step > MANUAL_AUTO_HOME_FAST_STEP_CM) {
                step = MANUAL_AUTO_HOME_FAST_STEP_CM;
            }
        } else {
            step = (dist < MANUAL_AUTO_HOME_FINE_STEP_CM) ? dist : MANUAL_AUTO_HOME_FINE_STEP_CM;
        }
        xn = xm - (xm / dist) * step;
        yn = ym - (ym / dist) * step;

        printf("MANUAL auto cmd k=%d target=(%.2f,%.2f) step=%.2f mode=EXACT\r\n",
               (int)k, (double)xn, (double)yn, (double)step);
        r = motion_move_between_nohome(xm, ym, xn, yn);
        if (r != MB_OK) {
            move_fail_count++;
            printf("MANUAL auto home move fail: %s retry=%d/3\r\n",
                   motor_result_str(r), (int)move_fail_count);
            if (r != MB_ERR_REJECT && move_fail_count < 3U) {
                Task_DelayService(300);
                continue;
            }
            OLED_Clear();
            OLED_ShowLine(0, "HOME MOVE FAIL");
            OLED_ShowLine(2, "KEY0: RETRY");
            OLED_Refresh();
            return 0U;
        }
        move_fail_count = 0U;

        Task_DelayService(300);
    }

    if (centered == 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "HOME FAIL");
        OLED_ShowLine(2, "NOT CENTERED");
        OLED_Refresh();
        printf("MANUAL auto home abort: not centered\r\n");
        return 0U;
    }

    if (normalize_tension != 0U) {
        mb_result_t tr;

        OLED_Clear();
        OLED_ShowLine(0, "TENSION SET");
        OLED_ShowLine(2, "LIGHT TAKEUP");
        OLED_Refresh();

        tr = Task_ManualCenterPretension();
        if (tr != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "TENSION FAIL");
            OLED_ShowLine(2, "CHECK BUS");
            OLED_Refresh();
            printf("MANUAL auto home abort: tension normalize failed: %s\r\n",
                   motor_result_str(tr));
            return 0U;
        }

        OLED_Clear();
        OLED_ShowLine(0, "TENSION DONE");
        OLED_ShowLine(2, "CAM RECENTER");
        OLED_Refresh();

        if (Task_ManualRecenterAfterPretension() == 0U) {
            OLED_Clear();
            OLED_ShowLine(0, "RECENTER FAIL");
            OLED_ShowLine(2, "KEY0: RETRY");
            OLED_Refresh();
            printf("MANUAL auto home abort: recenter after tension failed\r\n");
            return 0U;
        }
    }

    if (motion_set_home() != MB_OK) {
        OLED_Clear();
        OLED_ShowLine(0, "ZERO FAIL");
        OLED_ShowLine(2, "CHECK MOTOR BUS");
        OLED_Refresh();
        printf("MANUAL auto home abort: motor zero failed\r\n");
        return 0U;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        s_manual_zero_pos[i] = 0;
    }
    s_manual_zero_valid = 1U;
    s_manual_x = 0.0f;
    s_manual_y = 0.0f;
    motion_init();
    s_manual_jog_active = 0U;
    s_manual_jog_dir = MANUAL_JOG_DIR_NONE;
    s_manual_jog_tick = HAL_GetTick();
    s_manual_jog_done_tick = s_manual_jog_tick;
    s_manual_jog_run_start_tick = s_manual_jog_tick;
    s_manual_jog_run_duration_ms = 0U;
    s_manual_jog_start_x = 0.0f;
    s_manual_jog_start_y = 0.0f;
    s_manual_jog_target_x = 0.0f;
    s_manual_jog_target_y = 0.0f;
    s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;

    OLED_Clear();
    OLED_ShowLine(0, "CENTER ZERO OK");
    OLED_ShowLine(2, "PC0-3 JOG");
    OLED_Refresh();
    printf("MANUAL auto home done: motor encoder zero set, manual pos=(0,0)\r\n");
    Task_DelayService(500);
    Task_ShowManualJog();
    return 1U;
}

static const char *Task_ManualJogDirName(ManualJogDir_t dir)
{
    switch (dir) {
    case MANUAL_JOG_DIR_UP:    return "UP";
    case MANUAL_JOG_DIR_DOWN:  return "DOWN";
    case MANUAL_JOG_DIR_LEFT:  return "LEFT";
    case MANUAL_JOG_DIR_RIGHT: return "RIGHT";
    default:                   return "NONE";
    }
}

static ManualJogDir_t Task_ManualReadJogDir(void)
{
    uint8_t up = ManualKey_IsPressed(&s_manual_key_up);
    uint8_t down = ManualKey_IsPressed(&s_manual_key_down);
    uint8_t left = ManualKey_IsPressed(&s_manual_key_left);
    uint8_t right = ManualKey_IsPressed(&s_manual_key_right);

    if (up != 0U) return MANUAL_JOG_DIR_UP;
    if (down != 0U) return MANUAL_JOG_DIR_DOWN;
    if (left != 0U) return MANUAL_JOG_DIR_LEFT;
    if (right != 0U) return MANUAL_JOG_DIR_RIGHT;
    return MANUAL_JOG_DIR_NONE;
}

static void Task_ClampManualJogPoint(float *x, float *y)
{
    if (*x > MANUAL_JOG_SAFE_LIMIT_CM) *x = MANUAL_JOG_SAFE_LIMIT_CM;
    if (*x < -MANUAL_JOG_SAFE_LIMIT_CM) *x = -MANUAL_JOG_SAFE_LIMIT_CM;
    if (*y > MANUAL_JOG_SAFE_LIMIT_CM) *y = MANUAL_JOG_SAFE_LIMIT_CM;
    if (*y < -MANUAL_JOG_SAFE_LIMIT_CM) *y = -MANUAL_JOG_SAFE_LIMIT_CM;
}

static float Task_ManualJogRunCm(void)
{
    float ax = fabsf(s_manual_x);
    float ay = fabsf(s_manual_y);
    float edge = (ax > ay) ? ax : ay;

    if (edge >= MANUAL_JOG_CORNER_CM) {
        return MANUAL_JOG_CORNER_RUN_CM;
    }
    if (edge >= MANUAL_JOG_EDGE_CM) {
        return MANUAL_JOG_EDGE_RUN_CM;
    }
    return MANUAL_JOG_RUN_CM;
}

static void Task_ManualJogStop(void)
{
    if (s_manual_jog_active != 0U || s_manual_jog_dir != MANUAL_JOG_DIR_NONE) {
        mb_result_t last = MB_OK;
        uint32_t now = HAL_GetTick();

        if (s_manual_jog_active != 0U &&
            (int32_t)(now - s_manual_jog_done_tick) < 0) {
            for (uint8_t id = 1U; id <= 4U; id++) {
                mb_result_t r = motor_stop(id, MB_SYNC_NOW);
                if (r != MB_OK) {
                    last = r;
                }
                HAL_Delay(12);
            }
            printf("MANUAL jog stop: %s\r\n", motor_result_str(last));
        } else {
            printf("MANUAL jog stop: already complete\r\n");
        }
    }

    s_manual_jog_active = 0U;
    s_manual_jog_dir = MANUAL_JOG_DIR_NONE;
    s_manual_jog_tick = HAL_GetTick();
    s_manual_jog_done_tick = s_manual_jog_tick;
    s_manual_jog_run_start_tick = s_manual_jog_tick;
    s_manual_jog_run_duration_ms = 0U;
    s_manual_jog_start_x = s_manual_x;
    s_manual_jog_start_y = s_manual_y;
    s_manual_jog_target_x = s_manual_x;
    s_manual_jog_target_y = s_manual_y;
}

static void Task_ManualJogStopForce(void)
{
    if (s_manual_jog_active != 0U || s_manual_jog_dir != MANUAL_JOG_DIR_NONE) {
        mb_result_t last = MB_OK;

        for (uint8_t id = 1U; id <= 4U; id++) {
            mb_result_t r = motor_stop(id, MB_SYNC_NOW);
            if (r != MB_OK) {
                last = r;
            }
            HAL_Delay(12);
        }
        printf("MANUAL jog force stop: %s\r\n", motor_result_str(last));
    }

    s_manual_jog_active = 0U;
    s_manual_jog_dir = MANUAL_JOG_DIR_NONE;
    s_manual_jog_tick = HAL_GetTick();
    s_manual_jog_done_tick = s_manual_jog_tick;
    s_manual_jog_run_start_tick = s_manual_jog_tick;
    s_manual_jog_run_duration_ms = 0U;
    s_manual_jog_start_x = s_manual_x;
    s_manual_jog_start_y = s_manual_y;
    s_manual_jog_target_x = s_manual_x;
    s_manual_jog_target_y = s_manual_y;
}

static void Task_ManualJogIntegrate(uint32_t now)
{
    uint32_t elapsed;
    float ratio;
    float x_new;
    float y_new;

    if (s_manual_jog_active == 0U || s_manual_jog_run_duration_ms == 0U) {
        return;
    }

    elapsed = now - s_manual_jog_run_start_tick;
    if (elapsed >= s_manual_jog_run_duration_ms) {
        s_manual_x = s_manual_jog_target_x;
        s_manual_y = s_manual_jog_target_y;
        Task_ShowManualJogThrottled();
        return;
    }

    ratio = (float)elapsed / (float)s_manual_jog_run_duration_ms;
    x_new = s_manual_jog_start_x +
            (s_manual_jog_target_x - s_manual_jog_start_x) * ratio;
    y_new = s_manual_jog_start_y +
            (s_manual_jog_target_y - s_manual_jog_start_y) * ratio;
    Task_ClampManualJogPoint(&x_new, &y_new);

    s_manual_x = x_new;
    s_manual_y = y_new;
    Task_ShowManualJogThrottled();
}

static uint8_t Task_ManualJogCommand(ManualJogDir_t dir, uint8_t force)
{
    uint32_t now = HAL_GetTick();
    float cmd_x = s_manual_x;
    float cmd_y = s_manual_y;
    float soft_x = s_manual_x;
    float soft_y = s_manual_y;
    float run_cm = Task_ManualJogRunCm();
    uint32_t move_ms = 0U;
    mb_result_t r;

    if (dir == MANUAL_JOG_DIR_NONE) {
        Task_ManualJogStop();
        s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;
        return 1U;
    }

    if (dir == s_manual_limit_block_dir) {
        return 1U;
    }

    if (force == 0U &&
        s_manual_jog_active != 0U &&
        dir == s_manual_jog_dir &&
        (int32_t)(now - s_manual_jog_done_tick) < 0) {
        return 1U;
    }

    if (force != 0U && s_manual_jog_active != 0U && dir != s_manual_jog_dir) {
        Task_ManualJogStopForce();
    }

    switch (dir) {
    case MANUAL_JOG_DIR_UP:
        cmd_y += run_cm;
        soft_y += run_cm;
        break;
    case MANUAL_JOG_DIR_DOWN:
        cmd_y -= run_cm;
        soft_y -= run_cm;
        break;
    case MANUAL_JOG_DIR_LEFT:
        cmd_x -= run_cm;
        cmd_y += run_cm * MANUAL_JOG_X_UP_COMP;
        soft_x -= run_cm;
        break;
    case MANUAL_JOG_DIR_RIGHT:
        cmd_x += run_cm;
        cmd_y += run_cm * MANUAL_JOG_X_UP_COMP;
        soft_x += run_cm;
        break;
    default:
        return 1U;
    }

    {
        float before_soft_x = soft_x;
        float before_soft_y = soft_y;
        float before_cmd_x = cmd_x;
        float before_cmd_y = cmd_y;

        Task_ClampManualJogPoint(&soft_x, &soft_y);
        Task_ClampManualJogPoint(&cmd_x, &cmd_y);
        if (fabsf(soft_x - s_manual_x) < 0.01f &&
            fabsf(soft_y - s_manual_y) < 0.01f) {
            printf("MANUAL jog safe limit dir=%s soft=(%.2f,%.2f) limit=%.1f\r\n",
                   Task_ManualJogDirName(dir),
                   (double)s_manual_x, (double)s_manual_y,
                   (double)MANUAL_JOG_SAFE_LIMIT_CM);
            s_manual_limit_block_dir = dir;
            Task_ManualJogStop();
            return 1U;
        }
        if (fabsf(before_soft_x - soft_x) > 0.001f ||
            fabsf(before_soft_y - soft_y) > 0.001f ||
            fabsf(before_cmd_x - cmd_x) > 0.001f ||
            fabsf(before_cmd_y - cmd_y) > 0.001f) {
            printf("MANUAL jog clamp dir=%s soft=(%.2f,%.2f)->(%.2f,%.2f) cmd=(%.2f,%.2f)->(%.2f,%.2f) limit=%.1f\r\n",
                   Task_ManualJogDirName(dir),
                   (double)before_soft_x, (double)before_soft_y,
                   (double)soft_x, (double)soft_y,
                   (double)before_cmd_x, (double)before_cmd_y,
                   (double)cmd_x, (double)cmd_y,
                   (double)MANUAL_JOG_SAFE_LIMIT_CM);
        }
    }

    r = motion_jog_step_start(s_manual_x, s_manual_y, cmd_x, cmd_y,
                              MANUAL_JOG_TAKEUP_GAIN,
                              MANUAL_JOG_PAYOUT_GAIN,
                              &move_ms);
    if (r != MB_OK) {
        OLED_Clear();
        OLED_ShowLine(0, "JOG FAIL");
        OLED_ShowLine(2, "CHECK BUS/TENSION");
        OLED_Refresh();
        printf("MANUAL jog step fail dir=%s: %s\r\n",
               Task_ManualJogDirName(dir), motor_result_str(r));
        s_manual_jog_active = 0U;
        s_manual_jog_dir = MANUAL_JOG_DIR_NONE;
        s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;
        return 0U;
    }

    if (move_ms < 80U) {
        move_ms = 80U;
    }

    if (force != 0U || s_manual_jog_active == 0U || dir != s_manual_jog_dir) {
        printf("MANUAL jog start dir=%s run=%.2f gain(T=%.2f P=%.2f) x_comp=%.2f limit=%.1f\r\n",
               Task_ManualJogDirName(dir),
               (double)run_cm,
               (double)MANUAL_JOG_TAKEUP_GAIN,
               (double)MANUAL_JOG_PAYOUT_GAIN,
               (double)MANUAL_JOG_X_UP_COMP,
               (double)MANUAL_JOG_SAFE_LIMIT_CM);
    }

    s_manual_jog_active = 1U;
    s_manual_jog_dir = dir;
    s_manual_jog_tick = HAL_GetTick();
    s_manual_jog_run_start_tick = s_manual_jog_tick;
    s_manual_jog_run_duration_ms = move_ms;
    s_manual_jog_start_x = s_manual_x;
    s_manual_jog_start_y = s_manual_y;
    s_manual_jog_target_x = soft_x;
    s_manual_jog_target_y = soft_y;
    s_manual_jog_done_tick = s_manual_jog_tick + move_ms + MANUAL_JOG_GAP_MS;
    s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;
    Task_ShowManualJogThrottled();
    return 1U;
}

static mb_result_t Task_ManualSendMotorDelta(const int32_t delta[4])
{
    uint32_t max_mag = 0U;

    for (uint8_t i = 0U; i < 4U; i++) {
        uint32_t mag = (uint32_t)Task_AbsI32(delta[i]);
        if (mag > max_mag) {
            max_mag = mag;
        }
    }

    if (max_mag <= (uint32_t)MANUAL_RETURN_TOL_COUNTS) {
        return MB_OK;
    }

    printf("MANUAL return delta=[%ld,%ld,%ld,%ld] max=%lu\r\n",
           (long)delta[0], (long)delta[1], (long)delta[2], (long)delta[3],
           (unsigned long)max_mag);

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t id = (uint8_t)(i + 1U);
        uint32_t mag = (uint32_t)Task_AbsI32(delta[i]);
        uint8_t dir = (delta[i] >= 0) ? MB_DIR_CW : MB_DIR_CCW;
        float scale = (max_mag == 0U) ? 0.0f : ((float)mag / (float)max_mag);
        uint16_t vmax;
        mb_result_t r;

        if (mag == 0U) {
            continue;
        }
        if (scale < MOTION_MIN_SCALE) {
            scale = MOTION_MIN_SCALE;
        }
        vmax = (uint16_t)((float)MANUAL_RETURN_BASE_VMAX * scale);
        if (vmax < 1U) {
            vmax = 1U;
        }

        r = motor_move_pos(id, dir, vmax, mag, MB_MODE_REL_CUR, MB_SYNC_BUFFER);
        printf("  manual buf m%d dir=%s vmax=%u mag=%lu: %s\r\n",
               (int)id, (dir == MB_DIR_CW) ? "CW" : "CCW",
               (unsigned int)vmax, (unsigned long)mag,
               motor_result_str(r));
        if (r != MB_OK) {
            for (uint8_t sid = 1U; sid <= 4U; sid++) {
                (void)motor_stop(sid, MB_SYNC_NOW);
                HAL_Delay(15);
            }
            return r;
        }
        HAL_Delay(40);
    }

    {
        mb_result_t tr = motor_sync_trigger();
        uint32_t move_ms;

        printf("  manual trigger: %s\r\n", motor_result_str(tr));
        if (tr != MB_OK) {
            return tr;
        }

        move_ms = (uint32_t)(((float)max_mag / ((float)MANUAL_RETURN_BASE_VMAX * 6.0f)) * 1000.0f) + 500U;
        if (move_ms < 400U) move_ms = 400U;
        if (move_ms > 8000U) move_ms = 8000U;
        Task_DelayService(move_ms);
    }

    return MB_OK;
}

static uint8_t Task_ManualReturnZero(void)
{
    int32_t current[4] = {0};
    int32_t total_delta[4] = {0};
    uint32_t max_abs = 0U;

    if (s_manual_zero_valid == 0U) {
        printf("MANUAL return abort: zero not saved\r\n");
        return 0U;
    }

    Task_ManualJogStopForce();

    OLED_Clear();
    OLED_ShowLine(0, "RETURN MOTOR 0");
    OLED_Refresh();

    for (uint8_t id = 1U; id <= 4U; id++) {
        if (Task_ReadMotorPositionRetry(id, &current[id - 1U]) == 0U) {
            OLED_Clear();
            OLED_ShowLine(0, "RETURN READ FAIL");
            OLED_ShowLine(2, "CHECK MOTOR BUS");
            OLED_Refresh();
            return 0U;
        }
        total_delta[id - 1U] = s_manual_zero_pos[id - 1U] - current[id - 1U];
        if ((uint32_t)Task_AbsI32(total_delta[id - 1U]) > max_abs) {
            max_abs = (uint32_t)Task_AbsI32(total_delta[id - 1U]);
        }
        HAL_Delay(20);
    }

    printf("MANUAL return start cur=[%ld,%ld,%ld,%ld] zero=[%ld,%ld,%ld,%ld]\r\n",
           (long)current[0], (long)current[1], (long)current[2], (long)current[3],
           (long)s_manual_zero_pos[0], (long)s_manual_zero_pos[1],
           (long)s_manual_zero_pos[2], (long)s_manual_zero_pos[3]);

    if (max_abs <= (uint32_t)MANUAL_RETURN_TOL_COUNTS) {
        printf("MANUAL return skip: already near zero\r\n");
        s_manual_x = 0.0f;
        s_manual_y = 0.0f;
        motion_init();
        return Task_ManualAutoCenterAndRecord(0U);
    }

    printf("MANUAL return one-shot smooth max=%lu vmax=%u\r\n",
           (unsigned long)max_abs, (unsigned int)MANUAL_RETURN_BASE_VMAX);
    if (Task_ManualSendMotorDelta(total_delta) != MB_OK) {
        OLED_Clear();
        OLED_ShowLine(0, "RETURN MOVE FAIL");
        OLED_ShowLine(2, "CHECK BUS/TENSION");
        OLED_Refresh();
        return 0U;
    }

    for (uint8_t id = 1U; id <= 4U; id++) {
        int32_t pos = 0;
        if (Task_ReadMotorPositionRetry(id, &pos) != 0U) {
            printf("MANUAL return m%d pos=%ld zero=%ld err=%ld\r\n",
                   (int)id, (long)pos, (long)s_manual_zero_pos[id - 1U],
                   (long)(pos - s_manual_zero_pos[id - 1U]));
        }
        HAL_Delay(20);
    }

    s_manual_x = 0.0f;
    s_manual_y = 0.0f;
    motion_init();
    OLED_Clear();
    OLED_ShowLine(0, "RETURN MOTOR OK");
    OLED_ShowLine(2, "CAM FINE ZERO");
    OLED_Refresh();
    printf("MANUAL return motor zero done, visual fine zero follows\r\n");

    if (Task_ManualAutoCenterAndRecord(1U) == 0U) {
        printf("MANUAL return visual fine zero failed\r\n");
        return 0U;
    }

    printf("MANUAL return done: visual zero refreshed\r\n");
    return 1U;
}

static void Task_ManualZeroLoop(void)
{
    Task_ManualKeysService();

    switch (s_state) {
    case TS_MANUAL_SET_ZERO:
        if (Key0_Edge() != 0U) {
            if (Task_ManualAutoCenterAndRecord(1U) != 0U) {
                s_state = TS_MANUAL_JOG;
            } else {
                Task_DelayService(800);
                Task_ShowManualZeroPrompt();
            }
            s_key0_last = Key0_Raw();
        }
        break;

    case TS_MANUAL_JOG:
    {
        uint32_t now = HAL_GetTick();
        ManualJogDir_t dir;

        Task_ManualJogIntegrate(now);
        Task_ManualVisionLiveLog("jog");

        if (Key0_Edge() != 0U) {
            Task_ManualJogStopForce();
            s_key0_last = Key0_Raw();
            if (Task_ManualReturnZero() != 0U) {
                s_state = TS_MANUAL_JOG;
                Task_ShowManualJog();
            } else {
                s_state = TS_MANUAL_SET_ZERO;
                Task_DelayService(800);
                Task_ShowManualZeroPrompt();
            }
            break;
        }

        dir = Task_ManualReadJogDir();
        if (dir == MANUAL_JOG_DIR_NONE) {
            s_manual_limit_block_dir = MANUAL_JOG_DIR_NONE;
            Task_ManualJogStop();
        } else if (dir != s_manual_jog_dir || s_manual_jog_active == 0U) {
            (void)Task_ManualJogCommand(dir, 1U);
        } else {
            (void)Task_ManualJogCommand(dir, 0U);
        }
        break;
    }

    default:
        s_state = TS_MANUAL_SET_ZERO;
        Task_ShowManualZeroPrompt();
        break;
    }
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
#if TASK_BASIC1_ONLY
    OLED_ShowLine(0, "BASIC1 HOME");
    OLED_ShowLine(2, "KEY0: CENTER");
    OLED_ShowLine(4, "CAM READY FIRST");
#else
    OLED_ShowLine(0, "HOME REQUIRED");
    OLED_ShowLine(2, "KEY0: HOME");
    OLED_ShowLine(4, "CAM READY FIRST");
#endif
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

static uint8_t Task_MatchKnownVisionCircle(float cx, float cy, float *ref_x, float *ref_y)
{
    static const float kx[5] = {
        0.0f,
        -HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM,
        -HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM
    };
    static const float ky[5] = {
        0.0f,
        +HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM,
        -HOME_TOTAL_CORNER_CM, -HOME_TOTAL_CORNER_CM
    };

    for (uint8_t i = 0; i < 5U; i++) {
        if (fabsf(cx - kx[i]) <= HOME_CENTER_REF_TOL_CM &&
            fabsf(cy - ky[i]) <= HOME_CENTER_REF_TOL_CM) {
            if (ref_x != NULL) *ref_x = kx[i];
            if (ref_y != NULL) *ref_y = ky[i];
            return 1U;
        }
    }

    return 0U;
}

static void Task_ClampNoHomeCalcPoint(float *x, float *y)
{
    if (*x > HOME_NOHOME_CALC_LIMIT_CM) *x = HOME_NOHOME_CALC_LIMIT_CM;
    if (*x < -HOME_NOHOME_CALC_LIMIT_CM) *x = -HOME_NOHOME_CALC_LIMIT_CM;
    if (*y > HOME_NOHOME_CALC_LIMIT_CM) *y = HOME_NOHOME_CALC_LIMIT_CM;
    if (*y < -HOME_NOHOME_CALC_LIMIT_CM) *y = -HOME_NOHOME_CALC_LIMIT_CM;
}

static uint8_t Task_GetVisionGlobal(float *x, float *y, uint8_t *center_ref_seen)
{
    float err_x = 0.0f;
    float err_y = 0.0f;
    float circle_x = 0.0f;
    float circle_y = 0.0f;
    float ref_x = 0.0f;
    float ref_y = 0.0f;

    if (center_ref_seen != NULL) {
        *center_ref_seen = 0U;
    }

    if (vision_is_online(HOME_VISION_AGE_MS) == 0U) {
        return 0U;
    }

    if (vision_read_circle_error_with_ref_filtered(&err_x, &err_y,
                                                   &circle_x, &circle_y) != 0U) {
        if (Task_MatchKnownVisionCircle(circle_x, circle_y, &ref_x, &ref_y) != 0U) {
            *x = ref_x + err_x;
            *y = ref_y + err_y;
            if (center_ref_seen != NULL &&
                fabsf(ref_x) <= 0.1f && fabsf(ref_y) <= 0.1f) {
                *center_ref_seen = 1U;
            }
            printf("ROUGH src=CIRCLE_GLOBAL ref=(%.1f,%.1f) err=(%.2f,%.2f) x=%.2f y=%.2f\r\n",
                   (double)ref_x, (double)ref_y,
                   (double)err_x, (double)err_y,
                   (double)*x, (double)*y);
            return 1U;
        }
    }

    if (vision_read_filtered(x, y) != 0U) {
        printf("ROUGH src=LASER_GLOBAL x=%.2f y=%.2f\r\n", (double)*x, (double)*y);
        return 1U;
    }

    return 0U;
}

static uint8_t Task_GetVisionLaser(float *x, float *y)
{
    float err_x = 0.0f;
    float err_y = 0.0f;
    float circle_x = 0.0f;
    float circle_y = 0.0f;
    float ref_x = 0.0f;
    float ref_y = 0.0f;

    if (vision_is_online(HOME_VISION_AGE_MS) == 0U) {
        return 0U;
    }
    if (vision_read_home_filtered(x, y) != 0U) {
        printf("HOME src=HOME_ERR x=%.2f y=%.2f\r\n", (double)*x, (double)*y);
        return 1U;
    }
    if (vision_read_circle_error_with_ref_filtered(&err_x, &err_y,
                                                   &circle_x, &circle_y) != 0U) {
        if (Task_MatchKnownVisionCircle(circle_x, circle_y, &ref_x, &ref_y) != 0U &&
            (fabsf(ref_x) > 0.1f || fabsf(ref_y) > 0.1f)) {
            printf("HOME fine reject: current circle ref=(%.1f,%.1f), need center circle\r\n",
                   (double)ref_x, (double)ref_y);
            return 0U;
        }
        *x = err_x;
        *y = err_y;
        printf("HOME src=LASER-CIRCLE x=%.2f y=%.2f\r\n", (double)*x, (double)*y);
        return 1U;
    }
    printf("HOME fine abort: no center/local circle error\r\n");
    return 0U;
}

static uint8_t Task_RoughCenter_ByGlobal(void)
{
    float fine_x = 0.0f;
    float fine_y = 0.0f;
    float xg = 0.0f;
    float yg = 0.0f;
    float x0 = 0.0f;
    float y0 = 0.0f;
    float dist = 0.0f;
    float calc_dist = 0.0f;
    float target_x = 0.0f;
    float target_y = 0.0f;
    float move_dist = 0.0f;
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    uint8_t center_seen = 0U;
    uint8_t steps = 0U;

    if (Task_GetVisionLaser(&fine_x, &fine_y) != 0U) {
        float fine_dist = sqrtf(fine_x * fine_x + fine_y * fine_y);
        if (fine_dist <= HOME_FINE_DIRECT_CM) {
            printf("ROUGH skip: fine target visible d=%.2f\r\n", (double)fine_dist);
            return 1U;
        }
    }

    OLED_Clear();
    OLED_ShowLine(0, "ROUGH CENTER");
    OLED_Refresh();

    if (Task_GetVisionGlobal(&xg, &yg, &center_seen) == 0U) {
        printf("ROUGH abort: no global vision\r\n");
        return 0U;
    }

    dist = sqrtf(xg * xg + yg * yg);
    printf("ROUGH global x=%.2f y=%.2f d=%.2f center=%d\r\n",
           (double)xg, (double)yg, (double)dist, (int)center_seen);

    if (dist <= HOME_ROUGH_DONE_CM) {
        printf("ROUGH done: already near center circle\r\n");
        return 1U;
    }

    x0 = xg;
    y0 = yg;
    Task_ClampNoHomeCalcPoint(&x0, &y0);
    calc_dist = sqrtf(x0 * x0 + y0 * y0);
    if (calc_dist <= HOME_ROUGH_DONE_CM) {
        printf("ROUGH done by calc point\r\n");
        return 1U;
    }

    target_x = (x0 / calc_dist) * HOME_ROUGH_DONE_CM;
    target_y = (y0 / calc_dist) * HOME_ROUGH_DONE_CM;
    move_dist = calc_dist - HOME_ROUGH_DONE_CM;
    steps = (uint8_t)(move_dist / HOME_ROUGH_STEP_CM);
    if (((float)steps * HOME_ROUGH_STEP_CM) < move_dist) {
        steps++;
    }
    if (steps < 1U) {
        steps = 1U;
    }
    if (steps > HOME_ROUGH_MAX_ITER) {
        steps = HOME_ROUGH_MAX_ITER;
    }

    printf("ROUGH open start calc=(%.2f,%.2f) target=(%.2f,%.2f) move=%.2f steps=%d\r\n",
           (double)x0, (double)y0,
           (double)target_x, (double)target_y,
           (double)move_dist, (int)steps);

    prev_x = x0;
    prev_y = y0;
    for (uint8_t k = 0; k < steps; k++) {
        float t = (float)(k + 1U) / (float)steps;
        float xn = x0 + (target_x - x0) * t;
        float yn = y0 + (target_y - y0) * t;

        printf("ROUGH open cmd k=%d from=(%.2f,%.2f)->(%.2f,%.2f)\r\n",
               (int)k, (double)prev_x, (double)prev_y, (double)xn, (double)yn);

        if (motion_move_between_nohome_relaxed(prev_x, prev_y, xn, yn,
                                               HOME_RELAX_TAKEUP_GAIN,
                                               HOME_RELAX_PAYOUT_GAIN) != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "ROUGH MOVE FAIL");
            OLED_ShowLine(2, "KEY0: RETRY");
            OLED_Refresh();
            return 0U;
        }

        prev_x = xn;
        prev_y = yn;
        Task_DelayService(300);
    }

    printf("ROUGH open done\r\n");
    return 1U;
}

static uint8_t Task_ReturnCenter_ByVision(void)
{
    char line[22];
    uint8_t centered = 0U;
    float last_dist = -1.0f;
    uint8_t stale_count = 0U;
    uint8_t has_last_cmd = 0U;
    float last_cmd_x = 0.0f;
    float last_cmd_y = 0.0f;
    float last_tgt_x = 0.0f;
    float last_tgt_y = 0.0f;

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

    if (Task_RoughCenter_ByGlobal() == 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "ROUGH FAIL");
        OLED_ShowLine(2, "KEY0: RETRY");
        OLED_Refresh();
        return 0U;
    }

    OLED_Clear();
    OLED_ShowLine(0, "FINE CENTER");
    OLED_Refresh();
    Task_DelayService(200);

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

        if (last_dist >= 0.0f) {
            float improve = last_dist - dist;
            if (has_last_cmd != 0U) {
                printf("HOME obs dx=%.2f dy=%.2f  exp dx=%.2f dy=%.2f\r\n",
                       (double)(xm - last_cmd_x),
                       (double)(ym - last_cmd_y),
                       (double)(last_tgt_x - last_cmd_x),
                       (double)(last_tgt_y - last_cmd_y));
            }
            printf("HOME improve=%.2f last=%.2f now=%.2f\r\n",
                   (double)improve, (double)last_dist, (double)dist);

            if (dist > (last_dist + HOME_DIVERGE_CM)) {
                OLED_Clear();
                OLED_ShowLine(0, "HOME DIVERGE");
                OLED_ShowLine(2, "CHECK DIR/CAM");
                OLED_Refresh();
                printf("HOME abort: diverging, check motor direction or vision axis\r\n");
                return 0U;
            }

            if ((dist > HOME_OK_CM) && (improve < HOME_MIN_IMPROVE_CM)) {
                stale_count++;
                printf("HOME stale=%d/%d\r\n", (int)stale_count, (int)HOME_MAX_STALE);
                if (stale_count >= HOME_MAX_STALE) {
                    OLED_Clear();
                    OLED_ShowLine(0, "HOME NO GAIN");
                    OLED_ShowLine(2, "CHECK ROPE/ID");
                    OLED_Refresh();
                    printf("HOME abort: motors reached command but vision did not converge\r\n");
                    printf("HOME check: rope tension/slip, motor corner ID mapping, and LASER coordinate source\r\n");
                    return 0U;
                }
            } else {
                stale_count = 0U;
            }
        }
        last_dist = dist;

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

        printf("HOME cmd k=%d target=(%.2f,%.2f) step=%.2f\r\n",
               (int)k, (double)xn, (double)yn, (double)step);
        last_cmd_x = xm;
        last_cmd_y = ym;
        last_tgt_x = xn;
        last_tgt_y = yn;
        has_last_cmd = 1U;

        mb_result_t move_result;
        if (dist > HOME_RELAX_FINE_CM) {
            printf("HOME move mode=RELAX gain(T=%.2f P=%.2f)\r\n",
                   (double)HOME_RELAX_TAKEUP_GAIN,
                   (double)HOME_RELAX_PAYOUT_GAIN);
            move_result = motion_move_between_nohome_relaxed(xm, ym, xn, yn,
                                                             HOME_RELAX_TAKEUP_GAIN,
                                                             HOME_RELAX_PAYOUT_GAIN);
        } else {
            printf("HOME move mode=EXACT\r\n");
            move_result = motion_move_between_nohome(xm, ym, xn, yn);
        }

        if (move_result != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "HOME MOVE FAIL");
            OLED_ShowLine(2, "KEY0: RETRY");
            OLED_Refresh();
            return 0U;
        }

        Task_DelayService(300);
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
#if TASK_MANUAL_ZERO_MODE
    s_state = TS_MANUAL_SET_ZERO;
    s_last_menu_tick = HAL_GetTick();
    s_key0_last = Key0_Raw();
    Task_ManualKeysInit();
    Task_ShowManualZeroPrompt();

    printf("\r\nCDPR manual zero mode\r\n");
    printf("Place gondola near center, press KEY0: local visual home -> motor zero -> manual jog.\r\n");
    printf("PC0/PC1/PC2/PC3 jog after zero, KEY0 returns to saved motor zero.\r\n");
#else
    s_state = TS_HOME;
    s_last_menu_tick = HAL_GetTick();
    s_key0_last = Key0_Raw();
    Task_ShowHomePrompt();

    printf("\r\nCDPR ready: press KEY0 to visual home\r\n");
#endif
}

void Task_Loop(void)
{
    Task_FireService();

#if TASK_MANUAL_ZERO_MODE
    Task_ManualZeroLoop();
    return;
#endif

    switch (s_state) {
    case TS_HOME:
        if (Key0_Edge() != 0U) {
            if (Task_ReturnCenter_ByVision() != 0U) {
#if TASK_BASIC1_ONLY
                OLED_Clear();
                OLED_ShowLine(0, "BASIC1 DONE");
                OLED_ShowLine(2, "CENTER OK");
                OLED_ShowLine(4, "KEY0: RETRY");
                Task_DrawFireRecords();
                OLED_Refresh();
                s_state = TS_DONE;
                s_key0_last = Key0_Raw();
#else
                RouteMenu_Init();
                s_state = TS_MENU;
                s_last_menu_tick = HAL_GetTick();
                s_key0_last = Key0_Raw();
#endif
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
