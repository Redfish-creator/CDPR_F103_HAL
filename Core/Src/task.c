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
#define MANUAL_RETURN_ENCODER_SUSPECT_COUNTS 5000U
#define MANUAL_RETURN_VISUAL_NEAR_CM 8.0f
#define MANUAL_KEY_PERIOD_MS      10U
#define MANUAL_JOG_RUN_CM         4.0f
#define MANUAL_JOG_SAFE_LIMIT_CM  20.0f
/*
 * Keep hand motion neutral after the 1% loose trial made the first keyboard
 * moves visibly slack.  The former 1.04/0.98 bias accumulated extra take-up;
 * this neutral setting is not a measured tension model.
 */
#define MANUAL_JOG_TAKEUP_GAIN    1.00f
#define MANUAL_JOG_PAYOUT_GAIN    1.00f
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

#define EDGE_PATROL_START_PORT    GPIOA
#define EDGE_PATROL_START_PIN     GPIO_PIN_0
#define EDGE_PATROL_TARGET_CM     16.0f
#define EDGE_PATROL_APPROACH_CM   13.0f
#define EDGE_PATROL_MID_CM        10.5f
#define EDGE_PATROL_LT_X          -16.0f
#define EDGE_PATROL_LT_Y          15.0f
#define EDGE_PATROL_RT_X          16.0f
#define EDGE_PATROL_RT_Y          17.5f
#define EDGE_PATROL_RB_X          16.0f
#define EDGE_PATROL_RB_Y          -15.0f
#define EDGE_PATROL_LB_X          -16.0f
#define EDGE_PATROL_LB_Y          -15.0f
#define EDGE_PATROL_SEG_MAX_CM    2.0f
#define EDGE_PATROL_DWELL_MS      700U
#define EDGE_PATROL_SEG_SETTLE_MS 160U
#define EDGE_PATROL_OK_CM         0.8f
#define EDGE_PATROL_TAKEUP_GAIN   0.99f
#define EDGE_PATROL_PAYOUT_GAIN   1.01f
#define EDGE_PATROL_MAX_RELEASE_CM 2.0f
#define EDGE_PATROL_TOP_RELEASE_CM 1.2f
#define EDGE_PATROL_FINAL_RELEASE_CM 1.4f
#define EDGE_PATROL_MAX_TAKEUP_CM 3.2f
#define EDGE_PATROL_MAX_SUBSTEPS  24U
#define EDGE_PATROL_FINE_STEP_CM  0.45f
#define EDGE_PATROL_FINE_MAX_ITER 12U
#define EDGE_PATROL_TILT_SLOW     0.80f
#define EDGE_PATROL_SOFT_CENTER_CM 0.6f
#define EDGE_PATROL_CAL_USE_MIN   1U
#define EDGE_PATROL_CAL_MAX_COUNT 30U
#define EDGE_PATROL_NODE_LOG      1U
#define EDGE_PATROL_SEGMENT_LOG   1U
#define EDGE_PATROL_SEGMENT_READ_GAP_MS 80U
#define EDGE_PATROL_POST_READ_QUIET_MS 120U
#define EDGE_PATROL_VISION_RETRY_MAX 3U
#define EDGE_PATROL_VISION_RETRY_MS  120U
#define EDGE_PATROL_CENTER_RESET_RETRY_MAX 1U
#define EDGE_PATROL_CENTER_RESET_RETRY_WAIT_MS 800U
#define TASK_M2_BUFFER_PRE_CMD_EXTRA_DELAY_MS 120U
#define TASK_M2_BUFFER_POST_CMD_EXTRA_DELAY_MS 80U

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

typedef struct {
    float x;
    float y;
    const char *name;
    uint8_t circle_id;
} EdgePatrolWaypoint_t;

typedef struct {
    uint8_t count;
    uint8_t enc_count;
    float soft_x;
    float soft_y;
    float err_x;
    float err_y;
    float tilt_x;
    float tilt_y;
    float enc[4];
} EdgePatrolCal_t;

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
static uint8_t s_manual_jog_rehome_required = 0U;
static uint8_t s_manual_jog_fault_notice_shown = 0U;
static ManualKeyFilter_t s_manual_key_up;
static ManualKeyFilter_t s_manual_key_down;
static ManualKeyFilter_t s_manual_key_left;
static ManualKeyFilter_t s_manual_key_right;
static ManualKeyFilter_t s_edge_patrol_key;
static uint8_t s_edge_patrol_key_last = 0U;
static uint8_t s_manual_return_last_skipped = 0U;
static EdgePatrolCal_t s_edge_patrol_cal[5];
static int32_t s_edge_patrol_prev_enc[4] = {0};
static uint8_t s_edge_patrol_prev_enc_mask = 0U;
static uint8_t s_edge_patrol_confirmed_mask = 0U;
static uint8_t s_edge_patrol_unconfirmed_mask = 0U;

static const EdgePatrolWaypoint_t s_edge_patrol_path[] = {
    { 0.0f, 0.0f, "CENTER", 0U },
    { -4.0f, 4.0f, "LT-IN1", 0U },
    { -8.0f, 8.0f, "LT-IN2", 0U },
    { -12.0f, 11.5f, "LT-APP", 0U },
    { EDGE_PATROL_LT_X, EDGE_PATROL_LT_Y, "LT", 1U },
    { -8.0f, 8.0f, "LT-OUT1", 0U },
    { -4.0f, 4.0f, "LT-OUT2", 0U },
    { 0.0f, 0.0f, "CENTER-LT", 0U },
    { 4.0f, 4.0f, "RT-IN1", 0U },
    { 8.0f, 8.0f, "RT-IN2", 0U },
    { 12.0f, 12.0f, "RT-APP", 0U },
    { EDGE_PATROL_RT_X, EDGE_PATROL_RT_Y, "RT", 2U },
    { 8.0f, 8.0f, "RT-OUT1", 0U },
    { 4.0f, 4.0f, "RT-OUT2", 0U },
    { 0.0f, 0.0f, "CENTER-RT", 0U },
    { 4.0f, -4.0f, "RB-IN1", 0U },
    { 8.0f, -8.0f, "RB-IN2", 0U },
    { 12.0f, -11.5f, "RB-APP", 0U },
    { EDGE_PATROL_RB_X, EDGE_PATROL_RB_Y, "RB", 3U },
    { 8.0f, -8.0f, "RB-OUT1", 0U },
    { 4.0f, -4.0f, "RB-OUT2", 0U },
    { 0.0f, 0.0f, "CENTER-RB", 0U },
    { -4.0f, -4.0f, "LB-IN1", 0U },
    { -8.0f, -8.0f, "LB-IN2", 0U },
    { -12.0f, -11.5f, "LB-APP", 0U },
    { EDGE_PATROL_LB_X, EDGE_PATROL_LB_Y, "LB", 4U },
    { -8.0f, -8.0f, "LB-OUT1", 0U },
    { -4.0f, -4.0f, "LB-OUT2", 0U },
    { 0.0f, 0.0f, "CENTER", 0U }
};

static uint8_t Key0_Raw(void);
static uint8_t Key0_Edge(void);
static uint8_t Task_ReadMotorPositionRetry(uint8_t id, int32_t *pos);
static int32_t Task_AbsI32(int32_t value);
static mb_result_t Task_MotorMovePosBufferRetry(uint8_t id, uint8_t dir,
                                                uint16_t vmax, uint32_t mag,
                                                uint8_t mode,
                                                const char *tag);
static void ManualKey_Update(ManualKeyFilter_t *kf, uint8_t raw_pressed);
static uint8_t ManualKey_IsPressed(const ManualKeyFilter_t *kf);
static void Task_ManualKeysInit(void);
static void Task_ManualKeysService(void);
static uint8_t Task_EdgePatrolKeyEdge(void);
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
static void Task_ManualJogRequireRehome(const char *where, mb_result_t result);
static void Task_ManualJogIntegrate(uint32_t now);
static void Task_ManualJogStop(void);
static void Task_ManualJogStopForce(void);
static uint8_t Task_ManualJogCommand(ManualJogDir_t dir, uint8_t force);
static mb_result_t Task_ManualSendMotorDelta(const int32_t delta[4]);
static uint8_t Task_ManualReturnZero(void);
static const char *Task_EdgePatrolCircleName(uint8_t circle_id);
static uint32_t Task_VisionAge(uint32_t now, uint32_t sample_ms);
static void Task_ShowEdgePatrol(const char *name, uint8_t step, uint8_t total,
                                float x, float y);
static float Task_EdgePatrolReleaseLimit(float x0, float y0, float x1, float y1,
                                         uint8_t circle_id);
static uint8_t Task_EdgePatrolSegmentGentleEnough(float x0, float y0,
                                                  float x1, float y1,
                                                  float release_limit);
static uint8_t Task_EdgePatrolCalcSegments(float x0, float y0,
                                           float x1, float y1,
                                           uint8_t circle_id);
static uint8_t Task_EdgePatrolCircleReached(uint8_t circle_id);
static uint8_t Task_EdgePatrolGetLearnedTarget(uint8_t circle_id,
                                               float *x, float *y);
static void Task_EdgePatrolSampleNode(uint8_t circle_id,
                                      const char *name,
                                      float err_x, float err_y);
static void Task_EdgePatrolPrintCalTable(void);
static void Task_EdgePatrolLogNode(const char *name,
                                   uint8_t step, uint8_t total,
                                   uint8_t circle_id);
static void Task_EdgePatrolPrintEncVector(const char *label,
                                          const int32_t value[4],
                                          uint8_t mask);
static uint8_t Task_EdgePatrolReadEncOnce(int32_t pos[4],
                                          mb_result_t result[4]);
static void Task_EdgePatrolResetSegmentEncoderBaseline(void);
static void Task_EdgePatrolLogSegment(const char *name,
                                      uint8_t step, uint8_t total,
                                      uint8_t segment, uint8_t segment_total,
                                      uint8_t fine,
                                      float release_limit,
                                      mb_result_t motion_result);
static uint8_t Task_EdgePatrolEnsureLaserFresh(const char *name,
                                               uint8_t step,
                                               uint8_t segment);
static uint8_t Task_EdgePatrolFineCircle(uint8_t circle_id,
                                         uint8_t waypoint_step,
                                         uint8_t total);
static uint8_t Task_EdgePatrolMoveTo(const EdgePatrolWaypoint_t *wp,
                                     uint8_t step, uint8_t total);
static uint8_t Task_EdgePatrolAutoZeroBeforeRun(void);
static uint8_t Task_EdgePatrolAutoZeroWithRecovery(const char *reason);
static uint8_t Task_EdgePatrolRun(void);
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

static uint32_t Task_VisionAge(uint32_t now, uint32_t sample_ms)
{
    return (now >= sample_ms) ? (now - sample_ms) : 0U;
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

static void Task_MotorPreBufferDelay(uint8_t id)
{
    if (id == 2U && TASK_M2_BUFFER_PRE_CMD_EXTRA_DELAY_MS != 0U) {
        Task_DelayService(TASK_M2_BUFFER_PRE_CMD_EXTRA_DELAY_MS);
    }
}

static void Task_MotorPostBufferDelay(uint8_t id)
{
    if (id == 2U && TASK_M2_BUFFER_POST_CMD_EXTRA_DELAY_MS != 0U) {
        Task_DelayService(TASK_M2_BUFFER_POST_CMD_EXTRA_DELAY_MS);
    }
}

static mb_result_t Task_MotorMovePosBufferRetry(uint8_t id, uint8_t dir,
                                                uint16_t vmax, uint32_t mag,
                                                uint8_t mode,
                                                const char *tag)
{
    mb_result_t r = MB_ERR_TIMEOUT;

    for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
        Task_MotorPreBufferDelay(id);
        r = motor_move_pos(id, dir, vmax, mag, mode, MB_SYNC_BUFFER);
        if (r == MB_OK) {
            if (attempt != 0U) {
                printf("  %s buf m%d retry ok attempt=%u\r\n",
                       (tag != NULL) ? tag : "task",
                       (int)id, (unsigned int)(attempt + 1U));
            }
            Task_MotorPostBufferDelay(id);
            return MB_OK;
        }
        printf("  %s buf m%d attempt=%u %s\r\n",
               (tag != NULL) ? tag : "task",
               (int)id, (unsigned int)(attempt + 1U),
               motor_result_str(r));

        /*
         * Relative buffered motion is not safely repeatable after a lost or
         * corrupt response: the drive may have accepted the first request.
         * Retry only HAL UART_BUSY, which is returned before this blocking
         * transfer starts. Leave the group untriggered for every ambiguous
         * result, including TX_FAIL.
         */
        if (r != MB_ERR_BUSY) {
            motor_print_last_transaction_diag("  task buffer failure");
            break;
        }
        HAL_Delay(90U);
    }

    if (r == MB_ERR_BUSY) {
        motor_print_last_transaction_diag("  task uart busy");
    }
    return r;
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
    memset(&s_edge_patrol_key, 0, sizeof(s_edge_patrol_key));
    s_edge_patrol_key_last = 0U;
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
    ManualKey_Update(&s_edge_patrol_key,
                     (HAL_GPIO_ReadPin(EDGE_PATROL_START_PORT,
                                       EDGE_PATROL_START_PIN) == GPIO_PIN_SET) ? 1U : 0U);
}

static uint8_t Task_EdgePatrolKeyEdge(void)
{
    uint8_t pressed = ManualKey_IsPressed(&s_edge_patrol_key);
    uint8_t edge = (pressed != 0U && s_edge_patrol_key_last == 0U) ? 1U : 0U;

    s_edge_patrol_key_last = pressed;
    return edge;
}

static void Task_ShowManualZeroPrompt(void)
{
    OLED_Clear();
    OLED_ShowLine(0, "Q2 PATROL PREP");
    OLED_ShowLine(2, "PLACE NEAR 0");
    OLED_ShowLine(4, "KEY0: ZERO FIRST");
    OLED_ShowLine(6, "PA0 AFTER ZERO");
    OLED_Refresh();
}

static void Task_ShowManualJog(void)
{
    char line[22];

    OLED_Clear();
    if (s_manual_jog_rehome_required != 0U) {
        OLED_ShowLine(0, "JOG BUS UNCERT");
        OLED_ShowLine(2, "KEY0: REHOME");
        OLED_ShowLine(4, "PA0: REHOME+PAT");
        OLED_ShowLine(6, "PC0-3 LOCKED");
        OLED_Refresh();
        s_manual_oled_tick = HAL_GetTick();
        return;
    }
    OLED_ShowLine(0, "Q2 READY");
    snprintf(line, sizeof(line), "X:%5.1f Y:%5.1f", (double)s_manual_x, (double)s_manual_y);
    OLED_ShowLine(1, line);
    OLED_ShowLine(2, "PC0 UP  PC1 DOWN");
    OLED_ShowLine(3, "PC2 LEFT PC3 RIGHT");
    OLED_ShowLine(5, "PA0: PATROL4");
    OLED_ShowLine(6, "KEY0: RETURN 0");
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
               (unsigned long)Task_VisionAge(now, g_vision.home_ms));
    }
    if (g_vision.laser_count != 0U) {
        printf(" LASER cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.laser_count,
               (double)g_vision.laser_x,
               (double)g_vision.laser_y,
               (unsigned long)Task_VisionAge(now, g_vision.laser_ms));
    }
    if (g_vision.circle_count != 0U) {
        printf(" CIRCLE cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.circle_count,
               (double)g_vision.circle_x,
               (double)g_vision.circle_y,
               (unsigned long)Task_VisionAge(now, g_vision.circle_ms));
    }
    if (g_vision.tilt_count != 0U) {
        printf(" TILT cnt=%lu x=%.2f y=%.2f age=%lums",
               (unsigned long)g_vision.tilt_count,
               (double)g_vision.tilt_x,
               (double)g_vision.tilt_y,
               (unsigned long)Task_VisionAge(now, g_vision.tilt_ms));
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
        mb_result_t r = Task_MotorMovePosBufferRetry(id,
                                                     dir,
                                                     MANUAL_CENTER_TENSION_VMAX,
                                                     MANUAL_CENTER_TENSION_COUNTS,
                                                     MB_MODE_REL_CUR,
                                                     "tension");
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
    uint8_t recovering_jog = s_manual_jog_rehome_required;

    if (vision_send_mode("initial") == 0U) {
        printf("MANUAL auto home warn: &mode,initial# send failed\r\n");
    } else {
        printf("MANUAL auto home cmd: &mode,initial# sent\r\n");
    }
    Task_DelayService(350U);

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
            printf("MANUAL auto home move fail: %s abort; "
                   "relative buffer state uncertain\r\n",
                   motor_result_str(r));
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
    s_manual_jog_rehome_required = 0U;
    s_manual_jog_fault_notice_shown = 0U;

    OLED_Clear();
    OLED_ShowLine(0, "Q2 READY");
    OLED_ShowLine(2, "ZERO SAVED");
    OLED_ShowLine(4, "PA0: PATROL4");
    OLED_ShowLine(6, "PC0-3 JOG");
    OLED_Refresh();
    if (recovering_jog != 0U) {
        printf("MANUAL recovery complete: KEY0 rehome OK; PC0-PC3 unlocked\r\n");
    }
    printf("Q2 ready: center zero saved. Press PA0 to patrol four orange edge circles; PC0-PC3 jog; KEY0 return zero.\r\n");
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
    return MANUAL_JOG_RUN_CM;
}

static void Task_ManualJogRequireRehome(const char *where, mb_result_t result)
{
    s_manual_jog_rehome_required = 1U;
    s_manual_jog_fault_notice_shown = 1U;
    OLED_Clear();
    OLED_ShowLine(0, "JOG BUS UNCERT");
    OLED_ShowLine(2, "KEY0: REHOME");
    OLED_ShowLine(4, "PA0: REHOME+PAT");
    OLED_ShowLine(6, "PC0-3 LOCKED");
    OLED_Refresh();
    printf("MANUAL jog bus state uncertain at %s: %s; "
           "PC0-PC3 locked until KEY0 rehome\r\n",
           (where != NULL) ? where : "unknown",
           motor_result_str(result));
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
            if (last != MB_OK) {
                Task_ManualJogRequireRehome("stop", last);
            }
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
        if (last != MB_OK) {
            Task_ManualJogRequireRehome("force-stop", last);
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

    if (s_manual_jog_rehome_required != 0U) {
        if (s_manual_jog_fault_notice_shown == 0U) {
            s_manual_jog_fault_notice_shown = 1U;
            OLED_Clear();
            OLED_ShowLine(0, "JOG BUS UNCERT");
            OLED_ShowLine(2, "KEY0: REHOME");
            OLED_ShowLine(4, "PA0: REHOME+PAT");
            OLED_ShowLine(6, "PC0-3 LOCKED");
            OLED_Refresh();
            printf("MANUAL jog blocked: bus state uncertain; "
                   "press KEY0 to rehome\r\n");
        }
        return 0U;
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
        if (s_manual_jog_rehome_required != 0U) {
            return 0U;
        }
    }

    switch (dir) {
    case MANUAL_JOG_DIR_UP:
        if (run_cm > (MANUAL_JOG_SAFE_LIMIT_CM - s_manual_y)) {
            run_cm = MANUAL_JOG_SAFE_LIMIT_CM - s_manual_y;
        }
        break;
    case MANUAL_JOG_DIR_DOWN:
        if (run_cm > (s_manual_y + MANUAL_JOG_SAFE_LIMIT_CM)) {
            run_cm = s_manual_y + MANUAL_JOG_SAFE_LIMIT_CM;
        }
        break;
    case MANUAL_JOG_DIR_LEFT:
        if (run_cm > (s_manual_x + MANUAL_JOG_SAFE_LIMIT_CM)) {
            run_cm = s_manual_x + MANUAL_JOG_SAFE_LIMIT_CM;
        }
        break;
    case MANUAL_JOG_DIR_RIGHT:
        if (run_cm > (MANUAL_JOG_SAFE_LIMIT_CM - s_manual_x)) {
            run_cm = MANUAL_JOG_SAFE_LIMIT_CM - s_manual_x;
        }
        break;
    default:
        return 1U;
    }
    if (run_cm < 0.0f) {
        run_cm = 0.0f;
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
        s_manual_limit_block_dir = dir;
        Task_ManualJogRequireRehome("step-start", r);
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

        r = Task_MotorMovePosBufferRetry(id, dir, vmax, mag,
                                         MB_MODE_REL_CUR, "manual");
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

    s_manual_return_last_skipped = 0U;
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
        printf("MANUAL return skip: already near zero, visual fine zero follows\r\n");
        s_manual_return_last_skipped = 1U;
        s_manual_x = 0.0f;
        s_manual_y = 0.0f;
        motion_init();
        /*
         * A communication fault can occur after no motor displacement (for
         * example, while an acknowledgement is lost).  Do not leave the
         * rehome latch set merely because the encoder delta is small: KEY0
         * must still verify the visual center before unlocking PC0-PC3.
         * Avoid another blanket tension pulse here; the caller already has
         * the known motor-zero reference and the field has an over-tension
         * risk.
         */
        return Task_ManualAutoCenterAndRecord(0U);
    }

    if (max_abs >= (uint32_t)MANUAL_RETURN_ENCODER_SUSPECT_COUNTS) {
        float vx = 0.0f;
        float vy = 0.0f;

        if (Task_GetVisionLaser(&vx, &vy) != 0U) {
            float vd = sqrtf(vx * vx + vy * vy);

            printf("MANUAL return guard: encoder max=%lu visual=(%.2f,%.2f) d=%.2f\r\n",
                   (unsigned long)max_abs, (double)vx, (double)vy, (double)vd);
            if (vd <= MANUAL_RETURN_VISUAL_NEAR_CM) {
                printf("MANUAL return guard: visual near center, skip large encoder return\r\n");
                s_manual_return_last_skipped = 1U;
                s_manual_x = 0.0f;
                s_manual_y = 0.0f;
                motion_init();
                return Task_ManualAutoCenterAndRecord(1U);
            }
        } else {
            printf("MANUAL return guard: encoder max=%lu, visual unavailable\r\n",
                   (unsigned long)max_abs);
        }
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

static const char *Task_EdgePatrolCircleName(uint8_t circle_id)
{
    switch (circle_id) {
    case 1U: return "LT";
    case 2U: return "RT";
    case 3U: return "RB";
    case 4U: return "LB";
    default: return "--";
    }
}

static void Task_ShowEdgePatrol(const char *name, uint8_t step, uint8_t total,
                                float x, float y)
{
    char line[22];

    OLED_Clear();
    OLED_ShowLine(0, "EDGE PATROL");
    snprintf(line, sizeof(line), "%s %02u/%02u",
             (name != NULL) ? name : "MOVE", (unsigned int)step, (unsigned int)total);
    OLED_ShowLine(1, line);
    snprintf(line, sizeof(line), "X:%5.1f Y:%5.1f", (double)x, (double)y);
    OLED_ShowLine(3, line);
    OLED_ShowLine(6, "KEY0: ABORT");
    OLED_Refresh();
}

static float Task_EdgePatrolReleaseLimit(float x0, float y0, float x1, float y1,
                                         uint8_t circle_id)
{
    float limit = EDGE_PATROL_MAX_RELEASE_CM;
    float ax = fmaxf(fabsf(x0), fabsf(x1));
    float ay = fmaxf(fabsf(y0), fabsf(y1));

    if (y0 > 10.0f && y1 > 10.0f) {
        limit = EDGE_PATROL_TOP_RELEASE_CM;
    }
    if (circle_id != 0U || (ax > 13.0f && ay > 13.0f)) {
        if (EDGE_PATROL_FINAL_RELEASE_CM < limit) {
            limit = EDGE_PATROL_FINAL_RELEASE_CM;
        }
    }
    return limit;
}

static uint8_t Task_EdgePatrolSegmentGentleEnough(float x0, float y0,
                                                  float x1, float y1,
                                                  float release_limit)
{
    float L0[4];
    float L1[4];

    if (kin_in_range(x0, y0) == 0U || kin_in_range(x1, y1) == 0U) {
        return 0U;
    }

    kin_ik(x0, y0, L0);
    kin_ik(x1, y1, L1);

    for (uint8_t i = 0U; i < 4U; i++) {
        float d = L1[i] - L0[i];
        if (d > release_limit) {
            return 0U;
        }
        if ((-d) > EDGE_PATROL_MAX_TAKEUP_CM) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t Task_EdgePatrolCalcSegments(float x0, float y0,
                                           float x1, float y1,
                                           uint8_t circle_id)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float dist = sqrtf(dx * dx + dy * dy);
    uint8_t segments = (uint8_t)(dist / EDGE_PATROL_SEG_MAX_CM);

    if (((float)segments * EDGE_PATROL_SEG_MAX_CM) < dist) {
        segments++;
    }
    if (segments < 1U) {
        segments = 1U;
    }

    while (segments < EDGE_PATROL_MAX_SUBSTEPS) {
        uint8_t ok = 1U;
        for (uint8_t k = 0U; k < segments; k++) {
            float t0 = (float)k / (float)segments;
            float t1 = (float)(k + 1U) / (float)segments;
            float sx0 = x0 + dx * t0;
            float sy0 = y0 + dy * t0;
            float sx1 = x0 + dx * t1;
            float sy1 = y0 + dy * t1;
            float release_limit = Task_EdgePatrolReleaseLimit(sx0, sy0, sx1, sy1, circle_id);

            if (Task_EdgePatrolSegmentGentleEnough(sx0, sy0, sx1, sy1,
                                                   release_limit) == 0U) {
                ok = 0U;
                break;
            }
        }
        if (ok != 0U) {
            return segments;
        }
        segments++;
    }

    printf("PATROL warn: segment limit reached, using %u substeps\r\n",
           (unsigned int)EDGE_PATROL_MAX_SUBSTEPS);
    return EDGE_PATROL_MAX_SUBSTEPS;
}

static uint8_t Task_EdgePatrolCircleReached(uint8_t circle_id)
{
    static const float ref_x[4] = {
        -HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM,
        +HOME_TOTAL_CORNER_CM, -HOME_TOTAL_CORNER_CM
    };
    static const float ref_y[4] = {
        +HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM,
        -HOME_TOTAL_CORNER_CM, -HOME_TOTAL_CORNER_CM
    };
    float err_x = 0.0f;
    float err_y = 0.0f;
    float circle_x = 0.0f;
    float circle_y = 0.0f;
    float matched_x = 0.0f;
    float matched_y = 0.0f;
    float dist;
    uint32_t now = HAL_GetTick();
    uint8_t idx;

    if (circle_id == 0U || circle_id > 4U) {
        return 0U;
    }
    idx = (uint8_t)(circle_id - 1U);

    if (vision_is_online(HOME_VISION_AGE_MS) == 0U ||
        vision_read_circle_error_with_ref_filtered(&err_x, &err_y,
                                                   &circle_x, &circle_y) == 0U) {
        printf("PATROL circle %s vision confirm unavailable\r\n",
               Task_EdgePatrolCircleName(circle_id));
        return 0U;
    }

    if (Task_MatchKnownVisionCircle(circle_x, circle_y, &matched_x, &matched_y) == 0U ||
        fabsf(matched_x - ref_x[idx]) > 0.1f ||
        fabsf(matched_y - ref_y[idx]) > 0.1f) {
        printf("PATROL circle %s seen_other circle=(%.1f,%.1f) err=(%.2f,%.2f)\r\n",
               Task_EdgePatrolCircleName(circle_id),
               (double)circle_x, (double)circle_y,
               (double)err_x, (double)err_y);
        return 0U;
    }

    dist = sqrtf(err_x * err_x + err_y * err_y);
    printf("PATROL circle %s ref=(%.1f,%.1f) err=(%.2f,%.2f) d=%.2f",
           Task_EdgePatrolCircleName(circle_id),
           (double)matched_x, (double)matched_y,
           (double)err_x, (double)err_y,
           (double)dist);
    if (g_vision.tilt_count != 0U) {
        printf(" tilt=(%.2f,%.2f) age=%lums",
               (double)g_vision.tilt_x, (double)g_vision.tilt_y,
               (unsigned long)Task_VisionAge(now, g_vision.tilt_ms));
    }
    printf("\r\n");

    if (dist <= EDGE_PATROL_OK_CM) {
        Task_EdgePatrolSampleNode(circle_id,
                                  Task_EdgePatrolCircleName(circle_id),
                                  err_x, err_y);
        return 1U;
    }

    return 0U;
}

static uint8_t Task_EdgePatrolGetLearnedTarget(uint8_t circle_id,
                                               float *x, float *y)
{
    const EdgePatrolCal_t *cal;

    if (circle_id == 0U || circle_id > 4U || x == NULL || y == NULL) {
        return 0U;
    }

    cal = &s_edge_patrol_cal[circle_id];
    if (cal->count < EDGE_PATROL_CAL_USE_MIN) {
        return 0U;
    }
    if (kin_in_range(cal->soft_x, cal->soft_y) == 0U) {
        printf("PATROL target learned %s ignored: avg=(%.2f,%.2f) out of range\r\n",
               Task_EdgePatrolCircleName(circle_id),
               (double)cal->soft_x, (double)cal->soft_y);
        return 0U;
    }

    *x = cal->soft_x;
    *y = cal->soft_y;
    return 1U;
}

static void Task_EdgePatrolSampleNode(uint8_t circle_id,
                                      const char *name,
                                      float err_x, float err_y)
{
    EdgePatrolCal_t *cal;
    const char *label;
    int32_t pos[4] = {0};
    uint8_t enc_ok = 1U;
    uint8_t next_count;
    float denom;
    uint8_t tilt_ok = 0U;
    float tilt_x = 0.0f;
    float tilt_y = 0.0f;

    if (circle_id == 0U || circle_id > 4U) {
        return;
    }

    cal = &s_edge_patrol_cal[circle_id];
    label = (name != NULL) ? name : Task_EdgePatrolCircleName(circle_id);

    next_count = cal->count;
    if (next_count < EDGE_PATROL_CAL_MAX_COUNT) {
        next_count++;
    }
    if (next_count == 0U) {
        next_count = 1U;
    }
    denom = (float)next_count;

    cal->soft_x += (s_manual_x - cal->soft_x) / denom;
    cal->soft_y += (s_manual_y - cal->soft_y) / denom;
    cal->err_x += (err_x - cal->err_x) / denom;
    cal->err_y += (err_y - cal->err_y) / denom;
    cal->count = next_count;

    if (g_vision.tilt_count != 0U &&
        Task_VisionAge(HAL_GetTick(), g_vision.tilt_ms) <= HOME_VISION_AGE_MS) {
        tilt_ok = 1U;
        tilt_x = g_vision.tilt_x;
        tilt_y = g_vision.tilt_y;
        cal->tilt_x += (tilt_x - cal->tilt_x) / denom;
        cal->tilt_y += (tilt_y - cal->tilt_y) / denom;
    }

    for (uint8_t id = 1U; id <= 4U; id++) {
        if (Task_ReadMotorPositionRetry(id, &pos[id - 1U]) == 0U) {
            enc_ok = 0U;
        }
        HAL_Delay(20U);
    }

    if (enc_ok != 0U) {
        uint8_t next_enc_count = cal->enc_count;
        float enc_denom;

        if (next_enc_count < EDGE_PATROL_CAL_MAX_COUNT) {
            next_enc_count++;
        }
        if (next_enc_count == 0U) {
            next_enc_count = 1U;
        }
        enc_denom = (float)next_enc_count;

        for (uint8_t i = 0U; i < 4U; i++) {
            cal->enc[i] += ((float)pos[i] - cal->enc[i]) / enc_denom;
        }
        cal->enc_count = next_enc_count;
    }

    printf("PATROL CAL %s n=%u soft=(%.2f,%.2f) err=(%.2f,%.2f)",
           label, (unsigned int)cal->count,
           (double)s_manual_x, (double)s_manual_y,
           (double)err_x, (double)err_y);
    if (tilt_ok != 0U) {
        printf(" tilt=(%.2f,%.2f)", (double)tilt_x, (double)tilt_y);
    } else {
        printf(" tilt=NA");
    }
    if (enc_ok != 0U) {
        printf(" enc=[%ld,%ld,%ld,%ld] enc_n=%u",
               (long)pos[0], (long)pos[1], (long)pos[2], (long)pos[3],
               (unsigned int)cal->enc_count);
    } else {
        printf(" enc=READ_FAIL enc_n=%u",
               (unsigned int)cal->enc_count);
    }
    printf("\r\n");

    printf("PATROL AVG %s n=%u soft=(%.2f,%.2f) err=(%.2f,%.2f) tilt=(%.2f,%.2f)",
           label, (unsigned int)cal->count,
           (double)cal->soft_x, (double)cal->soft_y,
           (double)cal->err_x, (double)cal->err_y,
           (double)cal->tilt_x, (double)cal->tilt_y);
    if (cal->enc_count != 0U) {
        printf(" enc_avg=[%ld,%ld,%ld,%ld] enc_n=%u",
               (long)Task_RoundToInt(cal->enc[0]),
               (long)Task_RoundToInt(cal->enc[1]),
               (long)Task_RoundToInt(cal->enc[2]),
               (long)Task_RoundToInt(cal->enc[3]),
               (unsigned int)cal->enc_count);
    } else {
        printf(" enc_avg=NA enc_n=0");
    }
    printf("\r\n");
}

static void Task_EdgePatrolPrintCalTable(void)
{
    printf("PATROL CAL TABLE begin\r\n");
    for (uint8_t circle_id = 1U; circle_id <= 4U; circle_id++) {
        const EdgePatrolCal_t *cal = &s_edge_patrol_cal[circle_id];
        const char *name = Task_EdgePatrolCircleName(circle_id);

        if (cal->count == 0U) {
            printf("PATROL CAL TABLE %s n=0\r\n", name);
            continue;
        }

        printf("PATROL CAL TABLE %s n=%u soft_avg=(%.2f,%.2f) err_avg=(%.2f,%.2f) tilt_avg=(%.2f,%.2f)",
               name,
               (unsigned int)cal->count,
               (double)cal->soft_x, (double)cal->soft_y,
               (double)cal->err_x, (double)cal->err_y,
               (double)cal->tilt_x, (double)cal->tilt_y);
        if (cal->enc_count != 0U) {
            printf(" enc_avg=[%ld,%ld,%ld,%ld] enc_n=%u",
                   (long)Task_RoundToInt(cal->enc[0]),
                   (long)Task_RoundToInt(cal->enc[1]),
                   (long)Task_RoundToInt(cal->enc[2]),
                   (long)Task_RoundToInt(cal->enc[3]),
                   (unsigned int)cal->enc_count);
        } else {
            printf(" enc_avg=NA enc_n=0");
        }
        printf("\r\n");
    }
    printf("PATROL CAL TABLE end\r\n");
}

static void Task_EdgePatrolLogNode(const char *name,
                                   uint8_t step, uint8_t total,
                                   uint8_t circle_id)
{
#if EDGE_PATROL_NODE_LOG
    const char *label = (name != NULL) ? name : "NODE";
    uint32_t now = HAL_GetTick();
    int32_t pos[4] = {0};
    uint8_t enc_ok = 1U;
    const char *enc_source = "SEG";

    if (s_edge_patrol_prev_enc_mask == 0x0FU) {
        for (uint8_t i = 0U; i < 4U; i++) {
            pos[i] = s_edge_patrol_prev_enc[i];
        }
    } else {
        enc_source = "READ";
        for (uint8_t id = 1U; id <= 4U; id++) {
            if (Task_ReadMotorPositionRetry(id, &pos[id - 1U]) == 0U) {
                enc_ok = 0U;
            }
            HAL_Delay(20U);
        }
        if (enc_ok != 0U) {
            for (uint8_t i = 0U; i < 4U; i++) {
                s_edge_patrol_prev_enc[i] = pos[i];
            }
            s_edge_patrol_prev_enc_mask = 0x0FU;
        }
    }

    printf("PATROL NODE name=%s step=%u/%u circle=%u soft=(%.2f,%.2f)",
           label,
           (unsigned int)step, (unsigned int)total,
           (unsigned int)circle_id,
           (double)s_manual_x, (double)s_manual_y);
    if (g_vision.laser_count != 0U) {
        printf(" laser=(%.2f,%.2f,%lums)",
               (double)g_vision.laser_x, (double)g_vision.laser_y,
               (unsigned long)Task_VisionAge(now, g_vision.laser_ms));
    } else {
        printf(" laser=NA");
    }
    if (g_vision.circle_count != 0U) {
        printf(" circle=(%.2f,%.2f,%lums)",
               (double)g_vision.circle_x, (double)g_vision.circle_y,
               (unsigned long)Task_VisionAge(now, g_vision.circle_ms));
    } else {
        printf(" circle=NA");
    }
    if (g_vision.laser_count != 0U && g_vision.circle_count != 0U) {
        printf(" err=(%.2f,%.2f)",
               (double)(g_vision.laser_x - g_vision.circle_x),
               (double)(g_vision.laser_y - g_vision.circle_y));
    } else {
        printf(" err=NA");
    }
    if (g_vision.tilt_count != 0U) {
        printf(" tilt=(%.2f,%.2f,%lums)",
               (double)g_vision.tilt_x, (double)g_vision.tilt_y,
               (unsigned long)Task_VisionAge(now, g_vision.tilt_ms));
    } else {
        printf(" tilt=NA");
    }
    if (enc_ok != 0U) {
        printf(" enc=[%ld,%ld,%ld,%ld] enc_src=%s",
               (long)pos[0], (long)pos[1],
               (long)pos[2], (long)pos[3],
               enc_source);
    } else {
        printf(" enc=READ_FAIL");
    }
    printf("\r\n");
#else
    (void)name;
    (void)step;
    (void)total;
    (void)circle_id;
#endif
}

static void Task_EdgePatrolPrintEncVector(const char *label,
                                          const int32_t value[4],
                                          uint8_t mask)
{
    printf("%s=[", (label != NULL) ? label : "enc");
    for (uint8_t i = 0U; i < 4U; i++) {
        if (i != 0U) {
            putchar(',');
        }
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            printf("%ld", (long)value[i]);
        } else {
            printf("NA");
        }
    }
    printf("] mask=0x%02X", (unsigned int)mask);
}

static uint8_t Task_EdgePatrolReadEncOnce(int32_t pos[4],
                                          mb_result_t result[4])
{
    uint8_t mask = 0U;

    if (pos == NULL) {
        return 0U;
    }
    for (uint8_t i = 0U; i < 4U; i++) {
        mb_result_t r;

        pos[i] = 0;
        r = motor_read_position((uint8_t)(i + 1U), &pos[i]);
        if (result != NULL) {
            result[i] = r;
        }
        if (r == MB_OK) {
            mask |= (uint8_t)(1U << i);
        } else {
            printf("PATROL SEG enc m%u read=%s\r\n",
                   (unsigned int)(i + 1U), motor_result_str(r));
            motor_print_last_transaction_diag("  segment enc");
        }
        HAL_Delay(EDGE_PATROL_SEGMENT_READ_GAP_MS);
    }
    return mask;
}

static void Task_EdgePatrolResetSegmentEncoderBaseline(void)
{
    int32_t pos[4] = {0};
    mb_result_t result[4] = {MB_ERR_FRAME, MB_ERR_FRAME,
                             MB_ERR_FRAME, MB_ERR_FRAME};
    uint8_t mask = 0U;

    /*
     * A center baseline is reused by every following segment, so a missing
     * motor here would invalidate an entire patrol leg. Position reads are
     * side-effect free and may be retried safely.
     */
    for (uint8_t i = 0U; i < 4U; i++) {
        for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
            result[i] = motor_read_position((uint8_t)(i + 1U), &pos[i]);
            if (result[i] == MB_OK) {
                mask |= (uint8_t)(1U << i);
                break;
            }
            printf("PATROL SEG BASE m%u attempt=%u read=%s\r\n",
                   (unsigned int)(i + 1U),
                   (unsigned int)(attempt + 1U),
                   motor_result_str(result[i]));
            motor_print_last_transaction_diag("  segment base");
            HAL_Delay(40U);
        }
        HAL_Delay(EDGE_PATROL_SEGMENT_READ_GAP_MS);
    }
    s_edge_patrol_prev_enc_mask = mask;
    for (uint8_t i = 0U; i < 4U; i++) {
        s_edge_patrol_prev_enc[i] = pos[i];
    }
    printf("PATROL SEG BASE ");
    Task_EdgePatrolPrintEncVector("enc", s_edge_patrol_prev_enc,
                                  s_edge_patrol_prev_enc_mask);
    printf(" result=[%s,%s,%s,%s]\r\n",
           motor_result_str(result[0]), motor_result_str(result[1]),
           motor_result_str(result[2]), motor_result_str(result[3]));
    HAL_Delay(EDGE_PATROL_POST_READ_QUIET_MS);
}

static uint8_t Task_EdgePatrolEnsureLaserFresh(const char *name,
                                               uint8_t step,
                                               uint8_t segment)
{
    uint8_t mode_sent = 0U;

    for (uint8_t attempt = 0U; attempt < EDGE_PATROL_VISION_RETRY_MAX; attempt++) {
        uint32_t now;

        vision_task();
        now = HAL_GetTick();
        if (g_vision.laser_count != 0U &&
            Task_VisionAge(now, g_vision.laser_ms) <= HOME_VISION_AGE_MS) {
            return 1U;
        }

        if (attempt == 0U && mode_sent == 0U) {
            if (vision_send_mode("total") != 0U) {
                printf("PATROL vision refresh: &mode,total# sent\r\n");
            } else {
                printf("PATROL vision refresh: &mode,total# send failed\r\n");
            }
            mode_sent = 1U;
        }
        if ((attempt + 1U) < EDGE_PATROL_VISION_RETRY_MAX) {
            Task_DelayService(EDGE_PATROL_VISION_RETRY_MS);
        }
    }

    printf("PATROL abort: laser stale before %s step=%u seg=%u "
           "cnt=%lu age=%lums\r\n",
           (name != NULL) ? name : "?",
           (unsigned int)step,
           (unsigned int)segment,
           (unsigned long)g_vision.laser_count,
           (unsigned long)Task_VisionAge(HAL_GetTick(), g_vision.laser_ms));
    printf("PATROL SEG SKIP name=%s step=%u seg=%u "
           "reason=VISION_STALE\r\n",
           (name != NULL) ? name : "?",
           (unsigned int)step,
           (unsigned int)segment);
    return 0U;
}

static void Task_EdgePatrolLogSegment(const char *name,
                                      uint8_t step, uint8_t total,
                                      uint8_t segment, uint8_t segment_total,
                                      uint8_t fine,
                                      float release_limit,
                                      mb_result_t motion_result)
{
#if EDGE_PATROL_SEGMENT_LOG
    motion_segment_report_t report;
    int32_t enc_before[4] = {0};
    int32_t enc_after[4] = {0};
    int32_t enc_delta[4] = {0};
    mb_result_t enc_result[4] = {MB_ERR_FRAME, MB_ERR_FRAME,
                                 MB_ERR_FRAME, MB_ERR_FRAME};
    uint8_t enc_before_mask = s_edge_patrol_prev_enc_mask;
    uint8_t enc_after_mask;
    uint8_t enc_delta_mask = 0U;
    uint32_t now;
    const char *quality;

    motion_get_last_segment_report(&report);
    for (uint8_t i = 0U; i < 4U; i++) {
        enc_before[i] = s_edge_patrol_prev_enc[i];
    }
    enc_after_mask = Task_EdgePatrolReadEncOnce(enc_after, enc_result);
    now = HAL_GetTick();
    for (uint8_t i = 0U; i < 4U; i++) {
        if ((s_edge_patrol_prev_enc_mask & (uint8_t)(1U << i)) != 0U &&
            (enc_after_mask & (uint8_t)(1U << i)) != 0U) {
            enc_delta[i] = enc_after[i] - s_edge_patrol_prev_enc[i];
            enc_delta_mask |= (uint8_t)(1U << i);
        }
        if ((enc_after_mask & (uint8_t)(1U << i)) != 0U) {
            s_edge_patrol_prev_enc[i] = enc_after[i];
        }
    }
    s_edge_patrol_prev_enc_mask = enc_after_mask;

    if (motion_result != MB_OK || report.completed == 0U) {
        quality = "COMM_FAIL";
    } else if (enc_after_mask != 0x0FU) {
        quality = "ENC_PARTIAL";
    } else if (g_vision.laser_count == 0U ||
               Task_VisionAge(now, g_vision.laser_ms) > HOME_VISION_AGE_MS) {
        quality = "VISION_STALE";
    } else if (g_vision.circle_count == 0U ||
               Task_VisionAge(now, g_vision.circle_ms) > HOME_VISION_AGE_MS) {
        quality = "CIRCLE_STALE";
    } else {
        quality = "GOOD";
    }

    printf("PATROL SEG seq=%lu name=%s step=%u/%u seg=%u/%u fine=%u "
           "status=%s complete=%u move_ms=%lu bal=%u "
           "release<=%.2f "
           "from=(%.2f,%.2f) to=(%.2f,%.2f) "
           "dL=[%.4f,%.4f,%.4f,%.4f] cmd=[%.4f,%.4f,%.4f,%.4f] "
           "gain=[%.4f,%.4f,%.4f,%.4f]\r\n",
           (unsigned long)report.sequence,
           (name != NULL) ? name : "?",
           (unsigned int)step, (unsigned int)total,
           (unsigned int)segment, (unsigned int)segment_total,
           (unsigned int)fine,
           motor_result_str(motion_result),
           (unsigned int)report.completed,
           (unsigned long)report.estimated_move_ms,
           (unsigned int)report.bal,
           (double)release_limit,
           (double)report.x_now, (double)report.y_now,
           (double)report.x_next, (double)report.y_next,
           (double)report.dL[0], (double)report.dL[1],
           (double)report.dL[2], (double)report.dL[3],
           (double)report.cmd_cm[0], (double)report.cmd_cm[1],
           (double)report.cmd_cm[2], (double)report.cmd_cm[3],
           (double)report.gain[0], (double)report.gain[1],
           (double)report.gain[2], (double)report.gain[3]);

    printf("PATROL SEG BUS seq=%lu dir=[%u,%u,%u,%u] "
           "mag=[%lu,%lu,%lu,%lu] vmax=[%u,%u,%u,%u] "
           "motor=[%s,%s,%s,%s] trigger=%s trigger_attempted=%u "
           "attempted=0x%02X failed_m=%u\r\n",
           (unsigned long)report.sequence,
           (unsigned int)report.dir[0], (unsigned int)report.dir[1],
           (unsigned int)report.dir[2], (unsigned int)report.dir[3],
           (unsigned long)report.mag[0], (unsigned long)report.mag[1],
           (unsigned long)report.mag[2], (unsigned long)report.mag[3],
           (unsigned int)report.vmax[0], (unsigned int)report.vmax[1],
           (unsigned int)report.vmax[2], (unsigned int)report.vmax[3],
           motor_result_str(report.motor_result[0]),
           motor_result_str(report.motor_result[1]),
           motor_result_str(report.motor_result[2]),
           motor_result_str(report.motor_result[3]),
           motor_result_str(report.trigger_result),
           (unsigned int)report.trigger_attempted,
           (unsigned int)report.attempted_mask,
           (unsigned int)report.failed_motor);

    printf("PATROL SEG DATA seq=%lu tick=%lums ",
           (unsigned long)report.sequence, (unsigned long)now);
    Task_EdgePatrolPrintEncVector("enc_before", enc_before,
                                  enc_before_mask);
    printf(" ");
    Task_EdgePatrolPrintEncVector("enc_after", enc_after, enc_after_mask);
    printf(" ");
    Task_EdgePatrolPrintEncVector("denc", enc_delta, enc_delta_mask);
    printf(" enc_result=[%s,%s,%s,%s]",
           motor_result_str(enc_result[0]), motor_result_str(enc_result[1]),
           motor_result_str(enc_result[2]), motor_result_str(enc_result[3]));
    if (g_vision.laser_count != 0U) {
        printf(" laser=(%.2f,%.2f,%lums,cnt=%lu)",
               (double)g_vision.laser_x, (double)g_vision.laser_y,
               (unsigned long)Task_VisionAge(now, g_vision.laser_ms),
               (unsigned long)g_vision.laser_count);
    } else {
        printf(" laser=NA");
    }
    if (g_vision.circle_count != 0U) {
        printf(" circle=(%.2f,%.2f,%lums,cnt=%lu)",
               (double)g_vision.circle_x, (double)g_vision.circle_y,
               (unsigned long)Task_VisionAge(now, g_vision.circle_ms),
               (unsigned long)g_vision.circle_count);
    } else {
        printf(" circle=NA");
    }
    if (g_vision.laser_count != 0U && g_vision.circle_count != 0U) {
        printf(" err=(%.2f,%.2f)",
               (double)(g_vision.laser_x - g_vision.circle_x),
               (double)(g_vision.laser_y - g_vision.circle_y));
    } else {
        printf(" err=NA");
    }
    if (g_vision.tilt_count != 0U) {
        printf(" tilt=(%.2f,%.2f,%lums,cnt=%lu)",
               (double)g_vision.tilt_x, (double)g_vision.tilt_y,
               (unsigned long)Task_VisionAge(now, g_vision.tilt_ms),
               (unsigned long)g_vision.tilt_count);
    } else {
        printf(" tilt=NA");
    }
    printf(" quality=%s\r\n", quality);
    /*
     * The next patrol segment otherwise starts immediately after M4's
     * encoder response.  Preserve every per-segment encoder sample, but add
     * a quiet window before the following four buffered writes.  Read
     * failures remain logged as ENC_PARTIAL and relative writes are never
     * blindly repeated.
     */
    if (motion_result == MB_OK) {
        HAL_Delay(EDGE_PATROL_POST_READ_QUIET_MS);
    }
#else
    (void)name;
    (void)step;
    (void)total;
    (void)segment;
    (void)segment_total;
    (void)fine;
    (void)release_limit;
    (void)motion_result;
#endif
}

static uint8_t Task_EdgePatrolFineCircle(uint8_t circle_id,
                                         uint8_t waypoint_step,
                                         uint8_t total)
{
    static const float ref_x[4] = {
        -HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM,
        +HOME_TOTAL_CORNER_CM, -HOME_TOTAL_CORNER_CM
    };
    static const float ref_y[4] = {
        +HOME_TOTAL_CORNER_CM, +HOME_TOTAL_CORNER_CM,
        -HOME_TOTAL_CORNER_CM, -HOME_TOTAL_CORNER_CM
    };
    uint8_t idx;

    if (circle_id == 0U || circle_id > 4U) {
        return 0U;
    }
    idx = (uint8_t)(circle_id - 1U);

    for (uint8_t iter = 0U; iter < EDGE_PATROL_FINE_MAX_ITER; iter++) {
        float err_x = 0.0f;
        float err_y = 0.0f;
        float circle_x = 0.0f;
        float circle_y = 0.0f;
        float matched_x = 0.0f;
        float matched_y = 0.0f;
        float dist;
        float fine_step;
        float xn;
        float yn;
        float dx;
        float dy;
        uint8_t segments;

        if (Task_EdgePatrolEnsureLaserFresh(Task_EdgePatrolCircleName(circle_id),
                                            waypoint_step, 0U) == 0U) {
            printf("PATROL fine %s abort: laser stale at k=%u\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (unsigned int)iter);
            return 0U;
        }
        Task_DelayService(180U);
        if (vision_is_online(HOME_VISION_AGE_MS) == 0U ||
            vision_read_circle_error_with_ref_filtered(&err_x, &err_y,
                                                       &circle_x, &circle_y) == 0U) {
            printf("PATROL fine %s wait target unavailable k=%u\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (unsigned int)iter);
            Task_DelayService(250U);
            continue;
        }

        if (Task_MatchKnownVisionCircle(circle_x, circle_y, &matched_x, &matched_y) == 0U ||
            fabsf(matched_x - ref_x[idx]) > 0.1f ||
            fabsf(matched_y - ref_y[idx]) > 0.1f) {
            printf("PATROL fine %s skip: seen_other circle=(%.1f,%.1f) err=(%.2f,%.2f)\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (double)circle_x, (double)circle_y,
                   (double)err_x, (double)err_y);
            Task_DelayService(250U);
            continue;
        }

        dist = sqrtf(err_x * err_x + err_y * err_y);
        printf("PATROL fine %s k=%u err=(%.2f,%.2f) d=%.2f soft=(%.2f,%.2f)\r\n",
               Task_EdgePatrolCircleName(circle_id),
               (unsigned int)iter,
               (double)err_x, (double)err_y,
               (double)dist,
               (double)s_manual_x, (double)s_manual_y);
        if (dist <= EDGE_PATROL_OK_CM) {
            Task_EdgePatrolSampleNode(circle_id,
                                      Task_EdgePatrolCircleName(circle_id),
                                      err_x, err_y);
            return 1U;
        }

        fine_step = (dist < EDGE_PATROL_FINE_STEP_CM) ?
                    dist : EDGE_PATROL_FINE_STEP_CM;
        if (g_vision.tilt_count != 0U &&
            Task_VisionAge(HAL_GetTick(), g_vision.tilt_ms) <= HOME_VISION_AGE_MS &&
            (fabsf(g_vision.tilt_x) >= EDGE_PATROL_TILT_SLOW ||
             fabsf(g_vision.tilt_y) >= EDGE_PATROL_TILT_SLOW)) {
            fine_step *= 0.5f;
            printf("PATROL fine %s tilt slow tilt=(%.2f,%.2f) step=%.2f\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (double)g_vision.tilt_x, (double)g_vision.tilt_y,
                   (double)fine_step);
        }

        dx = -(err_x / dist) * fine_step;
        dy = -(err_y / dist) * fine_step;
        xn = s_manual_x + dx;
        yn = s_manual_y + dy;
        if (kin_in_range(xn, yn) == 0U) {
            printf("PATROL fine %s reject next=(%.2f,%.2f)\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (double)xn, (double)yn);
            return 0U;
        }

        segments = Task_EdgePatrolCalcSegments(s_manual_x, s_manual_y, xn, yn,
                                               circle_id);
        {
            float fine_start_x = s_manual_x;
            float fine_start_y = s_manual_y;

        for (uint8_t k = 0U; k < segments; k++) {
            float t = (float)(k + 1U) / (float)segments;
            float sx = fine_start_x + (xn - fine_start_x) * t;
            float sy = fine_start_y + (yn - fine_start_y) * t;
            float release_limit = Task_EdgePatrolReleaseLimit(s_manual_x,
                                                              s_manual_y,
                                                              sx, sy,
                                                              circle_id);
            mb_result_t r;

            if (Task_EdgePatrolEnsureLaserFresh(Task_EdgePatrolCircleName(circle_id),
                                                waypoint_step,
                                                (uint8_t)(k + 1U)) == 0U) {
                printf("PATROL fine %s abort: vision stale before segment\r\n",
                       Task_EdgePatrolCircleName(circle_id));
                return 0U;
            }
            printf("PATROL fine cmd %s seg=%u/%u "
                   "from=(%.2f,%.2f)->(%.2f,%.2f)\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (unsigned int)(k + 1U), (unsigned int)segments,
                   (double)s_manual_x, (double)s_manual_y,
                   (double)sx, (double)sy);
            r = motion_move_between_nohome_patrol(s_manual_x, s_manual_y,
                                                  sx, sy,
                                                  EDGE_PATROL_TAKEUP_GAIN,
                                                  EDGE_PATROL_PAYOUT_GAIN);
            if (r != MB_OK) {
                printf("PATROL fine %s move fail: %s\r\n",
                       Task_EdgePatrolCircleName(circle_id),
                       motor_result_str(r));
                Task_EdgePatrolLogSegment(Task_EdgePatrolCircleName(circle_id),
                                          waypoint_step, total,
                                          (uint8_t)(k + 1U),
                                          segments, 1U, release_limit, r);
                Task_ManualJogRequireRehome("patrol-fine", r);
                return 0U;
            }
            s_manual_x = sx;
            s_manual_y = sy;
            Task_ManualVisionLiveLog("patrol-fine");
            Task_DelayService(EDGE_PATROL_SEG_SETTLE_MS);
            Task_EdgePatrolLogSegment(Task_EdgePatrolCircleName(circle_id),
                                      waypoint_step, total,
                                      (uint8_t)(k + 1U),
                                      segments, 1U, release_limit, MB_OK);
        }
        }
    }

    return Task_EdgePatrolCircleReached(circle_id);
}

static uint8_t Task_EdgePatrolMoveTo(const EdgePatrolWaypoint_t *wp,
                                     uint8_t step, uint8_t total)
{
    float start_x;
    float start_y;
    float target_x;
    float target_y;
    float dx;
    float dy;
    float dist;
    uint8_t segments;

    if (wp == NULL) {
        return 0U;
    }
    target_x = wp->x;
    target_y = wp->y;
    if (wp->circle_id != 0U &&
        Task_EdgePatrolGetLearnedTarget(wp->circle_id,
                                        &target_x, &target_y) != 0U) {
        printf("PATROL target learned %s ideal=(%.2f,%.2f)->avg=(%.2f,%.2f) n=%u\r\n",
               Task_EdgePatrolCircleName(wp->circle_id),
               (double)wp->x, (double)wp->y,
               (double)target_x, (double)target_y,
               (unsigned int)s_edge_patrol_cal[wp->circle_id].count);
    }

    if (kin_in_range(target_x, target_y) == 0U) {
        printf("PATROL reject waypoint %s x=%.2f y=%.2f out of range\r\n",
               (wp->name != NULL) ? wp->name : "?",
               (double)target_x, (double)target_y);
        return 0U;
    }

    start_x = s_manual_x;
    start_y = s_manual_y;
    dx = target_x - start_x;
    dy = target_y - start_y;
    dist = sqrtf(dx * dx + dy * dy);

    if (dist < 0.05f) {
        Task_ShowEdgePatrol(wp->name, step, total, s_manual_x, s_manual_y);
        return 1U;
    }

    segments = Task_EdgePatrolCalcSegments(start_x, start_y, target_x, target_y,
                                           wp->circle_id);

    for (uint8_t k = 0U; k < segments; k++) {
        float t = (float)(k + 1U) / (float)segments;
        float xn = start_x + dx * t;
        float yn = start_y + dy * t;
        float release_limit = Task_EdgePatrolReleaseLimit(s_manual_x, s_manual_y,
                                                          xn, yn,
                                                          wp->circle_id);
        mb_result_t r;

        Task_ShowEdgePatrol(wp->name, step, total, xn, yn);
        printf("PATROL cmd %s step=%u/%u seg=%u/%u "
               "from=(%.2f,%.2f)->(%.2f,%.2f) "
               "gain(T=%.2f P=%.2f) rel<=%.1f\r\n",
               (wp->name != NULL) ? wp->name : "?",
               (unsigned int)step, (unsigned int)total,
               (unsigned int)(k + 1U), (unsigned int)segments,
               (double)s_manual_x, (double)s_manual_y,
               (double)xn, (double)yn,
               (double)EDGE_PATROL_TAKEUP_GAIN,
               (double)EDGE_PATROL_PAYOUT_GAIN,
               (double)release_limit);

        if (Task_EdgePatrolEnsureLaserFresh(wp->name, step,
                                            (uint8_t)(k + 1U)) == 0U) {
            printf("PATROL abort: vision stale before %s seg=%u/%u\r\n",
                   (wp->name != NULL) ? wp->name : "?",
                   (unsigned int)(k + 1U), (unsigned int)segments);
            return 0U;
        }

        r = motion_move_between_nohome_patrol(s_manual_x, s_manual_y,
                                              xn, yn,
                                              EDGE_PATROL_TAKEUP_GAIN,
                                              EDGE_PATROL_PAYOUT_GAIN);
        if (r != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "PATROL MOVE FAIL");
            OLED_ShowLine(2, "CHECK BUS/TENSION");
            OLED_Refresh();
            printf("PATROL move fail at %s seg=%u/%u: %s\r\n",
                   (wp->name != NULL) ? wp->name : "?",
                   (unsigned int)(k + 1U), (unsigned int)segments,
                   motor_result_str(r));
            Task_EdgePatrolLogSegment(wp->name, step, total,
                                      (uint8_t)(k + 1U), segments,
                                      0U, release_limit, r);
            Task_ManualJogRequireRehome("patrol-step", r);
            return 0U;
        }

        s_manual_x = xn;
        s_manual_y = yn;
        Task_ManualVisionLiveLog("patrol");
        Task_DelayService(EDGE_PATROL_SEG_SETTLE_MS);
        Task_EdgePatrolLogSegment(wp->name, step, total,
                                  (uint8_t)(k + 1U), segments,
                                  0U, release_limit, MB_OK);

        if (Key0_Raw() != 0U) {
            Task_ManualJogStopForce();
            printf("PATROL abort by KEY0 at %s\r\n",
                   (wp->name != NULL) ? wp->name : "?");
            return 0U;
        }
    }

    Task_EdgePatrolLogNode(wp->name, step, total, wp->circle_id);

    if (wp->circle_id != 0U) {
        uint8_t reached = Task_EdgePatrolFineCircle(wp->circle_id,
                                                    step, total);

        if (reached != 0U) {
            s_edge_patrol_confirmed_mask |=
                (uint8_t)(1U << (wp->circle_id - 1U));
            OLED_ShowLine(5, "CIRCLE OK");
            printf("PATROL hit %s confirmed\r\n",
                   Task_EdgePatrolCircleName(wp->circle_id));
        } else {
            s_edge_patrol_unconfirmed_mask |=
                (uint8_t)(1U << (wp->circle_id - 1U));
            OLED_ShowLine(5, "CIRCLE MISS");
            printf("PATROL target %s unconfirmed; encoder path retained, "
                   "calibration sample NOT recorded\r\n",
                   Task_EdgePatrolCircleName(wp->circle_id));
        }
        OLED_Refresh();
        Task_BuzzerSet(1U);
        Task_DelayService(80U);
        Task_BuzzerSet(0U);
        Task_DelayService(EDGE_PATROL_DWELL_MS);
    }

    return 1U;
}

static uint8_t Task_EdgePatrolAutoZeroBeforeRun(void)
{
    OLED_Clear();
    OLED_ShowLine(0, "PATROL AUTO 0");
    OLED_ShowLine(2, "CAM FINE ZERO");
    OLED_Refresh();

    if (fabsf(s_manual_x) <= EDGE_PATROL_SOFT_CENTER_CM &&
        fabsf(s_manual_y) <= EDGE_PATROL_SOFT_CENTER_CM) {
        printf("PATROL auto zero: soft near center, visual fine zero only\r\n");
        if (Task_ManualAutoCenterAndRecord(1U) == 0U) {
            printf("PATROL auto zero abort: visual fine zero failed\r\n");
            return 0U;
        }
        return 1U;
    }

    OLED_Clear();
    OLED_ShowLine(0, "PATROL AUTO 0");
    OLED_ShowLine(2, "MOTOR RETURN");
    OLED_Refresh();

    printf("PATROL auto zero: motor return first\r\n");
    if (Task_ManualReturnZero() == 0U) {
        printf("PATROL auto zero abort: motor return failed\r\n");
        return 0U;
    }

    if (s_manual_return_last_skipped != 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "PATROL AUTO 0");
        OLED_ShowLine(2, "CAM FINE ZERO");
        OLED_Refresh();

        printf("PATROL auto zero: already at motor zero, run visual fine zero\r\n");
        if (Task_ManualAutoCenterAndRecord(1U) == 0U) {
            printf("PATROL auto zero abort: visual fine zero failed\r\n");
            return 0U;
        }
    } else {
        printf("PATROL auto zero: motor return included visual fine zero\r\n");
    }

    return 1U;
}

static uint8_t Task_EdgePatrolAutoZeroWithRecovery(const char *reason)
{
    const char *label = (reason != NULL && reason[0] != '\0') ?
                        reason : "AUTOZERO";

    if (Task_EdgePatrolAutoZeroBeforeRun() != 0U) {
        return 1U;
    }

    for (uint8_t attempt = 0U;
         attempt < EDGE_PATROL_CENTER_RESET_RETRY_MAX;
         attempt++) {
        printf("PATROL auto zero recovery %u/%u after %s: "
               "stop, wait %lums, motor return + visual zero\r\n",
               (unsigned int)(attempt + 1U),
               (unsigned int)EDGE_PATROL_CENTER_RESET_RETRY_MAX,
               label,
               (unsigned long)EDGE_PATROL_CENTER_RESET_RETRY_WAIT_MS);

        OLED_Clear();
        OLED_ShowLine(0, "PATROL RECOVER");
        OLED_ShowLine(2, "WAIT + REHOME");
        OLED_Refresh();

        Task_ManualJogStopForce();
        Task_DelayService(EDGE_PATROL_CENTER_RESET_RETRY_WAIT_MS);

        if (Task_ManualReturnZero() != 0U) {
            printf("PATROL auto zero recovery OK after %s\r\n", label);
            return 1U;
        }
    }

    printf("PATROL auto zero recovery failed after %s\r\n", label);
    return 0U;
}

static uint8_t Task_EdgePatrolRun(void)
{
    uint8_t total = (uint8_t)(sizeof(s_edge_patrol_path) /
                              sizeof(s_edge_patrol_path[0]));

    if (s_manual_zero_valid == 0U) {
        printf("PATROL abort: motor zero not saved\r\n");
        return 0U;
    }

    printf("PATROL start: PA0, four edge orange circles, target=%.1fcm inner=%.1fcm\r\n",
           (double)EDGE_PATROL_TARGET_CM, (double)EDGE_PATROL_MID_CM);
    printf("PATROL tension profile: center=40@60, balance=OFF, "
           "path gain(T=%.2f P=%.2f), vmax=300\r\n",
           (double)EDGE_PATROL_TAKEUP_GAIN,
           (double)EDGE_PATROL_PAYOUT_GAIN);
    printf("PATROL bus pace: write_gap=160ms nohome_gap=160ms "
           "m2_pre=120ms m2_post=80ms "
           "encoder_gap=%lums post_read_quiet=%lums\r\n",
           (unsigned long)EDGE_PATROL_SEGMENT_READ_GAP_MS,
           (unsigned long)EDGE_PATROL_POST_READ_QUIET_MS);
    s_edge_patrol_prev_enc_mask = 0U;
    s_edge_patrol_confirmed_mask = 0U;
    s_edge_patrol_unconfirmed_mask = 0U;

    OLED_Clear();
    OLED_ShowLine(0, "PATROL PREP");
    OLED_ShowLine(2, "RETURN CENTER");
    OLED_Refresh();

    if (Task_EdgePatrolAutoZeroWithRecovery("PATROL-START") == 0U) {
        printf("PATROL abort: auto zero before patrol failed\r\n");
        return 0U;
    }

    if (vision_send_mode("total") == 0U) {
        printf("PATROL warn: &mode,total# send failed\r\n");
    } else {
        printf("PATROL cmd: &mode,total# sent\r\n");
    }
    Task_DelayService(300U);
    Task_EdgePatrolResetSegmentEncoderBaseline();

    for (uint8_t i = 1U; i < total; i++) {
        if (Task_EdgePatrolMoveTo(&s_edge_patrol_path[i], i, (uint8_t)(total - 1U)) == 0U) {
            printf("PATROL abort at waypoint %u/%u\r\n",
                   (unsigned int)i, (unsigned int)(total - 1U));
            Task_EdgePatrolPrintCalTable();
            return 0U;
        }

        if (i < (uint8_t)(total - 1U) &&
            fabsf(s_edge_patrol_path[i].x) <= 0.05f &&
            fabsf(s_edge_patrol_path[i].y) <= 0.05f) {
            printf("PATROL center reset after %s\r\n",
                   (s_edge_patrol_path[i].name != NULL) ?
                   s_edge_patrol_path[i].name : "CENTER");
            if (Task_EdgePatrolAutoZeroWithRecovery(
                    (s_edge_patrol_path[i].name != NULL) ?
                    s_edge_patrol_path[i].name : "CENTER") == 0U) {
                printf("PATROL abort: center reset failed after waypoint %u/%u\r\n",
                       (unsigned int)i, (unsigned int)(total - 1U));
                Task_EdgePatrolPrintCalTable();
                return 0U;
            }
            if (vision_send_mode("total") == 0U) {
                printf("PATROL warn: &mode,total# send failed after center reset\r\n");
            } else {
                printf("PATROL cmd: &mode,total# sent after center reset\r\n");
            }
            Task_DelayService(300U);
            Task_EdgePatrolResetSegmentEncoderBaseline();
        }
    }

    printf("PATROL path done, refresh visual center\r\n");
    OLED_Clear();
    OLED_ShowLine(0, "PATROL PATH OK");
    OLED_ShowLine(2, "FINE CENTER");
    OLED_Refresh();

    if (Task_EdgePatrolAutoZeroWithRecovery("PATROL-FINAL") == 0U) {
        printf("PATROL final center failed\r\n");
        Task_EdgePatrolPrintCalTable();
        return 0U;
    }

    Task_EdgePatrolPrintCalTable();
    printf("PATROL target masks confirmed=0x%02X unconfirmed=0x%02X\r\n",
           (unsigned int)s_edge_patrol_confirmed_mask,
           (unsigned int)s_edge_patrol_unconfirmed_mask);
    if (s_edge_patrol_confirmed_mask != 0x0FU) {
        OLED_Clear();
        OLED_ShowLine(0, "PATROL PARTIAL");
        OLED_ShowLine(2, "VISION CHECK");
        OLED_ShowLine(5, "PA0: AGAIN");
        OLED_Refresh();
        printf("PATROL partial: not all four circles visually confirmed; "
               "no unconfirmed point was used as calibration\r\n");
        Task_DelayService(700U);
        return 0U;
    }
    OLED_Clear();
    OLED_ShowLine(0, "PATROL DONE");
    OLED_ShowLine(2, "CENTER OK");
    OLED_ShowLine(5, "PA0: AGAIN");
    OLED_Refresh();
    printf("PATROL done: four edge circles visited and center refreshed\r\n");
    Task_DelayService(700U);
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

        if (Task_EdgePatrolKeyEdge() != 0U) {
            Task_ManualJogStopForce();
            s_key0_last = Key0_Raw();
            if (s_manual_jog_rehome_required != 0U) {
                printf("PATROL retry key PA0: auto rehome before restart\r\n");
            }
            if (Task_EdgePatrolRun() != 0U) {
                s_state = TS_MANUAL_JOG;
                Task_ShowManualJog();
            } else {
                s_state = TS_MANUAL_JOG;
                Task_DelayService(800U);
                Task_ShowManualJog();
            }
            s_edge_patrol_key_last = ManualKey_IsPressed(&s_edge_patrol_key);
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

    if (g_vision.laser_count == 0U && g_vision.circle_count != 0U) {
        if (vision_send_mode("total") != 0U) {
            printf("HOME fallback cmd: &mode,total# sent, waiting LASER-CIRCLE\r\n");
            Task_DelayService(350U);
            if (vision_read_circle_error_with_ref_filtered(&err_x, &err_y,
                                                           &circle_x, &circle_y) != 0U) {
                if (Task_MatchKnownVisionCircle(circle_x, circle_y, &ref_x, &ref_y) != 0U &&
                    (fabsf(ref_x) > 0.1f || fabsf(ref_y) > 0.1f)) {
                    printf("HOME fallback reject: current circle ref=(%.1f,%.1f), need center circle\r\n",
                           (double)ref_x, (double)ref_y);
                    return 0U;
                }
                *x = err_x;
                *y = err_y;
                printf("HOME src=TOTAL LASER-CIRCLE x=%.2f y=%.2f\r\n",
                       (double)*x, (double)*y);
                return 1U;
            }
        } else {
            printf("HOME fallback warn: &mode,total# send failed\r\n");
        }
    }

    {
        uint32_t now = HAL_GetTick();
        printf("HOME fine abort: no center/local circle error "
               "home=%lu/%lums laser=%lu/%lums circle=%lu/%lums tilt=%lu/%lums\r\n",
               (unsigned long)g_vision.home_count,
               (unsigned long)Task_VisionAge(now, g_vision.home_ms),
               (unsigned long)g_vision.laser_count,
               (unsigned long)Task_VisionAge(now, g_vision.laser_ms),
               (unsigned long)g_vision.circle_count,
               (unsigned long)Task_VisionAge(now, g_vision.circle_ms),
               (unsigned long)g_vision.tilt_count,
               (unsigned long)Task_VisionAge(now, g_vision.tilt_ms));
    }
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

    printf("\r\nCDPR Q2 edge patrol mode\r\n");
    printf("Step1 place gondola near center, press KEY0 to visual-zero and save motor zero.\r\n");
    printf("Step2 press PA0 to patrol four orange edge circles. PC0-PC3 jog, KEY0 returns zero.\r\n");
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
