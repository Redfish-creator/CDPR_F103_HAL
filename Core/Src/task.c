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
#include "route_menu_port.h"
#include "main.h"
#include "vision.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Competition log profile: USART1 prints only state transitions and faults.
 * Keep detailed traces compiled out by default so debug output cannot occupy
 * the foreground loop between closely spaced motor/vision operations.
 */
#ifndef APP_COMPACT_LOG
#define APP_COMPACT_LOG 1U
#endif
#if APP_COMPACT_LOG
#define TASK_TRACE(...) do { } while (0)
#else
#define TASK_TRACE(...) printf(__VA_ARGS__)
#endif

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
#ifndef MANUAL_RETURN_POST_ENCODER_READS
#define MANUAL_RETURN_POST_ENCODER_READS 0U
#endif
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
#ifndef MANUAL_VISION_LOG
#define MANUAL_VISION_LOG         0U
#endif
#define MANUAL_VISION_LIVE_MS     500U
#define MANUAL_VISION_STEP_SAMPLE 0U

#define EDGE_PATROL_START_PORT    GPIOA
#define EDGE_PATROL_START_PIN     GPIO_PIN_0
#define EDGE_PATROL_ORDER_MODE_PORT GPIOA
#define EDGE_PATROL_ORDER_MODE_PIN  GPIO_PIN_15
#define EDGE_PATROL_ORDER_COUNT   4U
#define EDGE_PATROL_ORDER_NONE    0xFFU
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
#define EDGE_PATROL_SEG_SETTLE_MS 300U
#define EDGE_PATROL_OK_CM         0.8f
#define EDGE_PATROL_TAKEUP_GAIN   0.99f
#define EDGE_PATROL_PAYOUT_GAIN   1.01f
/*
 * Field numbering: corner 1 is the M1-side corner, LB in the current route.
 * Latest field note says this corner has a long/loose cable.  Tighten only
 * LB: slightly more take-up and slightly less pay-out.
 */
#define EDGE_PATROL_LB_TAKEUP_GAIN 1.02f
#define EDGE_PATROL_LB_PAYOUT_GAIN 0.99f
#define EDGE_PATROL_MAX_RELEASE_CM 2.0f
#define EDGE_PATROL_TOP_RELEASE_CM 1.2f
#define EDGE_PATROL_FINAL_RELEASE_CM 1.4f
#define EDGE_PATROL_MAX_TAKEUP_CM 3.2f
#define EDGE_PATROL_MAX_SUBSTEPS  24U
#define EDGE_PATROL_FINE_STEP_CM  0.45f
#define EDGE_PATROL_FINE_WORSE_ABORT_CM 2.0f
#define EDGE_PATROL_FINE_MAX_ITER 12U
#define EDGE_PATROL_TILT_SLOW     0.80f
#define EDGE_PATROL_SOFT_CENTER_CM 0.6f
#define EDGE_PATROL_CAL_USE_MIN   1U
#define EDGE_PATROL_USE_TARGET_SEED 0U
#define EDGE_PATROL_CAL_MAX_COUNT 30U
#ifndef EDGE_PATROL_NODE_LOG
#define EDGE_PATROL_NODE_LOG      0U
#endif
#ifndef EDGE_PATROL_SEGMENT_LOG
#define EDGE_PATROL_SEGMENT_LOG   0U
#endif
#ifndef EDGE_PATROL_BUS_DIAG_LOG
/* Dedicated field capture: no extra RS485 reads are added per segment.
 * The already executed four buffered writes are summarized over USART1 only
 * after the move, so this evidence cannot steal USART3 response bytes. */
#define EDGE_PATROL_BUS_DIAG_LOG  1U
#endif
#ifndef EDGE_PATROL_CAL_ENCODER_READS
#define EDGE_PATROL_CAL_ENCODER_READS 0U
#endif
#ifndef EDGE_PATROL_CAL_LOG
#define EDGE_PATROL_CAL_LOG       0U
#endif
/*
 * Stable competition profile. Automatic visual homing uses the same four
 * buffered 0x10 writes and broadcast trigger as patrol, but it does not add
 * four 0x03 position reads after every short move. The competition build also
 * skips corner telemetry reads; only the pre-return positions needed to
 * compute a safe motor-zero return remain. Set the diagnostic switches to 1
 * only for a dedicated high-density capture run.
 */
#ifndef EDGE_PATROL_SEGMENT_ENCODER_READS
#define EDGE_PATROL_SEGMENT_ENCODER_READS 0U
#endif
#define EDGE_PATROL_SEGMENT_READ_GAP_MS 80U
#define EDGE_PATROL_POST_READ_QUIET_MS 120U
#define EDGE_PATROL_VISION_RETRY_MAX 3U
#define EDGE_PATROL_VISION_RETRY_MS  120U
#define EDGE_PATROL_CENTER_RESET_RETRY_MAX 3U
#define EDGE_PATROL_CENTER_RESET_RETRY_WAIT_MS 800U
#define EDGE_PATROL_MOVE_RECOVERY_RETRY_MAX 3U
#define EDGE_PATROL_MOVE_RECOVERY_WAIT_MS 800U
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

typedef struct {
    uint8_t valid;
    uint8_t samples;
    float x;
    float y;
    const char *source;
} EdgePatrolTargetSeed_t;

typedef struct {
    const char *name;
    uint8_t circle_id;
    const EdgePatrolWaypoint_t *waypoints;
    uint8_t waypoint_count;
} EdgePatrolCornerJob_t;

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
#if MANUAL_VISION_LOG
static uint32_t s_manual_vision_log_tick = 0U;
#endif
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
static ManualKeyFilter_t s_edge_order_mode_key;
static uint8_t s_edge_patrol_key_last = 0U;
static uint8_t s_edge_order_mode_key_last = 0U;
static uint8_t s_edge_order_up_last = 0U;
static uint8_t s_edge_order_down_last = 0U;
static uint8_t s_edge_order_left_last = 0U;
static uint8_t s_edge_order_right_last = 0U;
static uint8_t s_edge_order_menu_active = 0U;
static uint8_t s_edge_order_edit_count = 0U;
static uint8_t s_edge_order_cursor = 0U;
static uint8_t s_edge_order_edit[EDGE_PATROL_ORDER_COUNT] = {
    EDGE_PATROL_ORDER_NONE, EDGE_PATROL_ORDER_NONE,
    EDGE_PATROL_ORDER_NONE, EDGE_PATROL_ORDER_NONE
};
static uint8_t s_edge_patrol_order[EDGE_PATROL_ORDER_COUNT] = {
    3U, 1U, 2U, 0U
};
static uint8_t s_manual_return_last_skipped = 0U;
static EdgePatrolCal_t s_edge_patrol_cal[5];
static int32_t s_edge_patrol_prev_enc[4] = {0};
static uint8_t s_edge_patrol_prev_enc_mask = 0U;
static uint8_t s_edge_patrol_confirmed_mask = 0U;
static uint8_t s_edge_patrol_unconfirmed_mask = 0U;
static mb_result_t s_edge_patrol_last_move_result = MB_OK;

/*
 * Empirical soft targets fitted from visually confirmed corner hits.
 * These are not encoder absolute targets.  They are only used as first-pass
 * soft waypoints; visual fine correction still runs at the target circle.
 */
#if EDGE_PATROL_USE_TARGET_SEED
static const EdgePatrolTargetSeed_t s_edge_patrol_target_seed[5] = {
    { 0U, 0U, 0.00f, 0.00f, "" },
    { 1U, 4U, -17.30f, 14.90f, "fit:v11/v12/run12/run13 LT" },
    { 1U, 3U, 19.10f, 18.30f, "fit:v11/v12-retry1 RT" },
    { 1U, 1U, 17.70f, -16.47f, "v12-run8 RB" },
    { 1U, 1U, -15.42f, -13.82f, "v12-run8 LB" }
};
#endif

static const EdgePatrolWaypoint_t s_edge_patrol_lt_job[] = {
    { -4.0f, 4.0f, "LT-IN1", 0U },
    { -8.0f, 8.0f, "LT-IN2", 0U },
    { -12.0f, 11.5f, "LT-APP", 0U },
    { EDGE_PATROL_LT_X, EDGE_PATROL_LT_Y, "LT", 1U }
};

static const EdgePatrolWaypoint_t s_edge_patrol_rt_job[] = {
    { 4.0f, 4.0f, "RT-IN1", 0U },
    { 8.0f, 8.0f, "RT-IN2", 0U },
    { 12.0f, 12.0f, "RT-APP", 0U },
    { EDGE_PATROL_RT_X, EDGE_PATROL_RT_Y, "RT", 2U }
};

static const EdgePatrolWaypoint_t s_edge_patrol_rb_job[] = {
    { 4.0f, -4.0f, "RB-IN1", 0U },
    { 8.0f, -8.0f, "RB-IN2", 0U },
    { 12.0f, -11.5f, "RB-APP", 0U },
    { EDGE_PATROL_RB_X, EDGE_PATROL_RB_Y, "RB", 3U }
};

static const EdgePatrolWaypoint_t s_edge_patrol_lb_job[] = {
    { -4.0f, -4.0f, "LB-IN1", 0U },
    { -8.0f, -8.0f, "LB-IN2", 0U },
    { -12.0f, -11.5f, "LB-APP", 0U },
    { EDGE_PATROL_LB_X, EDGE_PATROL_LB_Y, "LB", 4U }
};

static const EdgePatrolCornerJob_t s_edge_patrol_corner_jobs[] = {
    { "LT", 1U, s_edge_patrol_lt_job,
      (uint8_t)(sizeof(s_edge_patrol_lt_job) /
                sizeof(s_edge_patrol_lt_job[0])) },
    { "RT", 2U, s_edge_patrol_rt_job,
      (uint8_t)(sizeof(s_edge_patrol_rt_job) /
                sizeof(s_edge_patrol_rt_job[0])) },
    { "RB", 3U, s_edge_patrol_rb_job,
      (uint8_t)(sizeof(s_edge_patrol_rb_job) /
                sizeof(s_edge_patrol_rb_job[0])) },
    { "LB", 4U, s_edge_patrol_lb_job,
      (uint8_t)(sizeof(s_edge_patrol_lb_job) /
                sizeof(s_edge_patrol_lb_job[0])) }
};

/*
 * Field point numbers follow the motor/anchor corner numbering, not the
 * internal job array above:
 *   P1/M1 = LB, P2/M2 = RT, P3/M3 = RB, P4/M4 = LT.
 * Keep these labels next to the job table so OLED selection, serial output,
 * and actual execution always use the same mapping.
 */
static const char *s_edge_patrol_point_labels[] = {
    "4LT", "2RT", "3RB", "1LB"
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
static uint8_t Task_EdgeOrderModeKeyEdge(void);
static uint8_t Task_KeyPressedEdge(const ManualKeyFilter_t *kf,
                                   uint8_t *last);
static void Task_EdgeOrderInitDefault(void);
static void Task_EdgeOrderSyncKeyEdges(void);
static void Task_EdgeOrderEnter(void);
static void Task_EdgeOrderExit(void);
static uint8_t Task_EdgeOrderMenuService(void);
static void Task_EdgeOrderDraw(void);
static void Task_EdgeOrderResetEdit(void);
static uint8_t Task_EdgeOrderPointUsed(uint8_t idx, uint8_t count);
static uint8_t Task_EdgeOrderFindNext(uint8_t cur);
static uint8_t Task_EdgeOrderFindPrev(uint8_t cur);
static void Task_EdgeOrderConfirm(void);
static void Task_EdgeOrderBack(void);
static uint8_t Task_EdgePatrolJobCount(void);
static const EdgePatrolCornerJob_t *Task_EdgePatrolOrderedJob(uint8_t order_pos);
static void Task_EdgePatrolFormatOrder(const uint8_t *order,
                                       uint8_t count,
                                       char *buf,
                                       uint8_t buf_len);
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
static mb_result_t Task_StopAllImmediate(const char *tag);
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
static void Task_EdgePatrolMotionGains(const char *name,
                                       uint8_t circle_id,
                                       float *takeup_gain,
                                       float *payout_gain);
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
#if EDGE_PATROL_SEGMENT_LOG
static uint8_t Task_EdgePatrolReadEncOnce(int32_t pos[4],
                                          mb_result_t result[4]);
#endif
static void Task_EdgePatrolResetSegmentEncoderBaseline(void);
static uint8_t Task_EdgePatrolRecoverAfterMoveFail(const char *name,
                                                   uint8_t step,
                                                   uint8_t total,
                                                   mb_result_t result);
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
static uint8_t Task_EdgePatrolRunCornerJob(const EdgePatrolCornerJob_t *job,
                                           uint8_t first_step,
                                           uint8_t total_steps);
static uint8_t Task_EdgePatrolResetAfterCorner(const EdgePatrolCornerJob_t *job,
                                               uint8_t job_step,
                                               uint8_t job_total);
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
                TASK_TRACE("  %s buf m%d retry ok attempt=%u\r\n",
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
    memset(&s_edge_order_mode_key, 0, sizeof(s_edge_order_mode_key));
    s_edge_patrol_key_last = 0U;
    s_edge_order_mode_key_last = 0U;
    s_edge_order_up_last = 0U;
    s_edge_order_down_last = 0U;
    s_edge_order_left_last = 0U;
    s_edge_order_right_last = 0U;
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
    Task_EdgeOrderInitDefault();
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
    ManualKey_Update(&s_edge_order_mode_key,
                     (HAL_GPIO_ReadPin(EDGE_PATROL_ORDER_MODE_PORT,
                                       EDGE_PATROL_ORDER_MODE_PIN) == GPIO_PIN_RESET) ? 1U : 0U);
}

static uint8_t Task_EdgePatrolKeyEdge(void)
{
    uint8_t pressed = ManualKey_IsPressed(&s_edge_patrol_key);
    uint8_t edge = (pressed != 0U && s_edge_patrol_key_last == 0U) ? 1U : 0U;

    s_edge_patrol_key_last = pressed;
    return edge;
}

static uint8_t Task_KeyPressedEdge(const ManualKeyFilter_t *kf,
                                   uint8_t *last)
{
    uint8_t pressed;
    uint8_t edge;

    if (kf == NULL || last == NULL) {
        return 0U;
    }

    pressed = ManualKey_IsPressed(kf);
    edge = (pressed != 0U && *last == 0U) ? 1U : 0U;
    *last = pressed;
    return edge;
}

static uint8_t Task_EdgeOrderModeKeyEdge(void)
{
    return Task_KeyPressedEdge(&s_edge_order_mode_key,
                               &s_edge_order_mode_key_last);
}

static uint8_t Task_EdgePatrolJobCount(void)
{
    return (uint8_t)(sizeof(s_edge_patrol_corner_jobs) /
                     sizeof(s_edge_patrol_corner_jobs[0]));
}

static const EdgePatrolCornerJob_t *Task_EdgePatrolOrderedJob(uint8_t order_pos)
{
    uint8_t job_total = Task_EdgePatrolJobCount();
    uint8_t idx;

    if (job_total == 0U) {
        return NULL;
    }
    if (order_pos >= EDGE_PATROL_ORDER_COUNT) {
        order_pos = 0U;
    }

    idx = s_edge_patrol_order[order_pos];
    if (idx >= job_total) {
        idx = (uint8_t)(order_pos % job_total);
    }
    return &s_edge_patrol_corner_jobs[idx];
}

static void Task_EdgePatrolFormatOrder(const uint8_t *order,
                                       uint8_t count,
                                       char *buf,
                                       uint8_t buf_len)
{
    uint8_t pos = 0U;

    if (buf == NULL || buf_len == 0U) {
        return;
    }
    buf[0] = '\0';

    for (uint8_t i = 0U; i < count && i < EDGE_PATROL_ORDER_COUNT; i++) {
        uint8_t idx = (order != NULL) ? order[i] : s_edge_patrol_order[i];
        const char *name = "?";
        int n;

        if (idx < Task_EdgePatrolJobCount()) {
            name = s_edge_patrol_point_labels[idx];
        }

        n = snprintf(&buf[pos], (size_t)(buf_len - pos),
                     "%s%s", (i == 0U) ? "" : ">", name);
        if (n <= 0) {
            return;
        }
        if ((uint8_t)n >= (uint8_t)(buf_len - pos)) {
            buf[buf_len - 1U] = '\0';
            return;
        }
        pos = (uint8_t)(pos + (uint8_t)n);
    }
}

static uint8_t Task_EdgeOrderPointUsed(uint8_t idx, uint8_t count)
{
    for (uint8_t i = 0U; i < count && i < EDGE_PATROL_ORDER_COUNT; i++) {
        if (s_edge_order_edit[i] == idx) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t Task_EdgeOrderFindNext(uint8_t cur)
{
    uint8_t total = Task_EdgePatrolJobCount();

    if (total == 0U) {
        return 0U;
    }
    for (uint8_t i = 1U; i <= total; i++) {
        uint8_t idx = (uint8_t)((cur + i) % total);
        if (Task_EdgeOrderPointUsed(idx, s_edge_order_edit_count) == 0U) {
            return idx;
        }
    }
    return cur;
}

static uint8_t Task_EdgeOrderFindPrev(uint8_t cur)
{
    uint8_t total = Task_EdgePatrolJobCount();

    if (total == 0U) {
        return 0U;
    }
    for (uint8_t i = 1U; i <= total; i++) {
        uint8_t idx = (uint8_t)((cur + total - i) % total);
        if (Task_EdgeOrderPointUsed(idx, s_edge_order_edit_count) == 0U) {
            return idx;
        }
    }
    return cur;
}

static void Task_EdgeOrderResetEdit(void)
{
    for (uint8_t i = 0U; i < EDGE_PATROL_ORDER_COUNT; i++) {
        s_edge_order_edit[i] = EDGE_PATROL_ORDER_NONE;
    }
    s_edge_order_edit_count = 0U;
    s_edge_order_cursor = 0U;
}

static void Task_EdgeOrderInitDefault(void)
{
    static const uint8_t default_order[EDGE_PATROL_ORDER_COUNT] = {
        0U, 1U, 2U, 3U
    };
    uint8_t total = Task_EdgePatrolJobCount();

    for (uint8_t i = 0U; i < EDGE_PATROL_ORDER_COUNT; i++) {
        uint8_t idx = default_order[i];
        s_edge_patrol_order[i] = (idx < total) ? idx : 0U;
    }
    Task_EdgeOrderResetEdit();
    s_edge_order_menu_active = 0U;
}

static void Task_EdgeOrderSyncKeyEdges(void)
{
    s_edge_order_up_last = ManualKey_IsPressed(&s_manual_key_up);
    s_edge_order_down_last = ManualKey_IsPressed(&s_manual_key_down);
    s_edge_order_left_last = ManualKey_IsPressed(&s_manual_key_left);
    s_edge_order_right_last = ManualKey_IsPressed(&s_manual_key_right);
    s_edge_patrol_key_last = ManualKey_IsPressed(&s_edge_patrol_key);
}

static const char *Task_EdgeOrderSlotName(uint8_t idx)
{
    if (idx == EDGE_PATROL_ORDER_NONE) {
        return "--";
    }
    if (idx < Task_EdgePatrolJobCount()) {
        return s_edge_patrol_point_labels[idx];
    }
    return "?";
}

static void Task_EdgeOrderLoadActive(void)
{
    uint8_t total = Task_EdgePatrolJobCount();

    if (total == 0U) {
        Task_EdgeOrderResetEdit();
        return;
    }

    for (uint8_t i = 0U; i < EDGE_PATROL_ORDER_COUNT; i++) {
        uint8_t idx = s_edge_patrol_order[i];
        if (idx >= total) {
            idx = (uint8_t)(i % total);
        }
        s_edge_order_edit[i] = idx;
    }

    s_edge_order_edit_count = EDGE_PATROL_ORDER_COUNT;
    s_edge_order_cursor = s_edge_order_edit[0];
}

static void Task_EdgeOrderDraw(void)
{
    char line[24];
    uint8_t ready = (s_edge_order_edit_count >= EDGE_PATROL_ORDER_COUNT) ? 1U : 0U;

    OLED_Menu_Clear();
    OLED_Menu_ShowLine(0, "Q3 PATROL ORDER");

    if (ready != 0U) {
        OLED_Menu_ShowLine(1, "ORDER READY");
    } else {
        const char *cursor = "?";

        if (s_edge_order_cursor < Task_EdgePatrolJobCount()) {
            cursor = s_edge_patrol_point_labels[s_edge_order_cursor];
        }

        snprintf(line, sizeof(line), "EDIT %u/%u CUR:%s",
                 (unsigned int)(s_edge_order_edit_count + 1U),
                 (unsigned int)EDGE_PATROL_ORDER_COUNT,
                 cursor);
        OLED_Menu_ShowLine(1, line);
    }

    for (uint8_t i = 0U; i < EDGE_PATROL_ORDER_COUNT; i++) {
        char mark = ' ';

        if (ready == 0U && i == s_edge_order_edit_count) {
            mark = '>';
        }
        snprintf(line, sizeof(line), "%c%u:%s",
                 mark,
                 (unsigned int)(i + 1U),
                 Task_EdgeOrderSlotName(s_edge_order_edit[i]));
        OLED_Menu_ShowLine((uint8_t)(2U + i), line);
    }

    if (ready != 0U) {
        OLED_Menu_ShowLine(6, "PC3 EDIT  PA0 START");
        OLED_Menu_ShowLine(7, "KEY1 EXIT");
    } else {
        OLED_Menu_ShowLine(6, "PC0 PRE PC1 NEXT");
        OLED_Menu_ShowLine(7, "PC2 OK  PC3 BACK");
    }

    OLED_Menu_Refresh();
    s_manual_oled_tick = HAL_GetTick();
}

static void Task_EdgeOrderConfirm(void)
{
    char order[22];

    if (s_edge_order_edit_count >= EDGE_PATROL_ORDER_COUNT) {
        TASK_TRACE("PATROL order key PC2 OK ignored: order ready; "
                   "PA0 start or PC3 back to edit\r\n");
        return;
    }
    if (Task_EdgeOrderPointUsed(s_edge_order_cursor,
                                s_edge_order_edit_count) != 0U) {
        printf("PATROL order key PC2 OK ignored: %s already used at step=%u/%u\r\n",
               (s_edge_order_cursor < Task_EdgePatrolJobCount()) ?
               s_edge_patrol_point_labels[s_edge_order_cursor] : "?",
               (unsigned int)(s_edge_order_edit_count + 1U),
               (unsigned int)EDGE_PATROL_ORDER_COUNT);
        s_edge_order_cursor = Task_EdgeOrderFindNext(s_edge_order_cursor);
        return;
    }

    s_edge_order_edit[s_edge_order_edit_count] = s_edge_order_cursor;
    TASK_TRACE("PATROL order key PC2 OK step=%u/%u select=%s\r\n",
               (unsigned int)(s_edge_order_edit_count + 1U),
               (unsigned int)EDGE_PATROL_ORDER_COUNT,
               (s_edge_order_cursor < Task_EdgePatrolJobCount()) ?
               s_edge_patrol_point_labels[s_edge_order_cursor] : "?");
    s_edge_order_edit_count++;

    if (s_edge_order_edit_count >= EDGE_PATROL_ORDER_COUNT) {
        for (uint8_t i = 0U; i < EDGE_PATROL_ORDER_COUNT; i++) {
            s_edge_patrol_order[i] = s_edge_order_edit[i];
        }
        Task_EdgePatrolFormatOrder(s_edge_patrol_order,
                                   EDGE_PATROL_ORDER_COUNT,
                                   order, sizeof(order));
        printf("PATROL order selected by OLED: %s\r\n", order);
        Task_BuzzerSet(1U);
        Task_DelayService(60U);
        Task_BuzzerSet(0U);
    } else {
        s_edge_order_cursor = Task_EdgeOrderFindNext(s_edge_order_cursor);
    }
}

static void Task_EdgeOrderBack(void)
{
    if (s_edge_order_edit_count == 0U) {
        s_edge_order_cursor = 0U;
        TASK_TRACE("PATROL order key PC3 BACK ignored: at first step\r\n");
        return;
    }

    s_edge_order_edit_count--;
    s_edge_order_cursor = s_edge_order_edit[s_edge_order_edit_count];
    s_edge_order_edit[s_edge_order_edit_count] = EDGE_PATROL_ORDER_NONE;
    TASK_TRACE("PATROL order key PC3 BACK to step=%u/%u cursor=%s\r\n",
               (unsigned int)(s_edge_order_edit_count + 1U),
               (unsigned int)EDGE_PATROL_ORDER_COUNT,
               (s_edge_order_cursor < Task_EdgePatrolJobCount()) ?
               s_edge_patrol_point_labels[s_edge_order_cursor] : "?");
}

static void Task_EdgeOrderEnter(void)
{
    char order[22];

    if (s_manual_jog_rehome_required != 0U) {
        printf("PATROL order menu blocked: bus state uncertain; "
               "press KEY0 to rehome first\r\n");
        Task_ShowManualJog();
        return;
    }

    Task_ManualJogStopForce();
    Task_EdgeOrderLoadActive();
    Task_EdgeOrderSyncKeyEdges();
    s_edge_order_menu_active = 1U;

    Task_EdgePatrolFormatOrder(s_edge_patrol_order,
                               EDGE_PATROL_ORDER_COUNT,
                               order, sizeof(order));
    TASK_TRACE("PATROL order menu enter: current=%s; "
               "PC3 back edits from the last slot; "
               "PC0 prev, PC1 next, PC2 ok, PA0 start when ready\r\n",
               order);
    Task_EdgeOrderDraw();
}

static void Task_EdgeOrderExit(void)
{
    char order[22];

    s_edge_order_menu_active = 0U;
    Task_EdgePatrolFormatOrder(s_edge_patrol_order,
                               EDGE_PATROL_ORDER_COUNT,
                               order, sizeof(order));
    TASK_TRACE("PATROL order menu exit: active=%s\r\n", order);
    Task_ShowManualJog();
}

static uint8_t Task_EdgeOrderMenuService(void)
{
    if (s_edge_order_menu_active == 0U) {
        return 0U;
    }

    if (Task_KeyPressedEdge(&s_manual_key_up,
                            &s_edge_order_up_last) != 0U) {
        if (s_edge_order_edit_count >= EDGE_PATROL_ORDER_COUNT) {
            TASK_TRACE("PATROL order key PC0 PRE ignored: order ready; "
                       "press PC3 back to edit\r\n");
        } else {
            s_edge_order_cursor = Task_EdgeOrderFindPrev(s_edge_order_cursor);
            TASK_TRACE("PATROL order key PC0 PRE cursor=%s step=%u/%u\r\n",
                       (s_edge_order_cursor < Task_EdgePatrolJobCount()) ?
                       s_edge_patrol_point_labels[s_edge_order_cursor] : "?",
                       (unsigned int)(s_edge_order_edit_count + 1U),
                       (unsigned int)EDGE_PATROL_ORDER_COUNT);
        }
        Task_EdgeOrderDraw();
    }
    if (Task_KeyPressedEdge(&s_manual_key_down,
                            &s_edge_order_down_last) != 0U) {
        if (s_edge_order_edit_count >= EDGE_PATROL_ORDER_COUNT) {
            TASK_TRACE("PATROL order key PC1 NEXT ignored: order ready; "
                       "press PC3 back to edit\r\n");
        } else {
            s_edge_order_cursor = Task_EdgeOrderFindNext(s_edge_order_cursor);
            TASK_TRACE("PATROL order key PC1 NEXT cursor=%s step=%u/%u\r\n",
                       (s_edge_order_cursor < Task_EdgePatrolJobCount()) ?
                       s_edge_patrol_point_labels[s_edge_order_cursor] : "?",
                       (unsigned int)(s_edge_order_edit_count + 1U),
                       (unsigned int)EDGE_PATROL_ORDER_COUNT);
        }
        Task_EdgeOrderDraw();
    }
    if (Task_KeyPressedEdge(&s_manual_key_left,
                            &s_edge_order_left_last) != 0U) {
        Task_EdgeOrderConfirm();
        Task_EdgeOrderDraw();
    }
    if (Task_KeyPressedEdge(&s_manual_key_right,
                            &s_edge_order_right_last) != 0U) {
        Task_EdgeOrderBack();
        Task_EdgeOrderDraw();
    }

    if (Task_EdgePatrolKeyEdge() != 0U) {
        if (s_edge_order_edit_count >= EDGE_PATROL_ORDER_COUNT) {
            s_edge_order_menu_active = 0U;
            TASK_TRACE("PATROL order menu start by PA0\r\n");
            return 1U;
        }
        printf("PATROL order menu PA0 ignored: order not ready step=%u/%u\r\n",
               (unsigned int)s_edge_order_edit_count,
               (unsigned int)EDGE_PATROL_ORDER_COUNT);
        OLED_Menu_ShowLine(5, "FINISH ORDER FIRST");
        OLED_Menu_Refresh();
    }

    return 0U;
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
    char order[16];

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
    OLED_ShowLine(4, "KEY1: ORDER");
    OLED_ShowLine(5, "PA0: PATROL4");
    OLED_ShowLine(6, "KEY0: RETURN 0");
    Task_EdgePatrolFormatOrder(s_edge_patrol_order,
                               EDGE_PATROL_ORDER_COUNT,
                               order, sizeof(order));
    snprintf(line, sizeof(line), "ORD:%s", order);
    OLED_ShowLine(7, line);
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
    TASK_TRACE("MANUAL tension normalize start count=%u vmax=%u\r\n",
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
        if (r != MB_OK) {
            printf("  tension buf m%d dir=%s mag=%u: %s\r\n",
                   (int)id,
                   (dir == MB_DIR_CW) ? "CW" : "CCW",
                   (unsigned int)MANUAL_CENTER_TENSION_COUNTS,
                   motor_result_str(r));
            for (uint8_t sid = 1U; sid <= 4U; sid++) {
                (void)motor_stop(sid, MB_SYNC_NOW);
                HAL_Delay(15);
            }
            return r;
        }
        TASK_TRACE("  tension buf m%d dir=%s mag=%u: %s\r\n",
                   (int)id,
                   (dir == MB_DIR_CW) ? "CW" : "CCW",
                   (unsigned int)MANUAL_CENTER_TENSION_COUNTS,
                   motor_result_str(r));
        HAL_Delay(25);
    }

    {
        mb_result_t tr = motor_sync_trigger();
        if (tr != MB_OK) {
            printf("  tension trigger: %s\r\n", motor_result_str(tr));
            for (uint8_t sid = 1U; sid <= 4U; sid++) {
                (void)motor_stop(sid, MB_SYNC_NOW);
                HAL_Delay(15);
            }
            return tr;
        }
        TASK_TRACE("  tension trigger: %s\r\n", motor_result_str(tr));
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
        TASK_TRACE("MANUAL tension recenter k=%d x=%.2f y=%.2f d=%.2f\r\n",
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
        TASK_TRACE("MANUAL tension recenter cmd k=%d target=(%.2f,%.2f) step=%.2f\r\n",
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
        TASK_TRACE("MANUAL auto home cmd: &mode,initial# sent\r\n");
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
        TASK_TRACE("MANUAL auto home k=%d x=%.2f y=%.2f d=%.2f\r\n",
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

        TASK_TRACE("MANUAL auto cmd k=%d target=(%.2f,%.2f) step=%.2f mode=EXACT\r\n",
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
    OLED_ShowLine(5, "KEY1: ORDER");
    OLED_ShowLine(6, "PC0-3 JOG");
    OLED_Refresh();
    if (recovering_jog != 0U) {
        printf("MANUAL recovery complete: KEY0 rehome OK; PC0-PC3 unlocked\r\n");
    }
    printf("CENTER zero OK; PA0 patrol, KEY1 order\r\n");
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

static mb_result_t Task_StopAllImmediate(const char *tag)
{
    mb_result_t last = MB_OK;

    for (uint8_t id = 1U; id <= 4U; id++) {
        mb_result_t r = motor_stop(id, MB_SYNC_NOW);
        if (r != MB_OK) {
            last = r;
        }
        HAL_Delay(15U);
    }

    printf("%s stop all: %s\r\n",
           (tag != NULL && tag[0] != '\0') ? tag : "TASK",
           motor_result_str(last));
    if (last != MB_OK) {
        motor_print_last_transaction_diag("  stop all diag");
    }
    return last;
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
            if (last != MB_OK) {
                printf("MANUAL jog stop: %s\r\n", motor_result_str(last));
                Task_ManualJogRequireRehome("stop", last);
            } else {
                TASK_TRACE("MANUAL jog stop: %s\r\n",
                           motor_result_str(last));
            }
        } else {
            TASK_TRACE("MANUAL jog stop: already complete\r\n");
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
        TASK_TRACE("MANUAL jog start dir=%s run=%.2f gain(T=%.2f P=%.2f) x_comp=%.2f limit=%.1f\r\n",
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

static mb_result_t Task_ManualClearBufferedMoves(const char *tag)
{
    const char *label = (tag != NULL && tag[0] != '\0') ? tag : "clear";

    for (uint8_t id = 1U; id <= 4U; id++) {
        mb_result_t r = Task_MotorMovePosBufferRetry(id, MB_DIR_CW, 1U, 0U,
                                                     MB_MODE_REL_CUR, label);
        if (r != MB_OK) {
            printf("  %s buf m%d zero: %s\r\n",
                   label, (int)id, motor_result_str(r));
            (void)Task_StopAllImmediate("buffer-clear-fail");
            return r;
        }
        TASK_TRACE("  %s buf m%d zero: %s\r\n",
                   label, (int)id, motor_result_str(r));
        HAL_Delay(40U);
    }

    {
        mb_result_t tr = motor_sync_trigger();
        if (tr != MB_OK) {
            printf("  %s trigger: %s\r\n", label, motor_result_str(tr));
            return tr;
        }
        TASK_TRACE("  %s trigger: %s\r\n", label, motor_result_str(tr));
        Task_DelayService(120U);
    }

    return MB_OK;
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
        if (s_manual_jog_rehome_required != 0U) {
            printf("MANUAL return near zero but bus was uncertain; "
                   "clear buffered commands first\r\n");
            return Task_ManualClearBufferedMoves("manual-clear");
        }
        return MB_OK;
    }

    TASK_TRACE("MANUAL return delta=[%ld,%ld,%ld,%ld] max=%lu\r\n",
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
            scale = MOTION_MIN_SCALE;
        } else if (scale < MOTION_MIN_SCALE) {
            scale = MOTION_MIN_SCALE;
        }
        vmax = (uint16_t)((float)MANUAL_RETURN_BASE_VMAX * scale);
        if (vmax < 1U) {
            vmax = 1U;
        }

        r = Task_MotorMovePosBufferRetry(id, dir, vmax, mag,
                                         MB_MODE_REL_CUR, "manual");
        if (r != MB_OK) {
            printf("  manual buf m%d dir=%s vmax=%u mag=%lu: %s\r\n",
                   (int)id, (dir == MB_DIR_CW) ? "CW" : "CCW",
                   (unsigned int)vmax, (unsigned long)mag,
                   motor_result_str(r));
            for (uint8_t sid = 1U; sid <= 4U; sid++) {
                (void)motor_stop(sid, MB_SYNC_NOW);
                HAL_Delay(15);
            }
            return r;
        }
        TASK_TRACE("  manual buf m%d dir=%s vmax=%u mag=%lu: %s\r\n",
                   (int)id, (dir == MB_DIR_CW) ? "CW" : "CCW",
                   (unsigned int)vmax, (unsigned long)mag,
                   motor_result_str(r));
        HAL_Delay(40);
    }

    {
        mb_result_t tr = motor_sync_trigger();
        uint32_t move_ms;

        if (tr != MB_OK) {
            printf("  manual trigger: %s\r\n", motor_result_str(tr));
            return tr;
        }
        TASK_TRACE("  manual trigger: %s\r\n", motor_result_str(tr));

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

    TASK_TRACE("MANUAL return start cur=[%ld,%ld,%ld,%ld] zero=[%ld,%ld,%ld,%ld]\r\n",
               (long)current[0], (long)current[1],
               (long)current[2], (long)current[3],
               (long)s_manual_zero_pos[0], (long)s_manual_zero_pos[1],
               (long)s_manual_zero_pos[2], (long)s_manual_zero_pos[3]);

    if (max_abs <= (uint32_t)MANUAL_RETURN_TOL_COUNTS) {
        TASK_TRACE("MANUAL return skip: already near zero, visual fine zero follows\r\n");
        if (s_manual_jog_rehome_required != 0U &&
            Task_ManualClearBufferedMoves("manual-clear") != MB_OK) {
            OLED_Clear();
            OLED_ShowLine(0, "CLEAR BUF FAIL");
            OLED_ShowLine(2, "CHECK BUS");
            OLED_Refresh();
            printf("MANUAL return abort: clear buffered commands failed\r\n");
            return 0U;
        }
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
                if (s_manual_jog_rehome_required != 0U &&
                    Task_ManualClearBufferedMoves("manual-clear") != MB_OK) {
                    OLED_Clear();
                    OLED_ShowLine(0, "CLEAR BUF FAIL");
                    OLED_ShowLine(2, "CHECK BUS");
                    OLED_Refresh();
                    printf("MANUAL return abort: clear buffered commands failed\r\n");
                    return 0U;
                }
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

    TASK_TRACE("MANUAL return one-shot smooth max=%lu vmax=%u\r\n",
               (unsigned long)max_abs,
               (unsigned int)MANUAL_RETURN_BASE_VMAX);
    if (Task_ManualSendMotorDelta(total_delta) != MB_OK) {
        OLED_Clear();
        OLED_ShowLine(0, "RETURN MOVE FAIL");
        OLED_ShowLine(2, "CHECK BUS/TENSION");
        OLED_Refresh();
        return 0U;
    }

#if MANUAL_RETURN_POST_ENCODER_READS
    for (uint8_t id = 1U; id <= 4U; id++) {
        int32_t pos = 0;
        if (Task_ReadMotorPositionRetry(id, &pos) != 0U) {
            TASK_TRACE("MANUAL return m%d pos=%ld zero=%ld err=%ld\r\n",
                       (int)id, (long)pos,
                       (long)s_manual_zero_pos[id - 1U],
                       (long)(pos - s_manual_zero_pos[id - 1U]));
        }
        HAL_Delay(20);
    }
#endif

    s_manual_x = 0.0f;
    s_manual_y = 0.0f;
    motion_init();
    OLED_Clear();
    OLED_ShowLine(0, "RETURN MOTOR OK");
    OLED_ShowLine(2, "CAM FINE ZERO");
    OLED_Refresh();
    TASK_TRACE("MANUAL return motor zero done, visual fine zero follows\r\n");

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

static void Task_EdgePatrolMotionGains(const char *name,
                                       uint8_t circle_id,
                                       float *takeup_gain,
                                       float *payout_gain)
{
    uint8_t is_lb_job = 0U;

    if (takeup_gain == NULL || payout_gain == NULL) {
        return;
    }

    *takeup_gain = EDGE_PATROL_TAKEUP_GAIN;
    *payout_gain = EDGE_PATROL_PAYOUT_GAIN;

    if (circle_id == 4U) {
        is_lb_job = 1U;
    } else if (name != NULL && name[0] == 'L' && name[1] == 'B') {
        is_lb_job = 1U;
    }

    if (is_lb_job != 0U) {
        *takeup_gain = EDGE_PATROL_LB_TAKEUP_GAIN;
        *payout_gain = EDGE_PATROL_LB_PAYOUT_GAIN;
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
#if !APP_COMPACT_LOG
    uint32_t now = HAL_GetTick();
#endif
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
    TASK_TRACE("PATROL circle %s ref=(%.1f,%.1f) err=(%.2f,%.2f) d=%.2f",
               Task_EdgePatrolCircleName(circle_id),
               (double)matched_x, (double)matched_y,
               (double)err_x, (double)err_y,
               (double)dist);
    if (g_vision.tilt_count != 0U) {
        TASK_TRACE(" tilt=(%.2f,%.2f) age=%lums",
                   (double)g_vision.tilt_x, (double)g_vision.tilt_y,
                   (unsigned long)Task_VisionAge(now, g_vision.tilt_ms));
    }
    TASK_TRACE("\r\n");

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
#if EDGE_PATROL_USE_TARGET_SEED
    const EdgePatrolTargetSeed_t *seed;
#endif

    if (circle_id == 0U || circle_id > 4U || x == NULL || y == NULL) {
        return 0U;
    }

    cal = &s_edge_patrol_cal[circle_id];
    if (cal->count >= EDGE_PATROL_CAL_USE_MIN) {
        if (kin_in_range(cal->soft_x, cal->soft_y) == 0U) {
            printf("PATROL target learned %s ignored: avg=(%.2f,%.2f) out of range\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (double)cal->soft_x, (double)cal->soft_y);
        } else {
            *x = cal->soft_x;
            *y = cal->soft_y;
            return 1U;
        }
    }

#if EDGE_PATROL_USE_TARGET_SEED
    seed = &s_edge_patrol_target_seed[circle_id];
    if (seed->valid == 0U) {
        return 0U;
    }
    if (kin_in_range(seed->x, seed->y) == 0U) {
        printf("PATROL target seed %s ignored: seed=(%.2f,%.2f) out of range\r\n",
               Task_EdgePatrolCircleName(circle_id),
               (double)seed->x, (double)seed->y);
        return 0U;
    }

    *x = seed->x;
    *y = seed->y;
    return 2U;
#else
    return 0U;
#endif
}

static void Task_EdgePatrolSampleNode(uint8_t circle_id,
                                      const char *name,
                                      float err_x, float err_y)
{
    EdgePatrolCal_t *cal;
#if EDGE_PATROL_CAL_LOG
    const char *label;
#endif
#if EDGE_PATROL_CAL_ENCODER_READS
    int32_t pos[4] = {0};
    uint8_t enc_ok = 1U;
#endif
    uint8_t next_count;
    float denom;
#if EDGE_PATROL_CAL_LOG
    uint8_t tilt_ok = 0U;
#endif

    if (circle_id == 0U || circle_id > 4U) {
        return;
    }

    cal = &s_edge_patrol_cal[circle_id];
#if EDGE_PATROL_CAL_LOG
    label = (name != NULL) ? name : Task_EdgePatrolCircleName(circle_id);
#else
    (void)name;
#endif

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
#if EDGE_PATROL_CAL_LOG
        tilt_ok = 1U;
#endif
        cal->tilt_x += (g_vision.tilt_x - cal->tilt_x) / denom;
        cal->tilt_y += (g_vision.tilt_y - cal->tilt_y) / denom;
    }

#if EDGE_PATROL_CAL_ENCODER_READS
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
#endif

#if EDGE_PATROL_CAL_LOG
    printf("PATROL CAL %s n=%u soft=(%.2f,%.2f) err=(%.2f,%.2f)",
           label, (unsigned int)cal->count,
           (double)s_manual_x, (double)s_manual_y,
           (double)err_x, (double)err_y);
    if (tilt_ok != 0U) {
        printf(" tilt=(%.2f,%.2f)",
               (double)g_vision.tilt_x, (double)g_vision.tilt_y);
    } else {
        printf(" tilt=NA");
    }
#if EDGE_PATROL_CAL_ENCODER_READS
    if (enc_ok != 0U) {
        printf(" enc=[%ld,%ld,%ld,%ld] enc_n=%u",
               (long)pos[0], (long)pos[1], (long)pos[2], (long)pos[3],
               (unsigned int)cal->enc_count);
    } else {
        printf(" enc=READ_FAIL enc_n=%u",
               (unsigned int)cal->enc_count);
    }
#else
    printf(" enc=DISABLED");
#endif
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
#endif
}

static void Task_EdgePatrolPrintCalTable(void)
{
#if EDGE_PATROL_CAL_LOG
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
#endif
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
    uint8_t enc_deferred = 0U;
    const char *enc_source = "SEG";

    if (EDGE_PATROL_SEGMENT_ENCODER_READS == 0U) {
        enc_deferred = 1U;
    } else if (s_edge_patrol_prev_enc_mask == 0x0FU) {
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
    if (enc_deferred != 0U) {
        printf(" enc=DEFERRED_TO_CORNER");
    } else if (enc_ok != 0U) {
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

#if EDGE_PATROL_SEGMENT_LOG
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
#endif

static void Task_EdgePatrolResetSegmentEncoderBaseline(void)
{
    int32_t pos[4] = {0};
    mb_result_t result[4] = {MB_ERR_FRAME, MB_ERR_FRAME,
                             MB_ERR_FRAME, MB_ERR_FRAME};
    uint8_t mask = 0U;

    if (EDGE_PATROL_SEGMENT_ENCODER_READS == 0U) {
        s_edge_patrol_prev_enc_mask = 0U;
        for (uint8_t i = 0U; i < 4U; i++) {
            s_edge_patrol_prev_enc[i] = 0;
        }
        TASK_TRACE("PATROL SEG BASE enc=SAFETY_RETURN_ONLY\r\n");
        return;
    }

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
                TASK_TRACE("PATROL vision refresh: &mode,total# sent\r\n");
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
    motion_segment_report_t report;

    motion_get_last_segment_report(&report);

#if EDGE_PATROL_BUS_DIAG_LOG
    printf("DSEG seq=%lu name=%s step=%u/%u seg=%u/%u fine=%u "
           "result=%s done=%u from=(%.2f,%.2f) to=(%.2f,%.2f) "
           "mag=[%lu,%lu,%lu,%lu] vmax=[%u,%u,%u,%u] "
           "vision_err=(%.2f,%.2f) tilt=(%.2f,%.2f)\r\n",
           (unsigned long)report.sequence,
           (name != NULL) ? name : "?",
           (unsigned int)step, (unsigned int)total,
           (unsigned int)segment, (unsigned int)segment_total,
           (unsigned int)fine,
           motor_result_str(motion_result),
           (unsigned int)report.completed,
           (double)report.x_now, (double)report.y_now,
           (double)report.x_next, (double)report.y_next,
           (unsigned long)report.mag[0], (unsigned long)report.mag[1],
           (unsigned long)report.mag[2], (unsigned long)report.mag[3],
           (unsigned int)report.vmax[0], (unsigned int)report.vmax[1],
           (unsigned int)report.vmax[2], (unsigned int)report.vmax[3],
           (double)(g_vision.laser_x - g_vision.circle_x),
           (double)(g_vision.laser_y - g_vision.circle_y),
           (double)g_vision.tilt_x, (double)g_vision.tilt_y);

    for (uint8_t i = 0U; i < 4U; i++) {
        const motor_transaction_summary_t *diag = &report.motor_diag[i];
        if ((report.attempted_mask & (uint8_t)(1U << i)) == 0U) {
            printf("MBTX seg=%lu m=%u slot=0 result=SKIPPED\r\n",
                   (unsigned long)report.sequence,
                   (unsigned int)(i + 1U));
            continue;
        }
        printf("MBTX seg=%lu m=%u slot=%u id=%lu result=%s "
               "func=0x%02X reg=0x%04X qty=%u "
               "rx=%u/%u raw=%u off=%u "
               "t_us=[tx:%u first:%u last:%u] total=%ums "
               "exc=0x%02X uart=0x%08lX\r\n",
               (unsigned long)report.sequence,
               (unsigned int)(i + 1U),
               (unsigned int)report.bus_slot[i],
               (unsigned long)diag->transaction_id,
               motor_result_str(diag->result),
               (unsigned int)diag->function,
               (unsigned int)diag->request_register,
               (unsigned int)diag->request_quantity,
               (unsigned int)diag->rx_len,
               (unsigned int)diag->expected_rx_len,
               (unsigned int)diag->raw_rx_len,
               (unsigned int)diag->frame_offset,
               (unsigned int)diag->tx_time_us,
               (unsigned int)diag->first_rx_us,
               (unsigned int)diag->last_rx_us,
               (unsigned int)diag->elapsed_ms,
               (unsigned int)diag->exception_code,
               (unsigned long)diag->uart_error);
    }
#endif

#if EDGE_PATROL_SEGMENT_LOG
    int32_t enc_before[4] = {0};
    int32_t enc_after[4] = {0};
    int32_t enc_delta[4] = {0};
    mb_result_t enc_result[4] = {MB_ERR_FRAME, MB_ERR_FRAME,
                                 MB_ERR_FRAME, MB_ERR_FRAME};
    uint8_t enc_before_mask = s_edge_patrol_prev_enc_mask;
    uint8_t enc_after_mask = 0U;
    uint8_t enc_delta_mask = 0U;
    uint8_t enc_sampled = 0U;
    uint32_t now;
    const char *quality;

    for (uint8_t i = 0U; i < 4U; i++) {
        enc_before[i] = s_edge_patrol_prev_enc[i];
    }
    if (EDGE_PATROL_SEGMENT_ENCODER_READS != 0U) {
        enc_after_mask = Task_EdgePatrolReadEncOnce(enc_after, enc_result);
        enc_sampled = 1U;
    }
    now = HAL_GetTick();
    if (enc_sampled != 0U) {
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
    }

    if (motion_result != MB_OK || report.completed == 0U) {
        quality = "COMM_FAIL";
    } else if (enc_sampled != 0U && enc_after_mask != 0x0FU) {
        quality = "ENC_PARTIAL";
    } else if (g_vision.laser_count == 0U ||
               Task_VisionAge(now, g_vision.laser_ms) > HOME_VISION_AGE_MS) {
        quality = "VISION_STALE";
    } else if (g_vision.circle_count == 0U ||
               Task_VisionAge(now, g_vision.circle_ms) > HOME_VISION_AGE_MS) {
        quality = "CIRCLE_STALE";
    } else if (enc_sampled == 0U) {
        quality = "CMD_OK_VISION_FRESH_ENC_DEFERRED";
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
           ((report.attempted_mask & 0x01U) != 0U) ?
               motor_result_str(report.motor_result[0]) : "SKIPPED",
           ((report.attempted_mask & 0x02U) != 0U) ?
               motor_result_str(report.motor_result[1]) : "SKIPPED",
           ((report.attempted_mask & 0x04U) != 0U) ?
               motor_result_str(report.motor_result[2]) : "SKIPPED",
           ((report.attempted_mask & 0x08U) != 0U) ?
               motor_result_str(report.motor_result[3]) : "SKIPPED",
           motor_result_str(report.trigger_result),
           (unsigned int)report.trigger_attempted,
           (unsigned int)report.attempted_mask,
           (unsigned int)report.failed_motor);

    printf("PATROL SEG DATA seq=%lu tick=%lums ",
           (unsigned long)report.sequence, (unsigned long)now);
    if (enc_sampled != 0U) {
        Task_EdgePatrolPrintEncVector("enc_before", enc_before,
                                      enc_before_mask);
        printf(" ");
        Task_EdgePatrolPrintEncVector("enc_after", enc_after, enc_after_mask);
        printf(" ");
        Task_EdgePatrolPrintEncVector("denc", enc_delta, enc_delta_mask);
        printf(" enc_result=[%s,%s,%s,%s]",
               motor_result_str(enc_result[0]), motor_result_str(enc_result[1]),
               motor_result_str(enc_result[2]), motor_result_str(enc_result[3]));
    } else {
        printf("enc_sample=DEFERRED_TO_CORNER");
    }
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
     * The high-density diagnostic profile samples all four encoders here and
     * therefore needs a quiet window before the following buffered writes.
     * The default stable profile performs no segment read and reaches this
     * point after the same 300 ms settle used by visual homing.
     */
    if (motion_result == MB_OK && enc_sampled != 0U) {
        HAL_Delay(EDGE_PATROL_POST_READ_QUIET_MS);
    }
#else
    (void)release_limit;
#endif
}

static const char *Task_MotorTypeName(uint8_t type_code)
{
    static const char *const names[] = {
        "20", "28", "35", "42", "57", "86"
    };
    return (type_code < (uint8_t)(sizeof(names) / sizeof(names[0]))) ?
           names[type_code] : "UNKNOWN";
}

static void Task_MotorPrintProbeSummary(const char *item,
                                        uint8_t motor_id,
                                        const motor_transaction_summary_t *diag)
{
    if (diag == NULL) {
        return;
    }
    printf("MBPROBE item=%s m=%u id=%lu result=%s "
           "func=0x%02X reg=0x%04X qty=%u "
           "rx=%u/%u raw=%u off=%u "
           "t_us=[tx:%u first:%u last:%u] total=%ums "
           "exc=0x%02X uart=0x%08lX\r\n",
           (item != NULL) ? item : "?",
           (unsigned int)motor_id,
           (unsigned long)diag->transaction_id,
           motor_result_str(diag->result),
           (unsigned int)diag->function,
           (unsigned int)diag->request_register,
           (unsigned int)diag->request_quantity,
           (unsigned int)diag->rx_len,
           (unsigned int)diag->expected_rx_len,
           (unsigned int)diag->raw_rx_len,
           (unsigned int)diag->frame_offset,
           (unsigned int)diag->tx_time_us,
           (unsigned int)diag->first_rx_us,
           (unsigned int)diag->last_rx_us,
           (unsigned int)diag->elapsed_ms,
           (unsigned int)diag->exception_code,
           (unsigned long)diag->uart_error);
}

static uint8_t Task_MotorInventory(void)
{
    uint8_t safe = 1U;

    printf("MOTOR INVENTORY begin manual=ZDT-MODBUS-V1.0.0_251204 "
           "read_func=0x04 baud=115200 format=8N1\r\n");
    for (uint8_t id = 1U; id <= 4U; id++) {
        motor_identity_t identity;
        motor_transaction_summary_t version_diag;
        motor_transaction_summary_t voltage_diag;
        motor_transaction_summary_t heartbeat_diag;
        motor_transaction_summary_t state_diag;
        uint16_t millivolts = 0U;
        uint32_t heartbeat_ms = 0U;
        uint8_t state = 0U;
        uint8_t aux_fail_streak = 0U;
        uint8_t aux_fail_streak_max = 0U;
        mb_result_t version_result;
        mb_result_t voltage_result;
        mb_result_t heartbeat_result;
        mb_result_t state_result;

        version_result = motor_read_identity_x(id, &identity);
        motor_get_last_transaction_summary(&version_diag);
        Task_MotorPrintProbeSummary("VERSION_X", id, &version_diag);
        if (version_result != MB_OK) {
            motor_print_last_transaction_diag("MOTOR VERSION FAIL");
            printf("MOTOR ID m=%u fw=UNKNOWN hw=UNKNOWN "
                   "reason=%s patrol_safe=0\r\n",
                   (unsigned int)id,
                   motor_result_str(version_result));
            safe = 0U;
            Task_DelayService(120U);
            continue;
        }

        Task_DelayService(80U);
        voltage_result = motor_read_bus_voltage(id, &millivolts);
        motor_get_last_transaction_summary(&voltage_diag);
        Task_MotorPrintProbeSummary("VBUS", id, &voltage_diag);
        if (voltage_result != MB_OK) {
            motor_print_last_transaction_diag("MOTOR VBUS FAIL");
            aux_fail_streak = 1U;
            aux_fail_streak_max = 1U;
        } else {
            aux_fail_streak = 0U;
        }

        Task_DelayService(80U);
        heartbeat_result =
            motor_read_heartbeat_time_x(id, &heartbeat_ms);
        motor_get_last_transaction_summary(&heartbeat_diag);
        Task_MotorPrintProbeSummary("HEARTBEAT", id, &heartbeat_diag);
        if (heartbeat_result != MB_OK) {
            motor_print_last_transaction_diag("MOTOR HEARTBEAT FAIL");
            aux_fail_streak++;
            if (aux_fail_streak > aux_fail_streak_max) {
                aux_fail_streak_max = aux_fail_streak;
            }
        } else {
            aux_fail_streak = 0U;
        }

        Task_DelayService(80U);
        state_result = motor_read_state_x(id, &state);
        motor_get_last_transaction_summary(&state_diag);
        Task_MotorPrintProbeSummary("STATE_X", id, &state_diag);
        if (state_result != MB_OK) {
            motor_print_last_transaction_diag("MOTOR STATE FAIL");
            aux_fail_streak++;
            if (aux_fail_streak > aux_fail_streak_max) {
                aux_fail_streak_max = aux_fail_streak;
            }
        } else {
            aux_fail_streak = 0U;
        }

        printf("MOTOR ID m=%u fw_raw=%u hw_raw=0x%04X "
               "series=%s series_code=%u type=%s type_code=%u hw_ver=%u "
               "vbus_mV=",
               (unsigned int)id,
               (unsigned int)identity.firmware_raw,
               (unsigned int)identity.hardware_raw,
               (identity.series_code == 0U) ? "X" :
               ((identity.series_code == 1U) ? "Y" : "UNCONFIRMED"),
               (unsigned int)identity.series_code,
               Task_MotorTypeName(identity.type_code),
               (unsigned int)identity.type_code,
               (unsigned int)identity.hardware_version);
        if (voltage_result == MB_OK) {
            printf("%u", (unsigned int)millivolts);
        } else {
            printf("NA");
        }
        printf(" heartbeat_ms=");
        if (heartbeat_result == MB_OK) {
            printf("%lu", (unsigned long)heartbeat_ms);
        } else {
            printf("NA");
        }
        printf(" state=");
        if (state_result == MB_OK) {
            printf("0x%02X enabled=%u reached=%u stall=%u protect=%u "
                   "left_limit=%u right_limit=%u overcurrent=%u",
                   (unsigned int)state,
                   (unsigned int)((state >> 0) & 0x01U),
                   (unsigned int)((state >> 1) & 0x01U),
                   (unsigned int)((state >> 2) & 0x01U),
                   (unsigned int)((state >> 3) & 0x01U),
                   (unsigned int)((state >> 4) & 0x01U),
                   (unsigned int)((state >> 5) & 0x01U),
                   (unsigned int)((state >> 7) & 0x01U));
        } else {
            printf("NA");
        }
        printf("\r\n");

        if ((identity.type_code > 5U) ||
            (identity.firmware_raw == 0U)) {
            printf("MOTOR ID m=%u decode=UNCONFIRMED patrol_safe=0\r\n",
                   (unsigned int)id);
            safe = 0U;
        }
        if (identity.series_code > 1U) {
            printf("MOTOR ID m=%u series_code=%u manual=UNCONFIRMED "
                   "patrol_safe=UNCHANGED\r\n",
                   (unsigned int)id,
                   (unsigned int)identity.series_code);
        }
        if (heartbeat_result == MB_OK && heartbeat_ms != 0U) {
            printf("MOTOR ID m=%u heartbeat_nonzero=%lums "
                   "patrol_safe=0 (parameter not modified)\r\n",
                   (unsigned int)id, (unsigned long)heartbeat_ms);
            safe = 0U;
        }
        if (aux_fail_streak_max >= 2U) {
            printf("MOTOR ID m=%u consecutive_aux_read_fail=%u "
                   "patrol_safe=0\r\n", (unsigned int)id,
                   (unsigned int)aux_fail_streak_max);
            safe = 0U;
        }
        Task_DelayService(120U);
    }
    printf("MOTOR INVENTORY end patrol_safe=%u\r\n",
           (unsigned int)safe);
    Task_DelayService(300U);
    return safe;
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
    float takeup_gain;
    float payout_gain;

    if (circle_id == 0U || circle_id > 4U) {
        return 0U;
    }
    idx = (uint8_t)(circle_id - 1U);
    Task_EdgePatrolMotionGains(Task_EdgePatrolCircleName(circle_id),
                               circle_id,
                               &takeup_gain,
                               &payout_gain);

    float last_dist = -1.0f;

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
            TASK_TRACE("PATROL fine %s wait target unavailable k=%u\r\n",
                       Task_EdgePatrolCircleName(circle_id),
                       (unsigned int)iter);
            Task_DelayService(250U);
            continue;
        }

        if (Task_MatchKnownVisionCircle(circle_x, circle_y, &matched_x, &matched_y) == 0U ||
            fabsf(matched_x - ref_x[idx]) > 0.1f ||
            fabsf(matched_y - ref_y[idx]) > 0.1f) {
            TASK_TRACE("PATROL fine %s skip: seen_other circle=(%.1f,%.1f) err=(%.2f,%.2f)\r\n",
                       Task_EdgePatrolCircleName(circle_id),
                       (double)circle_x, (double)circle_y,
                       (double)err_x, (double)err_y);
            Task_DelayService(250U);
            continue;
        }

        dist = sqrtf(err_x * err_x + err_y * err_y);
        TASK_TRACE("PATROL fine %s k=%u err=(%.2f,%.2f) d=%.2f soft=(%.2f,%.2f)\r\n",
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
        if (last_dist >= 0.0f &&
            dist > (last_dist + EDGE_PATROL_FINE_WORSE_ABORT_CM)) {
            printf("PATROL fine %s abort: error diverged prev=%.2f now=%.2f "
                   "soft=(%.2f,%.2f)\r\n",
                   Task_EdgePatrolCircleName(circle_id),
                   (double)last_dist, (double)dist,
                   (double)s_manual_x, (double)s_manual_y);
            return 0U;
        }
        last_dist = dist;

        fine_step = (dist < EDGE_PATROL_FINE_STEP_CM) ?
                    dist : EDGE_PATROL_FINE_STEP_CM;
        if (g_vision.tilt_count != 0U &&
            Task_VisionAge(HAL_GetTick(), g_vision.tilt_ms) <= HOME_VISION_AGE_MS &&
            (fabsf(g_vision.tilt_x) >= EDGE_PATROL_TILT_SLOW ||
             fabsf(g_vision.tilt_y) >= EDGE_PATROL_TILT_SLOW)) {
            fine_step *= 0.5f;
            TASK_TRACE("PATROL fine %s tilt slow tilt=(%.2f,%.2f) step=%.2f\r\n",
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
            TASK_TRACE("PATROL fine cmd %s seg=%u/%u "
                       "from=(%.2f,%.2f)->(%.2f,%.2f) "
                       "gain(T=%.2f P=%.2f)\r\n",
                       Task_EdgePatrolCircleName(circle_id),
                       (unsigned int)(k + 1U), (unsigned int)segments,
                       (double)s_manual_x, (double)s_manual_y,
                       (double)sx, (double)sy,
                       (double)takeup_gain, (double)payout_gain);
            r = motion_move_between_nohome_patrol(s_manual_x, s_manual_y,
                                                  sx, sy,
                                                  takeup_gain,
                                                  payout_gain);
            if (r != MB_OK) {
                s_edge_patrol_last_move_result = r;
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
    float takeup_gain;
    float payout_gain;

    s_edge_patrol_last_move_result = MB_OK;

    if (wp == NULL) {
        return 0U;
    }
    Task_EdgePatrolMotionGains(wp->name, wp->circle_id,
                               &takeup_gain, &payout_gain);
    target_x = wp->x;
    target_y = wp->y;
    if (wp->circle_id != 0U) {
        uint8_t target_source = Task_EdgePatrolGetLearnedTarget(wp->circle_id,
                                                                &target_x,
                                                                &target_y);
        if (target_source == 1U) {
            TASK_TRACE("PATROL target learned %s ideal=(%.2f,%.2f)->avg=(%.2f,%.2f) n=%u\r\n",
                       Task_EdgePatrolCircleName(wp->circle_id),
                       (double)wp->x, (double)wp->y,
                       (double)target_x, (double)target_y,
                       (unsigned int)s_edge_patrol_cal[wp->circle_id].count);
#if EDGE_PATROL_USE_TARGET_SEED
        } else if (target_source == 2U) {
            const EdgePatrolTargetSeed_t *seed =
                &s_edge_patrol_target_seed[wp->circle_id];
            TASK_TRACE("PATROL target seed %s ideal=(%.2f,%.2f)->fit=(%.2f,%.2f) n=%u source=%s\r\n",
                       Task_EdgePatrolCircleName(wp->circle_id),
                       (double)wp->x, (double)wp->y,
                       (double)target_x, (double)target_y,
                       (unsigned int)seed->samples,
                       seed->source);
#endif
        }
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
        TASK_TRACE("PATROL cmd %s step=%u/%u seg=%u/%u "
                   "from=(%.2f,%.2f)->(%.2f,%.2f) "
                   "gain(T=%.2f P=%.2f) rel<=%.1f\r\n",
                   (wp->name != NULL) ? wp->name : "?",
                   (unsigned int)step, (unsigned int)total,
                   (unsigned int)(k + 1U), (unsigned int)segments,
                   (double)s_manual_x, (double)s_manual_y,
                   (double)xn, (double)yn,
                   (double)takeup_gain,
                   (double)payout_gain,
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
                                              takeup_gain,
                                              payout_gain);
        if (r != MB_OK) {
            s_edge_patrol_last_move_result = r;
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
        } else if (s_manual_jog_rehome_required != 0U) {
            printf("PATROL target %s fine move left bus state uncertain; "
                   "rehome/retry required before continuing\r\n",
                   Task_EdgePatrolCircleName(wp->circle_id));
            return 0U;
        } else {
            s_edge_patrol_unconfirmed_mask |=
                (uint8_t)(1U << (wp->circle_id - 1U));
            OLED_ShowLine(5, "CIRCLE MISS");
            printf("PATROL target %s unconfirmed; visual calibration "
                   "sample NOT recorded\r\n",
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

    if (s_manual_jog_rehome_required == 0U &&
        fabsf(s_manual_x) <= EDGE_PATROL_SOFT_CENTER_CM &&
        fabsf(s_manual_y) <= EDGE_PATROL_SOFT_CENTER_CM) {
        TASK_TRACE("PATROL auto zero: soft near center, visual fine zero only\r\n");
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

    TASK_TRACE("PATROL auto zero: motor return first\r\n");
    if (Task_ManualReturnZero() == 0U) {
        printf("PATROL auto zero abort: motor return failed\r\n");
        return 0U;
    }

    if (s_manual_return_last_skipped != 0U) {
        OLED_Clear();
        OLED_ShowLine(0, "PATROL AUTO 0");
        OLED_ShowLine(2, "CAM FINE ZERO");
        OLED_Refresh();

        TASK_TRACE("PATROL auto zero: already at motor zero, run visual fine zero\r\n");
        if (Task_ManualAutoCenterAndRecord(1U) == 0U) {
            printf("PATROL auto zero abort: visual fine zero failed\r\n");
            return 0U;
        }
    } else {
        TASK_TRACE("PATROL auto zero: motor return included visual fine zero\r\n");
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

static uint8_t Task_EdgePatrolRecoverAfterMoveFail(const char *name,
                                                   uint8_t step,
                                                   uint8_t total,
                                                   mb_result_t result)
{
    const char *label = (name != NULL && name[0] != '\0') ? name : "?";

    printf("PATROL move recovery after %s step=%u/%u result=%s: "
           "stop all, wait %lums, motor return + visual zero, retry from known center\r\n",
           label,
           (unsigned int)step,
           (unsigned int)total,
           motor_result_str(result),
           (unsigned long)EDGE_PATROL_MOVE_RECOVERY_WAIT_MS);

    OLED_Clear();
    OLED_ShowLine(0, "PATROL RECOVER");
    OLED_ShowLine(2, "STOP + REHOME");
    OLED_ShowLine(5, "RETRY MAX 3");
    OLED_Refresh();

    (void)Task_StopAllImmediate("PATROL recovery");
    Task_DelayService(EDGE_PATROL_MOVE_RECOVERY_WAIT_MS);

    if (Task_ManualReturnZero() == 0U) {
        printf("PATROL move recovery failed after %s: rehome failed\r\n",
               label);
        return 0U;
    }

    if (vision_send_mode("total") == 0U) {
        printf("PATROL warn: &mode,total# send failed after move recovery\r\n");
    } else {
        TASK_TRACE("PATROL cmd: &mode,total# sent after move recovery\r\n");
    }
    Task_DelayService(300U);
    Task_EdgePatrolResetSegmentEncoderBaseline();

    printf("PATROL recovery OK after %s step=%u/%u\r\n",
           label, (unsigned int)step, (unsigned int)total);
    return 1U;
}

static uint8_t Task_EdgePatrolRunCornerJob(const EdgePatrolCornerJob_t *job,
                                           uint8_t first_step,
                                           uint8_t total_steps)
{
    if (job == NULL || job->waypoints == NULL || job->waypoint_count == 0U) {
        printf("PATROL corner job abort: invalid job\r\n");
        return 0U;
    }

    for (uint8_t attempt = 0U;
         attempt <= EDGE_PATROL_MOVE_RECOVERY_RETRY_MAX;
         attempt++) {
        uint8_t completed = 1U;

        if (attempt == 0U) {
            printf("PATROL corner job %s start: from center, waypoints=%u\r\n",
                   job->name, (unsigned int)job->waypoint_count);
        } else {
            printf("PATROL corner job %s restart attempt=%u/%u from center\r\n",
                   job->name,
                   (unsigned int)attempt,
                   (unsigned int)EDGE_PATROL_MOVE_RECOVERY_RETRY_MAX);
        }

        for (uint8_t i = 0U; i < job->waypoint_count; i++) {
            uint8_t step = (uint8_t)(first_step + i);
            const EdgePatrolWaypoint_t *wp = &job->waypoints[i];

            if (Task_EdgePatrolMoveTo(wp, step, total_steps) == 0U) {
                completed = 0U;

                if (s_manual_jog_rehome_required == 0U ||
                    attempt >= EDGE_PATROL_MOVE_RECOVERY_RETRY_MAX) {
                    printf("PATROL abort corner %s at waypoint %s step=%u/%u "
                           "result=%s rehome_required=%u\r\n",
                           job->name,
                           (wp->name != NULL) ? wp->name : "?",
                           (unsigned int)step,
                           (unsigned int)total_steps,
                           motor_result_str(s_edge_patrol_last_move_result),
                           (unsigned int)s_manual_jog_rehome_required);
                    return 0U;
                }

                printf("PATROL corner %s recovery request at waypoint %s "
                       "step=%u/%u retry=%u/%u result=%s; "
                       "restart this corner from center\r\n",
                       job->name,
                       (wp->name != NULL) ? wp->name : "?",
                       (unsigned int)step,
                       (unsigned int)total_steps,
                       (unsigned int)(attempt + 1U),
                       (unsigned int)EDGE_PATROL_MOVE_RECOVERY_RETRY_MAX,
                       motor_result_str(s_edge_patrol_last_move_result));

                if (Task_EdgePatrolRecoverAfterMoveFail(
                        wp->name, step, total_steps,
                        s_edge_patrol_last_move_result) == 0U) {
                    printf("PATROL abort: corner %s recovery failed at "
                           "waypoint %s step=%u/%u\r\n",
                           job->name,
                           (wp->name != NULL) ? wp->name : "?",
                           (unsigned int)step,
                           (unsigned int)total_steps);
                    return 0U;
                }
                break;
            }
        }

        if (completed != 0U) {
            uint8_t mask = (uint8_t)(1U << (job->circle_id - 1U));

            if ((s_edge_patrol_confirmed_mask & mask) != 0U) {
                TASK_TRACE("PATROL corner job %s target confirmed; "
                           "return center before next corner\r\n",
                           job->name);
            } else {
                TASK_TRACE("PATROL corner job %s target unconfirmed; "
                           "return center and continue isolated sequence\r\n",
                           job->name);
            }
            return 1U;
        }
    }

    return 0U;
}

static uint8_t Task_EdgePatrolResetAfterCorner(const EdgePatrolCornerJob_t *job,
                                               uint8_t job_step,
                                               uint8_t job_total)
{
    const char *label = (job != NULL && job->name != NULL) ? job->name : "?";
    uint8_t confirmed = 0U;

    if (job != NULL && job->circle_id >= 1U && job->circle_id <= 4U) {
        uint8_t mask = (uint8_t)(1U << (job->circle_id - 1U));
        confirmed = ((s_edge_patrol_confirmed_mask & mask) != 0U) ? 1U : 0U;
    }

    printf("PATROL center return after %s job=%u/%u confirmed=%u\r\n",
           label, (unsigned int)job_step, (unsigned int)job_total,
           (unsigned int)confirmed);

    OLED_Clear();
    OLED_ShowLine(0, "CORNER DONE");
    OLED_ShowLine(2, "RETURN CENTER");
    OLED_ShowLine(5, label);
    OLED_Refresh();

    if (Task_EdgePatrolAutoZeroWithRecovery(label) == 0U) {
        printf("PATROL abort: corner reset failed after %s\r\n", label);
        return 0U;
    }

    if (vision_send_mode("total") == 0U) {
        printf("PATROL warn: &mode,total# send failed after %s reset\r\n",
               label);
    } else {
        TASK_TRACE("PATROL cmd: &mode,total# sent after %s reset\r\n", label);
    }
    Task_DelayService(300U);
    Task_EdgePatrolResetSegmentEncoderBaseline();

    printf("PATROL center OK after %s\r\n", label);
    return 1U;
}

static uint8_t Task_EdgePatrolRun(void)
{
    uint8_t job_total = EDGE_PATROL_ORDER_COUNT;
    uint8_t total_steps = 0U;
    uint8_t first_step = 1U;
    char order_line[22];

    for (uint8_t j = 0U; j < job_total; j++) {
        const EdgePatrolCornerJob_t *job = Task_EdgePatrolOrderedJob(j);
        if (job == NULL) {
            printf("PATROL abort: invalid selected order slot %u\r\n",
                   (unsigned int)j);
            return 0U;
        }
        total_steps = (uint8_t)(total_steps + job->waypoint_count);
    }

    if (s_manual_zero_valid == 0U) {
        printf("PATROL abort: motor zero not saved\r\n");
        return 0U;
    }

    Task_EdgePatrolFormatOrder(s_edge_patrol_order,
                               EDGE_PATROL_ORDER_COUNT,
                               order_line, sizeof(order_line));
    printf("PATROL start order=%s target=%.1fcm rx=%s "
           "log=%s enc=%s\r\n",
           order_line, (double)EDGE_PATROL_TARGET_CM,
           motor_rx_mode_name(),
           (APP_COMPACT_LOG != 0U) ? "COMPACT" : "DETAIL",
           (EDGE_PATROL_SEGMENT_ENCODER_READS != 0U ||
            EDGE_PATROL_CAL_ENCODER_READS != 0U ||
            MANUAL_RETURN_POST_ENCODER_READS != 0U) ?
           "DIAG" : "RETURN_ONLY");
    printf("PATROL motion vmax=%u settle=%lums write=%s "
           "LB(T=%.2f P=%.2f)\r\n",
           (unsigned int)MOTION_PATROL_BASE_VMAX,
           (unsigned long)EDGE_PATROL_SEG_SETTLE_MS,
#if MOTION_PATROL_M2_FIRST
           "M2>M1>M3>M4",
#else
           "M1>M2>M3>M4",
#endif
           (double)EDGE_PATROL_LB_TAKEUP_GAIN,
           (double)EDGE_PATROL_LB_PAYOUT_GAIN);
    s_edge_patrol_prev_enc_mask = 0U;
    s_edge_patrol_confirmed_mask = 0U;
    s_edge_patrol_unconfirmed_mask = 0U;

    if (Task_MotorInventory() == 0U) {
        printf("PATROL abort: motor identity/bus preflight unsafe; "
               "no synchronized trigger sent\r\n");
        OLED_Clear();
        OLED_ShowLine(0, "BUS PREFLIGHT");
        OLED_ShowLine(2, "FAIL - SEE LOG");
        OLED_Refresh();
        return 0U;
    }

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
        TASK_TRACE("PATROL cmd: &mode,total# sent\r\n");
    }
    Task_DelayService(300U);
    Task_EdgePatrolResetSegmentEncoderBaseline();

    for (uint8_t j = 0U; j < job_total; j++) {
        const EdgePatrolCornerJob_t *job = Task_EdgePatrolOrderedJob(j);

        if (job == NULL) {
            printf("PATROL abort: invalid selected order slot %u\r\n",
                   (unsigned int)j);
            Task_EdgePatrolPrintCalTable();
            return 0U;
        }

        if (Task_EdgePatrolRunCornerJob(job, first_step,
                                        total_steps) == 0U) {
            printf("PATROL abort at corner job %s %u/%u\r\n",
                   job->name,
                   (unsigned int)(j + 1U),
                   (unsigned int)job_total);
            Task_EdgePatrolPrintCalTable();
            return 0U;
        }

        if (Task_EdgePatrolResetAfterCorner(job,
                                            (uint8_t)(j + 1U),
                                            job_total) == 0U) {
            Task_EdgePatrolPrintCalTable();
            return 0U;
        }

        first_step = (uint8_t)(first_step + job->waypoint_count);
    }

    printf("PATROL corner jobs done, final visual center check\r\n");
    OLED_Clear();
    OLED_ShowLine(0, "PATROL JOBS OK");
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
            if (s_edge_order_menu_active != 0U) {
                s_edge_order_menu_active = 0U;
            }
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

        if (Task_EdgeOrderModeKeyEdge() != 0U) {
            if (s_edge_order_menu_active != 0U) {
                Task_EdgeOrderExit();
            } else {
                Task_EdgeOrderEnter();
            }
            break;
        }

        if (s_edge_order_menu_active != 0U) {
            if (Task_EdgeOrderMenuService() != 0U) {
                Task_ManualJogStopForce();
                s_key0_last = Key0_Raw();
                if (Task_EdgePatrolRun() != 0U) {
                    s_state = TS_MANUAL_JOG;
                    Task_ShowManualJog();
                } else {
                    s_state = TS_MANUAL_JOG;
                    Task_DelayService(800U);
                    Task_ShowManualJog();
                }
                s_edge_patrol_key_last = ManualKey_IsPressed(&s_edge_patrol_key);
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
        OLED_Menu_Service();
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
            TASK_TRACE("ROUGH src=CIRCLE_GLOBAL ref=(%.1f,%.1f) err=(%.2f,%.2f) x=%.2f y=%.2f\r\n",
                       (double)ref_x, (double)ref_y,
                       (double)err_x, (double)err_y,
                       (double)*x, (double)*y);
            return 1U;
        }
    }

    if (vision_read_filtered(x, y) != 0U) {
        TASK_TRACE("ROUGH src=LASER_GLOBAL x=%.2f y=%.2f\r\n",
                   (double)*x, (double)*y);
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
        TASK_TRACE("HOME src=HOME_ERR x=%.2f y=%.2f\r\n",
                   (double)*x, (double)*y);
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
        TASK_TRACE("HOME src=LASER-CIRCLE x=%.2f y=%.2f\r\n",
                   (double)*x, (double)*y);
        return 1U;
    }

    if (g_vision.laser_count == 0U && g_vision.circle_count != 0U) {
        if (vision_send_mode("total") != 0U) {
            TASK_TRACE("HOME fallback cmd: &mode,total# sent, waiting LASER-CIRCLE\r\n");
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
                TASK_TRACE("HOME src=TOTAL LASER-CIRCLE x=%.2f y=%.2f\r\n",
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
            TASK_TRACE("ROUGH skip: fine target visible d=%.2f\r\n",
                       (double)fine_dist);
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
    TASK_TRACE("ROUGH global x=%.2f y=%.2f d=%.2f center=%d\r\n",
               (double)xg, (double)yg, (double)dist, (int)center_seen);

    if (dist <= HOME_ROUGH_DONE_CM) {
        TASK_TRACE("ROUGH done: already near center circle\r\n");
        return 1U;
    }

    x0 = xg;
    y0 = yg;
    Task_ClampNoHomeCalcPoint(&x0, &y0);
    calc_dist = sqrtf(x0 * x0 + y0 * y0);
    if (calc_dist <= HOME_ROUGH_DONE_CM) {
        TASK_TRACE("ROUGH done by calc point\r\n");
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

    TASK_TRACE("ROUGH open start calc=(%.2f,%.2f) target=(%.2f,%.2f) move=%.2f steps=%d\r\n",
               (double)x0, (double)y0,
               (double)target_x, (double)target_y,
               (double)move_dist, (int)steps);

    prev_x = x0;
    prev_y = y0;
    for (uint8_t k = 0; k < steps; k++) {
        float t = (float)(k + 1U) / (float)steps;
        float xn = x0 + (target_x - x0) * t;
        float yn = y0 + (target_y - y0) * t;

        TASK_TRACE("ROUGH open cmd k=%d from=(%.2f,%.2f)->(%.2f,%.2f)\r\n",
                   (int)k, (double)prev_x, (double)prev_y,
                   (double)xn, (double)yn);

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

    TASK_TRACE("ROUGH open done\r\n");
    return 1U;
}

static uint8_t Task_ReturnCenter_ByVision(void)
{
    char line[22];
    uint8_t centered = 0U;
    float last_dist = -1.0f;
    uint8_t stale_count = 0U;
#if !APP_COMPACT_LOG
    uint8_t has_last_cmd = 0U;
    float last_cmd_x = 0.0f;
    float last_cmd_y = 0.0f;
    float last_tgt_x = 0.0f;
    float last_tgt_y = 0.0f;
#endif

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
        TASK_TRACE("HOME k=%d x=%.2f y=%.2f d=%.2f\r\n",
                   (int)k, (double)xm, (double)ym, (double)dist);

        if (last_dist >= 0.0f) {
            float improve = last_dist - dist;
#if !APP_COMPACT_LOG
            if (has_last_cmd != 0U) {
                TASK_TRACE("HOME obs dx=%.2f dy=%.2f  exp dx=%.2f dy=%.2f\r\n",
                           (double)(xm - last_cmd_x),
                           (double)(ym - last_cmd_y),
                           (double)(last_tgt_x - last_cmd_x),
                           (double)(last_tgt_y - last_cmd_y));
            }
#endif
            TASK_TRACE("HOME improve=%.2f last=%.2f now=%.2f\r\n",
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

        TASK_TRACE("HOME cmd k=%d target=(%.2f,%.2f) step=%.2f\r\n",
                   (int)k, (double)xn, (double)yn, (double)step);
#if !APP_COMPACT_LOG
        last_cmd_x = xm;
        last_cmd_y = ym;
        last_tgt_x = xn;
        last_tgt_y = yn;
        has_last_cmd = 1U;
#endif

        mb_result_t move_result;
        if (dist > HOME_RELAX_FINE_CM) {
            TASK_TRACE("HOME move mode=RELAX gain(T=%.2f P=%.2f)\r\n",
                       (double)HOME_RELAX_TAKEUP_GAIN,
                       (double)HOME_RELAX_PAYOUT_GAIN);
            move_result = motion_move_between_nohome_relaxed(xm, ym, xn, yn,
                                                             HOME_RELAX_TAKEUP_GAIN,
                                                             HOME_RELAX_PAYOUT_GAIN);
        } else {
            TASK_TRACE("HOME move mode=EXACT\r\n");
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

    OLED_Menu_Init();
#if TASK_MANUAL_ZERO_MODE
    s_state = TS_MANUAL_SET_ZERO;
    s_last_menu_tick = HAL_GetTick();
    s_key0_last = Key0_Raw();
    Task_ManualKeysInit();
    Task_ShowManualZeroPrompt();

    printf("CDPR ready rx=%s log=%s enc=%s; "
           "KEY0 zero, PA0 patrol, KEY1 order\r\n",
           motor_rx_mode_name(),
           (APP_COMPACT_LOG != 0U) ? "COMPACT" : "DETAIL",
           (EDGE_PATROL_SEGMENT_ENCODER_READS != 0U ||
            EDGE_PATROL_CAL_ENCODER_READS != 0U ||
            MANUAL_RETURN_POST_ENCODER_READS != 0U) ?
           "DIAG" : "RETURN_ONLY");
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
    OLED_Menu_Service();

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
