/* ============================================================
 *  motion.c — 协调运动控制 (商家式: 先全部缓存, 再广播一起触发)
 *  发命令时四个电机都不动 -> 总线干净, 不会因"边动边发"冲坏命令
 * ============================================================ */
#include "motion.h"
#include "kinematics.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "vision.h"

#if MOTION_COMPACT_LOG
#define MOTION_TRACE(...) do { } while (0)
#else
#define MOTION_TRACE(...) printf(__VA_ARGS__)
#endif

static float cur_x = 0.0f, cur_y = 0.0f;
static float cur_L[4];
static float home_L[4];      /* 中心(home)处绳长, 编码器在此清零 */
static motion_segment_report_t s_last_segment_report;
static uint32_t s_segment_sequence = 0U;

void motion_get_last_segment_report(motion_segment_report_t *report)
{
    if (report != NULL) {
        *report = s_last_segment_report;
    }
}

static void motion_stop_all_now(void)
{
    for (uint8_t id = 1U; id <= 4U; id++) {
        (void)motor_stop(id, MB_SYNC_NOW);
        HAL_Delay(15);
    }
}

static void motion_delay_service(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < delay_ms) {
        vision_task();
        HAL_Delay(5);
    }
}

#ifndef MOTION_BUFFER_RETRY_MAX
#define MOTION_BUFFER_RETRY_MAX      3U
#endif
#ifndef MOTION_BUFFER_RETRY_DELAY_MS
#define MOTION_BUFFER_RETRY_DELAY_MS 90U
#endif
#ifndef MOTION_PATROL_BALANCE_RADIUS_CM
#define MOTION_PATROL_BALANCE_RADIUS_CM 8.0f
#endif
#ifndef MOTION_PATROL_DOM_PAYOUT_RATIO
#define MOTION_PATROL_DOM_PAYOUT_RATIO 0.70f
#endif
#ifndef MOTION_PATROL_DOM_PAYOUT_GAIN
#define MOTION_PATROL_DOM_PAYOUT_GAIN 1.00f
#endif
#ifndef MOTION_PATROL_SIDE_PAYOUT_GAIN
#define MOTION_PATROL_SIDE_PAYOUT_GAIN 1.00f
#endif
#ifndef MOTION_PATROL_BALANCE_ENABLE
/*
 * The former 0.86/1.03 heuristic intentionally changed the geometric
 * payout.  The RB log showed a 2.442762 cm cumulative M4 command deficit,
 * and the cable has since been reported broken.  The old values remain in
 * the handoff/evidence documents for offline comparison, but this
 * unverified tension heuristic is not applied in field motion.
 */
#define MOTION_PATROL_BALANCE_ENABLE 0U
#endif
#ifndef MOTION_PATROL_BASE_VMAX
#define MOTION_PATROL_BASE_VMAX 300U
#endif
/*
 * Per-motor empirical scale hooks. Defaults are neutral so the existing
 * field behavior is preserved; later calibration can change one motor
 * without applying a blind global tension offset.
 */
#ifndef MOTION_PATROL_M1_PAYOUT_SCALE
#define MOTION_PATROL_M1_PAYOUT_SCALE 1.00f
#endif
#ifndef MOTION_PATROL_M2_PAYOUT_SCALE
#define MOTION_PATROL_M2_PAYOUT_SCALE 1.00f
#endif
#ifndef MOTION_PATROL_M3_PAYOUT_SCALE
#define MOTION_PATROL_M3_PAYOUT_SCALE 1.00f
#endif
#ifndef MOTION_PATROL_M4_PAYOUT_SCALE
#define MOTION_PATROL_M4_PAYOUT_SCALE 1.00f
#endif

static float motion_patrol_payout_scale(uint8_t index)
{
    switch (index) {
    case 0U: return MOTION_PATROL_M1_PAYOUT_SCALE;
    case 1U: return MOTION_PATROL_M2_PAYOUT_SCALE;
    case 2U: return MOTION_PATROL_M3_PAYOUT_SCALE;
    case 3U: return MOTION_PATROL_M4_PAYOUT_SCALE;
    default: return 1.0f;
    }
}

static mb_result_t motion_buffer_move_pos_retry(uint8_t id,
                                                uint8_t dir,
                                                uint16_t vmax,
                                                uint32_t mag,
                                                uint8_t mode,
                                                const char *tag)
{
    mb_result_t r = MB_ERR_TIMEOUT;

    for (uint8_t attempt = 0U; attempt < MOTION_BUFFER_RETRY_MAX; attempt++) {
        r = motor_move_pos(id, dir, vmax, mag, mode, MB_SYNC_BUFFER);
        if (r == MB_OK) {
            if (attempt != 0U && tag != NULL && tag[0] != '\0') {
                printf("  %s buf m%d retry ok attempt=%u\r\n",
                       tag, (int)id, (unsigned int)(attempt + 1U));
            }
            return MB_OK;
        }

        if (tag != NULL && tag[0] != '\0') {
            printf("  %s buf m%d attempt=%u %s\r\n",
                   tag, (int)id,
                   (unsigned int)(attempt + 1U),
                   motor_result_str(r));
        }

        /*
         * A relative buffered write may already have been accepted when its
         * response is lost or corrupted.  Re-sending CRC/TIMEOUT/REJECT
         * blindly can queue the same motion twice. HAL_BUSY is the only result
         * known to occur before this blocking transfer starts; all other
         * results are ambiguous and must abort this synchronized group.
         */
        if (r != MB_ERR_BUSY) {
            motor_print_last_transaction_diag("  buffer failure");
            break;
        }
        HAL_Delay(MOTION_BUFFER_RETRY_DELAY_MS);
    }

    if (r == MB_ERR_BUSY) {
        motor_print_last_transaction_diag("  buffer uart busy");
    }
    return r;
}

void motion_init(void)
{
    cur_x = 0.0f; cur_y = 0.0f;
    kin_ik(0.0f, 0.0f, cur_L);
    for (int i = 0; i < 4; i++) home_L[i] = cur_L[i];
}

mb_result_t motion_enable_all(uint8_t on)
{
    mb_result_t r = MB_OK;
    for (uint8_t id = 1; id <= 4; id++) {
        mb_result_t ri = motor_enable(id, on, MB_SYNC_NOW);
        if (ri != MB_OK) r = ri;
        HAL_Delay(20);
    }
    return r;
}

mb_result_t motion_set_home(void)
{
    for (uint8_t id = 1; id <= 4; id++) {
        mb_result_t r = MB_ERR_TIMEOUT;

        for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
            r = motor_zero_position(id);
            if (r == MB_OK) {
                break;
            }
            printf("  m%d zero retry %u: %s\r\n",
                   id, (unsigned int)(attempt + 1U), motor_result_str(r));
            HAL_Delay(40);
        }
        if (r != MB_OK) {
            printf("  m%d zero FAIL: %s -> home NOT set\r\n", id, motor_result_str(r));
            return r;                       /* 某个清零失败 -> 不更新基准 */
        }
        HAL_Delay(20);
    }
    cur_x = 0.0f; cur_y = 0.0f;
    kin_ik(0.0f, 0.0f, cur_L);
    for (int i = 0; i < 4; i++) home_L[i] = cur_L[i];
    return MB_OK;
}

void motion_get_pos(float *x, float *y) { *x = cur_x; *y = cur_y; }

/* 加了异常保护+回读门控后, REL_LAST 满足"只有校验成功才更新"的前提。
 * 想对比可改成 MB_MODE_REL_CUR (前提:你的 0x00F0 固件接受模式 0x02)。 */
#ifndef MOTION_MOVE_MODE
#define MOTION_MOVE_MODE          MB_MODE_REL_LAST
#endif
/* 回读校验容差(0.1°计数): 正常误差只有±1~2, 真丢步会差几千, 300 足够区分。 */
#ifndef MOTION_VERIFY_TOL_COUNTS
#define MOTION_VERIFY_TOL_COUNTS  300
#endif

#ifndef MOTION_NOHOME_BASE_VMAX
#define MOTION_NOHOME_BASE_VMAX   300    /* 视觉回中心/精修阶段速度更低: 30.0RPM */
#endif
#ifndef MOTION_NOHOME_INTER_CMD_DELAY_MS
/*
 * V11 proved the slower patrol/M2 pacing can pass many consecutive segments,
 * but center visual re-zero still used the old 20 ms nohome gap and stopped
 * at CENTER-RT on an M2 rx_len=0 TIMEOUT.  Match nohome visual-zero pacing to
 * patrol pacing; this changes only bus timing, not commanded rope length.
 */
#define MOTION_NOHOME_INTER_CMD_DELAY_MS 160U
#endif
#ifndef MOTION_NOHOME_READBACK
#define MOTION_NOHOME_READBACK    0
#endif
#ifndef MOTION_MANUAL_JOG_BASE_VMAX
#define MOTION_MANUAL_JOG_BASE_VMAX 600
#endif
#ifndef MOTION_MANUAL_JOG_INTER_CMD_DELAY_MS
#define MOTION_MANUAL_JOG_INTER_CMD_DELAY_MS 120U
#endif
#ifndef MOTION_PATROL_INTER_CMD_DELAY_MS
/*
 * Patrol sends four buffered writes automatically and then repeats the same
 * transaction group after four encoder reads.  V8 field logs showed true
 * no-reply timeouts (rx_len=0, 251 ms) on different motor addresses while
 * operator-paced jog remained stable.  Keep jog pacing unchanged and give
 * only the denser patrol sequence more drive/bus recovery time.
 */
#define MOTION_PATROL_INTER_CMD_DELAY_MS 160U
#endif
#ifndef MOTION_M2_PRE_CMD_EXTRA_DELAY_MS
/*
 * V10 field logs still show full-window no-reply timeouts on M2 0x10 buffered
 * writes after the previous motor returned OK.  Give M2 a dedicated quiet
 * window without making relative writes retryable after TIMEOUT/CRC/FRAME.
 */
#define MOTION_M2_PRE_CMD_EXTRA_DELAY_MS 120U
#endif
#ifndef MOTION_M2_POST_CMD_EXTRA_DELAY_MS
#define MOTION_M2_POST_CMD_EXTRA_DELAY_MS 80U
#endif
#ifndef MOTION_JOG_SPEED_MIN_0P1RPM
#define MOTION_JOG_SPEED_MIN_0P1RPM 60U
#endif
#ifndef MOTION_JOG_SPEED_MAX_0P1RPM
#define MOTION_JOG_SPEED_MAX_0P1RPM 800U
#endif
#ifndef MOTION_JOG_ACCEL
#define MOTION_JOG_ACCEL 20U
#endif
#define MOTION_JOG_DIFF_DT_S 0.10f

static void motion_pre_buffer_delay(uint8_t id)
{
    if (id == 2U && MOTION_M2_PRE_CMD_EXTRA_DELAY_MS != 0U) {
        motion_delay_service(MOTION_M2_PRE_CMD_EXTRA_DELAY_MS);
    }
}

static void motion_post_buffer_delay(uint8_t id)
{
    if (id == 2U && MOTION_M2_POST_CMD_EXTRA_DELAY_MS != 0U) {
        motion_delay_service(MOTION_M2_POST_CMD_EXTRA_DELAY_MS);
    }
}

mb_result_t motion_move_to(float x, float y)
{
    if (!kin_in_range(x, y)) { printf("REJECT (%.1f,%.1f) out of range\r\n", x, y); return MB_ERR_REJECT; }

    float Lt[4], dL[4];
    kin_ik(x, y, Lt);

    float dmax = 0.0f;
    for (int i = 0; i < 4; i++) { dL[i] = Lt[i] - cur_L[i]; float a = fabsf(dL[i]); if (a > dmax) dmax = a; }
    MOTION_TRACE("move(%.1f,%.1f) dL=%.2f %.2f %.2f %.2f  dmax=%.2f\r\n",
                 x, y, dL[0], dL[1], dL[2], dL[3], dmax);
    if (dmax < 0.01f) {
        MOTION_TRACE("  skip\r\n");
        return MB_OK;
    }

    /* 1) 缓存四条命令; 任何一条失败 -> 不触发, 直接返回 */
    for (int i = 0; i < 4; i++) {
        uint8_t  id  = (uint8_t)(i + 1);
        uint8_t  dir = motor_cable_direction(id, (dL[i] < 0.0f) ?
                                              MOTOR_CABLE_TAKEUP : MOTOR_CABLE_PAYOUT);
        uint32_t mag = (uint32_t)(fabsf(dL[i]) * KIN_COUNTS_PER_CM + 0.5f);
        float scale  = fabsf(dL[i]) / dmax; if (scale < MOTION_MIN_SCALE) scale = MOTION_MIN_SCALE;
        uint16_t vmax = (uint16_t)(MOTION_BASE_VMAX * scale); if (vmax < 1) vmax = 1;

        motion_pre_buffer_delay(id);
        mb_result_t rr = motion_buffer_move_pos_retry(id, dir, vmax, mag,
                                                      MOTION_MOVE_MODE, "move");
        if (rr != MB_OK) {
            printf("  buf m%d: %s\r\n", id, motor_result_str(rr));
            printf("  buffer FAIL -> stop all, abort\r\n");
            motion_stop_all_now();
            return rr;
        }
        MOTION_TRACE("  buf m%d: %s\r\n", id, motor_result_str(rr));
        motion_delay_service(20U);
        motion_post_buffer_delay(id);
    }

    /* 2) 广播触发, 判返回值 */
    mb_result_t tr = motor_sync_trigger();
    if (tr != MB_OK) {
        printf("  trigger FAIL: %s\r\n", motor_result_str(tr));
        motion_stop_all_now();
        return tr;
    }

    /* 3) 估时等待 (运动期间不读总线) */
    uint32_t mag_max = (uint32_t)(dmax * KIN_COUNTS_PER_CM + 0.5f);
    uint32_t move_ms = (uint32_t)(((float)mag_max / (MOTION_BASE_VMAX * 6.0f)) * 1000.0f) + 1500;
    if (move_ms < 800) move_ms = 800;
    if (move_ms > 12000) move_ms = 12000;
    motion_delay_service(move_ms);

    /* 4) 停稳后回读校验; 任一读失败或超差 -> 不更新模型 */
    int32_t max_abs_err = 0; uint8_t read_ok = 1;
    for (uint8_t id = 1; id <= 4; id++) {
        int32_t p = 0;
        mb_result_t pr = motor_read_position(id, &p);
        if (pr != MB_OK) { printf("  m%d read FAIL: %s\r\n", id, motor_result_str(pr)); read_ok = 0; continue; }
        int32_t exp = (int32_t)((home_L[id-1] - Lt[id-1]) * KIN_COUNTS_PER_CM);
        int32_t e = p - exp, ae = (e < 0) ? -e : e;
        if (ae > max_abs_err) max_abs_err = ae;
        MOTION_TRACE("  m%d pos=%ld  exp=%ld  (err=%ld)\r\n", id, p, exp, e);
        HAL_Delay(20);
    }
    if (!read_ok || max_abs_err > MOTION_VERIFY_TOL_COUNTS) {
        printf("  VERIFY FAIL (max|err|=%ld) -> model NOT updated\r\n", max_abs_err);
        return MB_ERR_REJECT;                       /* 不更新 cur_x/cur_y/cur_L */
    }

    /* 5) 只有校验通过才更新模型 */
    cur_x = x; cur_y = y;
    for (int i = 0; i < 4; i++) cur_L[i] = Lt[i];
    return MB_OK;
}

#define FT_MAX_STEPS  5
#define FT_TOL_CM     1.0f      /* 误差阈值, 给 ±2cm 留余量 */
#define FT_GAIN       0.7f      /* 欠松弛, 防过冲 */
#define FT_SETTLE_MS  300

/* 目标(xt,yt) 已被 A 粗定位到附近; 返回 1=收敛到阈值内 */
uint8_t fine_tune_to(float xt, float yt)
{
    if (!vision_is_online(500)) return 0;             /* 视觉掉线 -> 放弃精修, 用 A 的结果(降级) */

    for (int k = 0; k < FT_MAX_STEPS; k++) {
        motion_delay_service(FT_SETTLE_MS);           /* 停稳 */
        float xm, ym;
        if (!vision_read_filtered(&xm, &ym)) return 0;
        float ex = xt - xm, ey = yt - ym;
        float e  = sqrtf(ex*ex + ey*ey);
        MOTION_TRACE("  fine k=%d meas(%.2f,%.2f) err=%.2f\r\n", k, xm, ym, e);
        if (e < FT_TOL_CM) return 1;                  /* 误差<阈值 -> 完成 */

        /* 世界误差 -> 绳长修正: 让吊舱从实测位走到目标位(欠松弛) */
        float xc = xm + FT_GAIN*ex, yc = ym + FT_GAIN*ey;
    if (motion_move_between_nohome(xm, ym, xc, yc) != MB_OK) return 0;
    }
    return 0;                                         /* 没收敛: 记一笔, 用最后测量值当坐标 */
}

static mb_result_t motion_move_between_nohome_scaled_ex(float x_now, float y_now,
                                                        float x_next, float y_next,
                                                        float takeup_gain,
                                                        float payout_gain,
                                                        uint16_t base_vmax,
                                                        uint32_t wait_extra_ms,
                                                        uint32_t wait_min_ms,
                                                        uint32_t wait_max_ms,
                                                        uint32_t inter_cmd_delay_ms,
                                                        const char *tag)
{
    memset(&s_last_segment_report, 0, sizeof(s_last_segment_report));
    s_last_segment_report.sequence = ++s_segment_sequence;
    s_last_segment_report.x_now = x_now;
    s_last_segment_report.y_now = y_now;
    s_last_segment_report.x_next = x_next;
    s_last_segment_report.y_next = y_next;
    s_last_segment_report.trigger_result = MB_ERR_FRAME;
    for (uint8_t i = 0U; i < 4U; i++) {
        s_last_segment_report.motor_result[i] = MB_ERR_FRAME;
    }

    uint8_t verbose = (tag != NULL && tag[0] != '\0') ? 1U : 0U;

    if (takeup_gain <= 0.0f) takeup_gain = 1.0f;
    if (payout_gain <= 0.0f) payout_gain = 1.0f;
    if (base_vmax == 0U) base_vmax = MOTION_NOHOME_BASE_VMAX;

    if (!kin_in_range(x_now, y_now) || !kin_in_range(x_next, y_next)) {
        printf("NOHOME REJECT now(%.1f,%.1f) next(%.1f,%.1f)\r\n",
               x_now, y_now, x_next, y_next);
        return MB_ERR_REJECT;
    }

    float L0[4], L1[4], dL[4];
    float cmd_cm[4];
#if MOTION_NOHOME_READBACK
    uint8_t cmd_dir[4] = {0};
    uint32_t cmd_mag[4] = {0};
    int32_t pos_before[4] = {0};
    uint8_t pos_before_ok[4] = {0};
#endif
    kin_ik(x_now,  y_now,  L0);
    kin_ik(x_next, y_next, L1);

    float dmax = 0.0f;
    float cmd_dmax = 0.0f;
    float max_payout = 0.0f;
    uint8_t patrol_balance = 0U;

    if (MOTION_PATROL_BALANCE_ENABLE != 0U &&
        tag != NULL && strcmp(tag, "patrol") == 0 &&
        sqrtf(x_next * x_next + y_next * y_next) >= MOTION_PATROL_BALANCE_RADIUS_CM) {
        patrol_balance = 1U;
    }

    for (int i = 0; i < 4; i++) {
        dL[i] = L1[i] - L0[i];
        s_last_segment_report.dL[i] = dL[i];
        float a = fabsf(dL[i]);
        if (a > dmax) dmax = a;
        if (dL[i] > max_payout) max_payout = dL[i];
    }

    for (int i = 0; i < 4; i++) {
        float gain = (dL[i] < 0.0f) ? takeup_gain : payout_gain;
        float a = fabsf(dL[i]);

        if (patrol_balance != 0U && dL[i] > 0.0f && max_payout > 0.01f) {
            if (dL[i] >= (max_payout * MOTION_PATROL_DOM_PAYOUT_RATIO)) {
                gain *= MOTION_PATROL_DOM_PAYOUT_GAIN;
            } else {
                gain *= MOTION_PATROL_SIDE_PAYOUT_GAIN;
            }
            gain *= motion_patrol_payout_scale((uint8_t)i);
        }

        s_last_segment_report.gain[i] = gain;
        cmd_cm[i] = a * gain;
        s_last_segment_report.cmd_cm[i] = cmd_cm[i];
        if (cmd_cm[i] > cmd_dmax) cmd_dmax = cmd_cm[i];
    }
    s_last_segment_report.bal = patrol_balance;

    if (verbose != 0U) {
        MOTION_TRACE("%s move %.1f %.1f -> %.1f %.1f dL=%.2f %.2f %.2f %.2f cmd=%.2f %.2f %.2f %.2f gain(T=%.2f P=%.2f) bal=%u\r\n",
                     tag, x_now, y_now, x_next, y_next,
                     dL[0], dL[1], dL[2], dL[3],
                     (double)cmd_cm[0], (double)cmd_cm[1],
                     (double)cmd_cm[2], (double)cmd_cm[3],
                     (double)takeup_gain, (double)payout_gain,
                     (unsigned int)patrol_balance);
    }

    if (dmax < 0.01f || cmd_dmax < 0.01f) {
        s_last_segment_report.completed = 1U;
        s_last_segment_report.trigger_result = MB_OK;
        return MB_OK;
    }

#if MOTION_NOHOME_READBACK
    for (uint8_t id = 1; id <= 4; id++) {
        mb_result_t pr = motor_read_position(id, &pos_before[id - 1U]);
        pos_before_ok[id - 1U] = (pr == MB_OK) ? 1U : 0U;
        if (pr != MB_OK) {
            printf("  home m%d before read: %s\r\n", id, motor_result_str(pr));
        }
        HAL_Delay(15);
    }
#endif

    for (uint8_t slot = 0U; slot < 4U; slot++) {
        uint8_t i = slot;

#if MOTION_PATROL_M2_FIRST
        /*
         * Patrol-only bus order. M2 historically failed most often while it
         * occupied the second transaction slot. Put it first after the long
         * motion/settle quiet period without changing motor IDs, magnitudes,
         * speeds, or the final synchronized trigger.
         */
        if (tag != NULL && strcmp(tag, "patrol") == 0) {
            static const uint8_t patrol_order[4] = {1U, 0U, 2U, 3U};
            i = patrol_order[slot];
        }
#endif

        uint8_t id  = (uint8_t)(i + 1);
        uint8_t dir = motor_cable_direction(id, (dL[i] < 0.0f) ?
                                             MOTOR_CABLE_TAKEUP : MOTOR_CABLE_PAYOUT);
        uint32_t mag = (uint32_t)(cmd_cm[i] * KIN_COUNTS_PER_CM + 0.5f);
        s_last_segment_report.dir[i] = dir;
        s_last_segment_report.mag[i] = mag;
#if MOTION_NOHOME_READBACK
        cmd_dir[i] = dir;
        cmd_mag[i] = mag;
#endif

        float scale = cmd_cm[i] / cmd_dmax;
        if (scale < MOTION_MIN_SCALE) scale = MOTION_MIN_SCALE;

        uint16_t vmax = (uint16_t)((float)base_vmax * scale);
        if (vmax < 1) vmax = 1;
        s_last_segment_report.vmax[i] = vmax;

        motion_pre_buffer_delay(id);
        mb_result_t r = motion_buffer_move_pos_retry(id, dir, vmax, mag,
                                                     MB_MODE_REL_CUR, "home");
        s_last_segment_report.attempted_mask |= (uint8_t)(1U << i);
        s_last_segment_report.motor_result[i] = r;
        s_last_segment_report.bus_slot[i] = (uint8_t)(slot + 1U);
        motor_get_last_transaction_summary(
            &s_last_segment_report.motor_diag[i]);
        if (r != MB_OK) {
            printf("  home buf m%d: %s\r\n", id, motor_result_str(r));
        } else if (verbose != 0U) {
            MOTION_TRACE("  home buf m%d: %s\r\n", id, motor_result_str(r));
        }
        if (r != MB_OK) {
            s_last_segment_report.failed_motor = id;
            printf("  home buffer FAIL -> stop all, abort\r\n");
            motion_stop_all_now();
            return r;
        }

        motion_delay_service(inter_cmd_delay_ms);
        motion_post_buffer_delay(id);
    }

    mb_result_t tr = motor_sync_trigger();
    s_last_segment_report.trigger_attempted = 1U;
    s_last_segment_report.trigger_result = tr;
    if (tr != MB_OK) {
        printf("  home trigger: %s\r\n", motor_result_str(tr));
    } else if (verbose != 0U) {
        MOTION_TRACE("  home trigger: %s\r\n", motor_result_str(tr));
    }
    if (tr != MB_OK) {
        motor_print_last_transaction_diag("  trigger failure");
        motion_stop_all_now();
        return tr;
    }

    uint32_t mag_max = (uint32_t)(cmd_dmax * KIN_COUNTS_PER_CM + 0.5f);
    uint32_t move_ms = (uint32_t)(((float)mag_max / ((float)base_vmax * 6.0f)) * 1000.0f) + wait_extra_ms;
    if (move_ms < wait_min_ms) move_ms = wait_min_ms;
    if (move_ms > wait_max_ms) move_ms = wait_max_ms;
    s_last_segment_report.estimated_move_ms = move_ms;

    motion_delay_service(move_ms);

#if MOTION_NOHOME_READBACK
    for (uint8_t id = 1; id <= 4; id++) {
        int32_t pos_after = 0;
        mb_result_t pr = motor_read_position(id, &pos_after);
        if ((pr == MB_OK) && (pos_before_ok[id - 1U] != 0U)) {
            int32_t actual = pos_after - pos_before[id - 1U];
            int32_t expected = (cmd_dir[id - 1U] == MB_DIR_CW) ?
                               (int32_t)cmd_mag[id - 1U] :
                               -(int32_t)cmd_mag[id - 1U];
            int32_t err = actual - expected;
            if (verbose != 0U) {
                MOTION_TRACE("  home m%d delta=%ld exp=%ld err=%ld\r\n",
                             id, (long)actual, (long)expected, (long)err);
            }
        } else {
            printf("  home m%d after read: %s\r\n", id, motor_result_str(pr));
        }
        HAL_Delay(15);
    }
#endif

    cur_x = x_next;
    cur_y = y_next;
    for (int i = 0; i < 4; i++) {
        cur_L[i] = L1[i];
    }
    s_last_segment_report.completed = 1U;

    return MB_OK;
}

static mb_result_t motion_move_between_nohome_scaled(float x_now, float y_now,
                                                     float x_next, float y_next,
                                                     float takeup_gain,
                                                     float payout_gain,
                                                     const char *tag)
{
    return motion_move_between_nohome_scaled_ex(x_now, y_now, x_next, y_next,
                                                takeup_gain, payout_gain,
                                                MOTION_NOHOME_BASE_VMAX,
                                                1000U, 500U, 8000U,
                                                MOTION_NOHOME_INTER_CMD_DELAY_MS,
                                                tag);
}

mb_result_t motion_move_between_nohome(float x_now, float y_now,
                                       float x_next, float y_next)
{
    return motion_move_between_nohome_scaled(x_now, y_now, x_next, y_next,
                                             1.0f, 1.0f, "nohome");
}

mb_result_t motion_move_between_nohome_relaxed(float x_now, float y_now,
                                               float x_next, float y_next,
                                               float takeup_gain,
                                               float payout_gain)
{
    return motion_move_between_nohome_scaled(x_now, y_now, x_next, y_next,
                                             takeup_gain, payout_gain,
                                             "nohome relaxed");
}

mb_result_t motion_move_between_nohome_relaxed_quick(float x_now, float y_now,
                                                     float x_next, float y_next,
                                                     float takeup_gain,
                                                     float payout_gain)
{
    return motion_move_between_nohome_scaled_ex(x_now, y_now, x_next, y_next,
                                                takeup_gain, payout_gain,
                                                MOTION_MANUAL_JOG_BASE_VMAX,
                                                80U, 90U, 500U, 5U,
                                                NULL);
}

mb_result_t motion_move_between_nohome_patrol(float x_now, float y_now,
                                              float x_next, float y_next,
                                              float takeup_gain,
                                              float payout_gain)
{
    return motion_move_between_nohome_scaled_ex(x_now, y_now, x_next, y_next,
                                                takeup_gain, payout_gain,
                                                MOTION_PATROL_BASE_VMAX,
                                                1200U, 700U, 12000U,
                                                MOTION_PATROL_INTER_CMD_DELAY_MS,
                                                "patrol");
}

mb_result_t motion_jog_step_start(float x_now, float y_now,
                                  float x_next, float y_next,
                                  float takeup_gain,
                                  float payout_gain,
                                  uint32_t *move_ms)
{
    float L0[4];
    float L1[4];
    float dL[4];
    float cmd_cm[4];
    float dmax = 0.0f;
    float cmd_dmax = 0.0f;

    if (move_ms != NULL) {
        *move_ms = 0U;
    }
    if (takeup_gain <= 0.0f) takeup_gain = 1.0f;
    if (payout_gain <= 0.0f) payout_gain = 1.0f;

    if (kin_in_range(x_now, y_now) == 0U || kin_in_range(x_next, y_next) == 0U) {
        return MB_ERR_REJECT;
    }

    kin_ik(x_now, y_now, L0);
    kin_ik(x_next, y_next, L1);

    for (uint8_t i = 0U; i < 4U; i++) {
        float gain;
        dL[i] = L1[i] - L0[i];
        if (fabsf(dL[i]) > dmax) {
            dmax = fabsf(dL[i]);
        }
        gain = (dL[i] < 0.0f) ? takeup_gain : payout_gain;
        cmd_cm[i] = fabsf(dL[i]) * gain;
        if (cmd_cm[i] > cmd_dmax) {
            cmd_dmax = cmd_cm[i];
        }
    }

    if (dmax < 0.01f || cmd_dmax < 0.01f) {
        return MB_OK;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t id = (uint8_t)(i + 1U);
        uint8_t dir = motor_cable_direction(id, (dL[i] < 0.0f) ?
                                            MOTOR_CABLE_TAKEUP :
                                            MOTOR_CABLE_PAYOUT);
        uint32_t mag = (uint32_t)(cmd_cm[i] * KIN_COUNTS_PER_CM + 0.5f);
        float scale = cmd_cm[i] / cmd_dmax;
        uint16_t vmax;
        mb_result_t r;

        if (scale < MOTION_MIN_SCALE) {
            scale = MOTION_MIN_SCALE;
        }
        vmax = (uint16_t)((float)MOTION_MANUAL_JOG_BASE_VMAX * scale);
        if (vmax < 1U) {
            vmax = 1U;
        }

        motion_pre_buffer_delay(id);
        r = motion_buffer_move_pos_retry(id, dir, vmax, mag,
                                         MB_MODE_REL_CUR, "jog");
        if (r != MB_OK) {
            motion_stop_all_now();
            return r;
        }
        motion_delay_service(MOTION_MANUAL_JOG_INTER_CMD_DELAY_MS);
        motion_post_buffer_delay(id);
    }

    {
        mb_result_t tr = motor_sync_trigger();
        uint32_t mag_max = (uint32_t)(cmd_dmax * KIN_COUNTS_PER_CM + 0.5f);
        uint32_t ms = (uint32_t)(((float)mag_max / ((float)MOTION_MANUAL_JOG_BASE_VMAX * 6.0f)) * 1000.0f) + 80U;

        if (tr != MB_OK) {
            motion_stop_all_now();
            return tr;
        }
        if (ms < 120U) ms = 120U;
        if (ms > 4000U) ms = 4000U;
        if (move_ms != NULL) {
            *move_ms = ms;
        }
    }

    return MB_OK;
}

static uint16_t motion_jog_rope_speed_to_0p1rpm(float rope_cm_s)
{
    float speed = fabsf(rope_cm_s) * KIN_COUNTS_PER_CM / 6.0f;

    if (speed < 0.5f) {
        return 0U;
    }
    if (speed < (float)MOTION_JOG_SPEED_MIN_0P1RPM) {
        speed = (float)MOTION_JOG_SPEED_MIN_0P1RPM;
    }
    if (speed > (float)MOTION_JOG_SPEED_MAX_0P1RPM) {
        speed = (float)MOTION_JOG_SPEED_MAX_0P1RPM;
    }

    return (uint16_t)(speed + 0.5f);
}

mb_result_t motion_jog_velocity_at(float x_now, float y_now,
                                   float vx_cm_s, float vy_cm_s)
{
    float x_next = x_now + vx_cm_s * MOTION_JOG_DIFF_DT_S;
    float y_next = y_now + vy_cm_s * MOTION_JOG_DIFF_DT_S;
    float L0[4];
    float L1[4];

    if (fabsf(vx_cm_s) < 0.01f && fabsf(vy_cm_s) < 0.01f) {
        return motion_jog_stop();
    }

    if (kin_in_range(x_now, y_now) == 0U || kin_in_range(x_next, y_next) == 0U) {
        return MB_ERR_REJECT;
    }

    kin_ik(x_now, y_now, L0);
    kin_ik(x_next, y_next, L1);

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t id = (uint8_t)(i + 1U);
        float rope_cm_s = (L1[i] - L0[i]) / MOTION_JOG_DIFF_DT_S;
        uint16_t speed = motion_jog_rope_speed_to_0p1rpm(rope_cm_s);
        uint8_t dir = motor_cable_direction(id, (rope_cm_s < 0.0f) ?
                                            MOTOR_CABLE_TAKEUP :
                                            MOTOR_CABLE_PAYOUT);
        mb_result_t r = motor_run_speed(id, dir, speed, MOTION_JOG_ACCEL, MB_SYNC_BUFFER);

        if (r != MB_OK) {
            (void)motion_jog_stop();
            return r;
        }
        HAL_Delay(15);
    }

    return motor_sync_trigger();
}

mb_result_t motion_jog_stop(void)
{
    mb_result_t result = MB_OK;

    for (uint8_t id = 1U; id <= 4U; id++) {
        mb_result_t r = motor_run_speed(id, MB_DIR_CW, 0U, MOTION_JOG_ACCEL, MB_SYNC_BUFFER);
        if (r != MB_OK) {
            result = r;
        }
        HAL_Delay(15);
    }

    if (result == MB_OK) {
        return motor_sync_trigger();
    }

    for (uint8_t id = 1U; id <= 4U; id++) {
        (void)motor_stop(id, MB_SYNC_NOW);
        HAL_Delay(15);
    }

    return result;
}
