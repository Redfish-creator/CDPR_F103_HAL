/* ============================================================
 *  motion.c — 协调运动控制 (商家式: 先全部缓存, 再广播一起触发)
 *  发命令时四个电机都不动 -> 总线干净, 不会因"边动边发"冲坏命令
 * ============================================================ */
#include "motion.h"
#include "kinematics.h"
#include <math.h>
#include <stdio.h>
#include "vision.h"
#include <stdlib.h>     /* qsort */
static float cur_x = 0.0f, cur_y = 0.0f;
static float cur_L[4];
static float home_L[4];      /* 中心(home)处绳长, 编码器在此清零 */

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
        mb_result_t r = motor_zero_position(id);
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

mb_result_t motion_move_to(float x, float y)
{
    if (!kin_in_range(x, y)) { printf("REJECT (%.1f,%.1f) out of range\r\n", x, y); return MB_ERR_REJECT; }

    float Lt[4], dL[4];
    kin_ik(x, y, Lt);

    float dmax = 0.0f;
    for (int i = 0; i < 4; i++) { dL[i] = Lt[i] - cur_L[i]; float a = fabsf(dL[i]); if (a > dmax) dmax = a; }
    printf("move(%.1f,%.1f) dL=%.2f %.2f %.2f %.2f  dmax=%.2f\r\n", x, y, dL[0], dL[1], dL[2], dL[3], dmax);
    if (dmax < 0.01f) { printf("  skip\r\n"); return MB_OK; }

    /* 1) 缓存四条命令; 任何一条失败 -> 不触发, 直接返回 */
    for (int i = 0; i < 4; i++) {
        uint8_t  id  = (uint8_t)(i + 1);
        uint8_t  dir = (dL[i] < 0.0f) ? MB_DIR_CW : MB_DIR_CCW;          /* CW=收线 */
        uint32_t mag = (uint32_t)(fabsf(dL[i]) * KIN_COUNTS_PER_CM + 0.5f);
        float scale  = fabsf(dL[i]) / dmax; if (scale < MOTION_MIN_SCALE) scale = MOTION_MIN_SCALE;
        uint16_t vmax = (uint16_t)(MOTION_BASE_VMAX * scale); if (vmax < 1) vmax = 1;

        mb_result_t rr = motor_move_pos(id, dir, vmax, mag, MOTION_MOVE_MODE, MB_SYNC_BUFFER);
        printf("  buf m%d: %s\r\n", id, motor_result_str(rr));
        if (rr != MB_OK) { printf("  buffer FAIL -> NO trigger, abort\r\n"); return rr; }   /* 关键 */
        HAL_Delay(20);
    }

    /* 2) 广播触发, 判返回值 */
    mb_result_t tr = motor_sync_trigger();
    if (tr != MB_OK) { printf("  trigger FAIL: %s\r\n", motor_result_str(tr)); return tr; }

    /* 3) 估时等待 (运动期间不读总线) */
    uint32_t mag_max = (uint32_t)(dmax * KIN_COUNTS_PER_CM + 0.5f);
    uint32_t move_ms = (uint32_t)(((float)mag_max / (MOTION_BASE_VMAX * 6.0f)) * 1000.0f) + 1500;
    if (move_ms < 800) move_ms = 800; if (move_ms > 12000) move_ms = 12000;
    HAL_Delay(move_ms);

    /* 4) 停稳后回读校验; 任一读失败或超差 -> 不更新模型 */
    int32_t max_abs_err = 0; uint8_t read_ok = 1;
    for (uint8_t id = 1; id <= 4; id++) {
        int32_t p = 0;
        mb_result_t pr = motor_read_position(id, &p);
        if (pr != MB_OK) { printf("  m%d read FAIL: %s\r\n", id, motor_result_str(pr)); read_ok = 0; continue; }
        int32_t exp = (int32_t)((home_L[id-1] - Lt[id-1]) * KIN_COUNTS_PER_CM);
        int32_t e = p - exp, ae = (e < 0) ? -e : e;
        if (ae > max_abs_err) max_abs_err = ae;
        printf("  m%d pos=%ld  exp=%ld  (err=%ld)\r\n", id, p, exp, e);
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

#define HOME_MAX_ITER   8
#define HOME_TOL_CM     1.0f      /* 进这个圈就算到中心 */
#define HOME_STEP_CM    5.0f      /* 每步朝中心最多走 5cm(防止远处一把猛冲) */



/* 目标(xt,yt) 已被 A 粗定位到附近; 返回 1=收敛到阈值内 */
uint8_t fine_tune_to(float xt, float yt)
{
    if (!vision_is_online(500)) return 0;             /* 视觉掉线 -> 放弃精修, 用 A 的结果(降级) */

    for (int k = 0; k < FT_MAX_STEPS; k++) {
        HAL_Delay(FT_SETTLE_MS);                      /* 停稳 */
        float xm, ym;
        if (!vision_read_filtered(&xm, &ym)) return 0;
        float ex = xt - xm, ey = yt - ym;
        float e  = sqrtf(ex*ex + ey*ey);
        printf("  fine k=%d meas(%.2f,%.2f) err=%.2f\r\n", k, xm, ym, e);
        if (e < FT_TOL_CM) return 1;                  /* 误差<阈值 -> 完成 */

        /* 世界误差 -> 绳长修正: 让吊舱从实测位走到目标位(欠松弛) */
        float xc = xm + FT_GAIN*ex, yc = ym + FT_GAIN*ey;
    if (motion_move_between_nohome(xm, ym, xc, yc) != MB_OK) return 0;
    }
    return 0;                                         /* 没收敛: 记一笔, 用最后测量值当坐标 */
}

mb_result_t motion_move_between_nohome(float x_now, float y_now,
                                       float x_next, float y_next)
{
    if (!kin_in_range(x_now, y_now) || !kin_in_range(x_next, y_next)) {
        printf("NOHOME REJECT now(%.1f,%.1f) next(%.1f,%.1f)\r\n",
               x_now, y_now, x_next, y_next);
        return MB_ERR_REJECT;
    }

    float L0[4], L1[4], dL[4];
    kin_ik(x_now,  y_now,  L0);
    kin_ik(x_next, y_next, L1);

    float dmax = 0.0f;
    for (int i = 0; i < 4; i++) {
        dL[i] = L1[i] - L0[i];
        float a = fabsf(dL[i]);
        if (a > dmax) dmax = a;
    }

    printf("nohome move %.1f %.1f -> %.1f %.1f dL=%.2f %.2f %.2f %.2f\r\n",
           x_now, y_now, x_next, y_next, dL[0], dL[1], dL[2], dL[3]);

    if (dmax < 0.01f) return MB_OK;

    for (int i = 0; i < 4; i++) {
        uint8_t id  = (uint8_t)(i + 1);
        uint8_t dir = (dL[i] < 0.0f) ? MB_DIR_CW : MB_DIR_CCW;
        uint32_t mag = (uint32_t)(fabsf(dL[i]) * KIN_COUNTS_PER_CM + 0.5f);

        float scale = fabsf(dL[i]) / dmax;
        if (scale < MOTION_MIN_SCALE) scale = MOTION_MIN_SCALE;

        uint16_t vmax = (uint16_t)(MOTION_BASE_VMAX * scale);
        if (vmax < 1) vmax = 1;

        mb_result_t r = motor_move_pos(id, dir, vmax, mag,
                                       MB_MODE_REL_CUR, MB_SYNC_BUFFER);
        printf("  home buf m%d: %s\r\n", id, motor_result_str(r));
        if (r != MB_OK) return r;

        HAL_Delay(20);
    }

    mb_result_t tr = motor_sync_trigger();
    if (tr != MB_OK) return tr;

    uint32_t mag_max = (uint32_t)(dmax * KIN_COUNTS_PER_CM + 0.5f);
    uint32_t move_ms = (uint32_t)(((float)mag_max / (MOTION_BASE_VMAX * 6.0f)) * 1000.0f) + 800;
    if (move_ms < 500) move_ms = 500;
    if (move_ms > 5000) move_ms = 5000;

    HAL_Delay(move_ms);
    return MB_OK;
}

