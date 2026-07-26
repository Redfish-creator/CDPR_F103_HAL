#ifndef __MOTION_H
#define __MOTION_H

#include "motor.h"
#include <stdint.h>

/* Competition default: keep only faults in the USART1 motion log. */
#ifndef MOTION_COMPACT_LOG
#define MOTION_COMPACT_LOG 1U
#endif

/* ============================================================
 *  motion — 协调运动控制
 *  把"吊舱移到 (x,y)" 翻译成 4 电机的同步运动:
 *    同步启动(广播触发) + 同步到达(速度按 |ΔL| 比例缩放)
 * ============================================================ */

/* 主导电机(ΔL 最大者)的基准运动参数 */
#define MOTION_BASE_ACC          500     /* 加速加速度 RPM/s */
#define MOTION_BASE_DEC          500     /* 减速加速度 RPM/s */
#define MOTION_BASE_VMAX         1000    /* 最大速度 0.1RPM (=100.0RPM) */
#define MOTION_PATROL_BASE_VMAX  300     /* 巡检主导电机速度 0.1RPM (=30.0RPM) */
#define MOTION_MIN_SCALE         0.05f   /* 速度缩放下限, 防止过慢/为0 */
#define MOTION_REACH_TIMEOUT_MS  10000   /* 等到位超时 */

/*
 * Stable patrol profile: historical DMA logs contain substantially more
 * no-byte 0x10 failures at M2, while M2 was always the second buffered write.
 * Queue M2 first after the long inter-segment quiet period. All four commands
 * are still buffered and are executed only by the same broadcast trigger, so
 * this changes bus transaction order, not the commanded motion.
 */
#ifndef MOTION_PATROL_M2_FIRST
#define MOTION_PATROL_M2_FIRST   1U
#endif

typedef struct {
    uint32_t sequence;
    float x_now;
    float y_now;
    float x_next;
    float y_next;
    float dL[4];
    float cmd_cm[4];
    float gain[4];
    uint8_t dir[4];
    uint32_t mag[4];
    uint16_t vmax[4];
    uint8_t bal;
    uint8_t attempted_mask;
    uint8_t trigger_attempted;
    uint8_t completed;
    uint8_t failed_motor;
    uint8_t bus_slot[4];
    mb_result_t motor_result[4];
    motor_transaction_summary_t motor_diag[4];
    mb_result_t trigger_result;
    uint32_t estimated_move_ms;
} motion_segment_report_t;

void        motion_init(void);                 /* 上电后调一次 */
mb_result_t motion_enable_all(uint8_t on);     /* 使能(1)/松开(0)全部 4 电机 */
mb_result_t motion_set_home(void);             /* 吊舱此刻在中心: 清零编码器+设基准 */
mb_result_t motion_move_to(float x, float y);  /* 协调移动吊舱中心到(x,y), 阻塞到到位 */
void        motion_get_pos(float *x, float *y);/* 取当前(软件跟踪)吊舱位置 */
void        motion_get_last_segment_report(motion_segment_report_t *report);
uint8_t fine_tune_to(float xt, float yt);


mb_result_t motion_move_between_nohome(float x_now, float y_now, float x_next, float y_next);
mb_result_t motion_move_between_nohome_relaxed(float x_now, float y_now,
                                               float x_next, float y_next,
                                               float takeup_gain,
                                               float payout_gain);
mb_result_t motion_move_between_nohome_relaxed_quick(float x_now, float y_now,
                                                     float x_next, float y_next,
                                                     float takeup_gain,
                                                     float payout_gain);
mb_result_t motion_move_between_nohome_patrol(float x_now, float y_now,
                                              float x_next, float y_next,
                                              float takeup_gain,
                                              float payout_gain);
mb_result_t motion_jog_step_start(float x_now, float y_now,
                                  float x_next, float y_next,
                                  float takeup_gain,
                                  float payout_gain,
                                  uint32_t *move_ms);
mb_result_t motion_jog_velocity_at(float x_now, float y_now,
                                   float vx_cm_s, float vy_cm_s);
mb_result_t motion_jog_stop(void);
#endif /* __MOTION_H */
