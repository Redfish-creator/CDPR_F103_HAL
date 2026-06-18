#ifndef __MOTION_H
#define __MOTION_H

#include "motor.h"
#include <stdint.h>

/* ============================================================
 *  motion — 协调运动控制
 *  把"吊舱移到 (x,y)" 翻译成 4 电机的同步运动:
 *    同步启动(广播触发) + 同步到达(速度按 |ΔL| 比例缩放)
 * ============================================================ */

/* 主导电机(ΔL 最大者)的基准运动参数 */
#define MOTION_BASE_ACC          500     /* 加速加速度 RPM/s */
#define MOTION_BASE_DEC          500     /* 减速加速度 RPM/s */
#define MOTION_BASE_VMAX         1000    /* 最大速度 0.1RPM (=100.0RPM) */
#define MOTION_MIN_SCALE         0.05f   /* 速度缩放下限, 防止过慢/为0 */
#define MOTION_REACH_TIMEOUT_MS  10000   /* 等到位超时 */

void        motion_init(void);                 /* 上电后调一次 */
mb_result_t motion_enable_all(uint8_t on);     /* 使能(1)/松开(0)全部 4 电机 */
mb_result_t motion_set_home(void);             /* 吊舱此刻在中心: 清零编码器+设基准 */
mb_result_t motion_move_to(float x, float y);  /* 协调移动吊舱中心到(x,y), 阻塞到到位 */
void        motion_get_pos(float *x, float *y);/* 取当前(软件跟踪)吊舱位置 */
uint8_t fine_tune_to(float xt, float yt);


mb_result_t motion_move_between_nohome(float x_now, float y_now, float x_next, float y_next);
#endif /* __MOTION_H */
