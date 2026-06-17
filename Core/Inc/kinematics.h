#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/* ============================================================
 *  kinematics — 4 绳悬吊吊舱 运动学 (纯数学, 不碰硬件)
 *
 *  坐标系: 原点=中心橙圆, 纸面 z=0, z 向上, x 右正 y 上正 (与视觉一致)
 *  电机下标 0..3 = 电机 1..4:  1=左下 2=右上 3=右下 4=左上
 * ============================================================ */

/* ---- 几何常量 (cm), 已锁定 ---- */
#define KIN_H_PULLEY   38.75f                       /* 滑轮出绳点高度 */
#define KIN_Z_GONDOLA  20.0f                        /* 吊舱系绳孔平面高度 */
#define KIN_DZ         (KIN_H_PULLEY - KIN_Z_GONDOLA) /* 18.75, 定高运动时恒定 */
#define KIN_POLE       30.0f                        /* 立柱在 ±30,±30 */
#define KIN_GX         4.25f                        /* 吊舱半长(沿X, 8.5/2) */
#define KIN_GY         3.25f                        /* 吊舱半宽(沿Y, 6.5/2) */

/* ---- 绕线轮 ----
 * 10mm 法兰盘 + 0.5mm 绳 -> 绳心半径 5.25mm。
 * 单层时恒定; 标定后(转10圈量绳长/(2π*10))改这一个值即可 */
#define KIN_R_EFF_MM       5.25f
#define KIN_COUNTS_PER_CM  (36000.0f / (2.0f * 3.14159265f * KIN_R_EFF_MM)) /* ~1091, 0.1°/cm */

/* ---- 可达区限制 (cm), 防止给出超范围目标 ---- */
#define KIN_XY_LIMIT   22.0f

/* (x,y) -> 4 段"滑轮到挂点"绳长 cm */
void    kin_ik(float x, float y, float L[4]);

/* 4 段绳长 -> (x,y), 最小二乘多边定位; 返回 1=成功 0=无解 */
uint8_t kin_fk(const float L[4], float *x, float *y);

/* 中心(0,0)处第 i 根(0..3)绳长, 即归零基准长度 */
float   kin_home_len(int motor_idx);

/* 目标 (x,y) 是否在可达区内 */
uint8_t kin_in_range(float x, float y);

#endif /* __KINEMATICS_H */
