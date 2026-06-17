/* ============================================================
 *  kinematics.c — 4 绳悬吊吊舱 运动学
 * ============================================================ */
#include "kinematics.h"
#include <math.h>

/* 下标 0..3 = 电机 1..4;  1=左下 2=右上 3=右下 4=左上 */
static const float Px[4] = { -30.0f, +30.0f, +30.0f, -30.0f };  /* 滑轮 X */
static const float Py[4] = { -30.0f, +30.0f, -30.0f, +30.0f };  /* 滑轮 Y */
static const float Ax[4] = { -KIN_GX, +KIN_GX, +KIN_GX, -KIN_GX }; /* 挂点偏移 X */
static const float Ay[4] = { -KIN_GY, +KIN_GY, -KIN_GY, +KIN_GY }; /* 挂点偏移 Y */

/* ---------- 逆运动学: (x,y) -> 4 段绳长 ---------- */
void kin_ik(float x, float y, float L[4])
{
    for (int i = 0; i < 4; i++) {
        float ex = Px[i] - (x + Ax[i]);
        float ey = Py[i] - (y + Ay[i]);
        L[i] = sqrtf(ex * ex + ey * ey + KIN_DZ * KIN_DZ);
    }
}

float kin_home_len(int motor_idx)
{
    float L[4];
    kin_ik(0.0f, 0.0f, L);
    return L[motor_idx];
}

uint8_t kin_in_range(float x, float y)
{
    return (fabsf(x) <= KIN_XY_LIMIT && fabsf(y) <= KIN_XY_LIMIT) ? 1 : 0;
}

/* ---------- 正运动学: 4 段绳长 -> (x,y) ----------
 * 每根绳在水平面上确定一个圆: (x-Cx)^2 + (y-Cy)^2 = L^2 - DZ^2
 * 其中有效锚点 C = (Px-Ax, Py-Ay)。以圆0为基准两两作差得线性方程,
 * 4 圆 -> 3 方程, 最小二乘解 (x,y)。 */
uint8_t kin_fk(const float L[4], float *x, float *y)
{
    float Cx[4], Cy[4], rho2[4];
    for (int i = 0; i < 4; i++) {
        Cx[i] = Px[i] - Ax[i];
        Cy[i] = Py[i] - Ay[i];
        rho2[i] = L[i] * L[i] - KIN_DZ * KIN_DZ;
        if (rho2[i] < 0.0f) return 0;            /* 绳比竖直落差还短, 无解 */
    }

    float k0 = rho2[0] - (Cx[0]*Cx[0] + Cy[0]*Cy[0]);
    /* 正规方程 (A^T A) p = A^T d */
    float a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
    for (int i = 1; i < 4; i++) {
        float ai = 2.0f * (Cx[i] - Cx[0]);
        float bi = 2.0f * (Cy[i] - Cy[0]);
        float ki = rho2[i] - (Cx[i]*Cx[i] + Cy[i]*Cy[i]);
        float di = ki - k0;
        a11 += ai * ai;  a12 += ai * bi;  a22 += bi * bi;
        b1  += ai * di;  b2  += bi * di;
    }
    float det = a11 * a22 - a12 * a12;
    if (fabsf(det) < 1e-6f) return 0;
    *x = ( a22 * b1 - a12 * b2) / det;
    *y = (-a12 * b1 + a11 * b2) / det;
    return 1;
}
