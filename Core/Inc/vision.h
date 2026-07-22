#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 *  vision.c — MaixCAM Pro 视觉数据接收 (USART2 + DMA + IDLE)
 *
 *  协议: 文本包, 形如  $TAG,x,y#  或  $HB,n#
 *    $LASER,x,y#   激光点(=吊舱)全局坐标, cm
 *    $FIRE,x,y#    火源全局坐标, cm
 *    $CIRCLE,x,y#  当前匹配橙色圆全局坐标, cm
 *    $TILT,x,y#    姿态(定性), 导航时一般不用
 *    $HB,n#        心跳, STM32 需原样回发 $HB,n#
 * ============================================================ */

/* 视觉数据集中放这里, 主循环读取 */
typedef struct {
    float   laser_x,  laser_y;    /* 激光点(吊舱)全局坐标 cm */
    uint8_t laser_fresh;          /* 1=有新数据, 主循环读后清0 */
    uint32_t laser_ms;            /* 最后一次更新的时刻 HAL_GetTick() */

    float   fire_x,   fire_y;     /* 火源全局坐标 cm */
    uint8_t fire_fresh;
    uint32_t fire_ms;

    float   circle_x, circle_y;   /* 当前匹配圆坐标 cm */
    uint8_t circle_fresh;
    uint32_t circle_ms;

    float   tilt_x,   tilt_y;     /* 姿态 */
    uint8_t tilt_fresh;
    uint32_t tilt_ms;

    int      hb_value;            /* 收到的心跳编号 */
    uint8_t  hb_pending;          /* 1=待回发, 回发后清0 */
    uint32_t last_rx_ms;          /* 任意一包数据的最后到达时刻 */

    uint32_t rx_byte_count;       /* USART2 已处理字节数, 用于判断物理链路是否有数据 */
    uint32_t frame_count;         /* 完整 $...# 帧数量 */
    uint32_t bad_frame_count;     /* 超长/缺字段等异常帧 */
    uint32_t unknown_count;       /* 未识别 TAG 数量 */
    uint32_t laser_count;
    uint32_t fire_count;
    uint32_t circle_count;
    uint32_t tilt_count;
    uint32_t hb_count;
} vision_data_t;

extern vision_data_t g_vision;

/* 初始化: 启动 USART2 的 DMA+IDLE 接收。HAL 初始化完成后调用一次 */
void vision_init(void);

/* 主循环周期调用: 处理心跳回发等。建议每个主循环都调 */
void vision_task(void);

/* 判断视觉链路是否在线 (timeout_ms 内有收到过任意数据) */
uint8_t vision_is_online(uint32_t timeout_ms);

uint8_t vision_read_filtered(float *x, float *y);

#endif /* __VISION_H */
