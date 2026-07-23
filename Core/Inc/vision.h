#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 *  vision.c — MaixCAM Pro 视觉数据接收 (USART2 + DMA + IDLE)
 *
 *  协议: 文本包, 形如  $TAG,x,y#  或  $HB,n#
 *    $LASER,x,y#   激光点(=吊舱)全局坐标, cm
 *    $HOME,dx,dy#  激光点相对中心圆的偏差, cm, +x右 +y上; CENTER/ORIGIN/OFFSET 同义
 *    $FIRE,x,y#    火源全局坐标, cm
 *    $CIRCLE,x,y#  当前匹配橙色圆全局坐标, cm
 *    $TILT,x,y#    姿态(定性), 导航时一般不用
 *    $HB,n#        心跳, STM32 需原样回发 $HB,n#
 * ============================================================ */

#define VISION_FRAME_TEXT_MAX  48U

typedef enum {
    VISION_PARSE_ERROR_NONE = 0,
    VISION_PARSE_ERROR_EMPTY,
    VISION_PARSE_ERROR_NO_START,
    VISION_PARSE_ERROR_RESTARTED,
    VISION_PARSE_ERROR_TOO_LONG,
    VISION_PARSE_ERROR_MISSING_FIELD,
    VISION_PARSE_ERROR_INVALID_NUMBER
} vision_parse_error_t;

/* 视觉数据集中放这里, 主循环读取 */
typedef struct {
    float   laser_x,  laser_y;    /* 激光点(吊舱)全局坐标 cm */
    uint8_t laser_fresh;          /* 1=有新数据, 主循环读后清0 */
    uint32_t laser_ms;            /* 最后一次更新的时刻 HAL_GetTick() */

    float   home_x, home_y;       /* 激光点相对中心圆的偏差 cm, 用于视觉回零 */
    uint8_t home_fresh;
    uint32_t home_ms;

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
    uint8_t  dma_started;         /* 1=USART2 DMA 接收已成功启动 */
    uint8_t  dma_start_status;    /* HAL_UARTEx_ReceiveToIdle_DMA 返回值 */
    uint32_t last_byte_ms;        /* 最后一个 USART2 字节的到达时刻 */
    uint32_t last_rx_ms;          /* 最后一个完整 $...# 帧的到达时刻 */
    uint32_t last_valid_ms;       /* 最后一个合法已识别帧的到达时刻 */

    uint32_t rx_byte_count;       /* USART2 已处理字节数, 用于判断物理链路是否有数据 */
    uint32_t frame_count;         /* 完整 $...# 帧数量 */
    uint32_t valid_frame_count;   /* 格式正确且 TAG 已识别的帧数量 */
    uint32_t bad_frame_count;     /* 超长/缺字段等异常帧 */
    uint32_t unknown_count;       /* 未识别 TAG 数量 */
    uint32_t uart_error_count;    /* USART2 HAL 错误回调次数 */
    uint32_t last_uart_error;     /* 最近一次 HAL UART 错误码 */
    vision_parse_error_t last_bad_reason;
    char last_bad_frame[VISION_FRAME_TEXT_MAX];
    char last_unknown_frame[VISION_FRAME_TEXT_MAX];
    uint32_t laser_count;
    uint32_t home_count;
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

/* 下位机 -> MaixCAM 模式切换命令: initial / total / fire */
uint8_t vision_send_mode(const char *mode);

/* 判断视觉协议是否在线 (timeout_ms 内收到过合法已识别帧) */
uint8_t vision_is_online(uint32_t timeout_ms);

uint8_t vision_read_filtered(float *x, float *y);
uint8_t vision_read_home_filtered(float *x, float *y);
uint8_t vision_read_circle_error_filtered(float *x, float *y);
uint8_t vision_read_circle_error_with_ref_filtered(float *err_x, float *err_y,
                                                   float *circle_x, float *circle_y);

#endif /* __VISION_H */
