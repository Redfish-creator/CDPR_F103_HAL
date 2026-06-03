#ifndef __MOTOR_H
#define __MOTOR_H
 
#include "main.h"
#include <stdint.h>
#include <stdio.h> 
/* ============================================================
 *  ZDT X42S 闭环步进电机 — Modbus-RTU 通信驱动
 *  总线: USART3 + RS485(自动方向)，4 个电机地址 1~4
 *
 *  电机编号 <-> 角位:
 *    1 = 左下(BL)   2 = 右上(TR)   3 = 右下(BR)   4 = 左上(TL)
 *    Modbus 地址 = 电机编号
 * ============================================================ */
 
/* ====== Modbus 功能码 ====== */
#define MB_FUNC_READ      0x03   /* 读寄存器 (X42S 支持 03/04) */
#define MB_FUNC_WRITE1    0x06   /* 写单个寄存器 */
#define MB_FUNC_WRITEN    0x10   /* 写多个寄存器 */
 
/* ====== !!! 测试前对照 PDF 蓝色(X固件)数字核对这 4 项 !!! ====== */
#define FUNC_ENABLE       0x10   /* 手册3.2.1: 表里印 06，但带寄存器数量+字节数，
                                    实为 0x10 多寄存器写。若使能被拒绝改成 0x06 */
#define REG_ENABLE        0x00E0 /* 手册3.2.1 "F3(E0)" 取蓝色(X)值 */
#define REG_READ_RTPOS    0x0046 /* 手册3.4.12 "36(46)" 取蓝色(X)值 */
#define REG_READ_STATUS   0x0050 /* 手册3.4.14 "3A(50)" 取蓝色(X)值 */
 
/* ====== 以下寄存器已确认(X固件专用 或 双固件通用) ====== */
#define REG_TRAP_POS      0x00F6 /* 3.2.9  梯形曲线位置(X) */
#define REG_STOP          0x00FE /* 3.2.12 立即停止 (通用) */
#define REG_SYNC          0x00FF /* 3.2.13 多机同步 (通用) */
#define REG_CAL_ENCODER   0x0006 /* 3.1.1  编码器校准 (通用) */
#define REG_ZERO_POS      0x000A /* 3.1.3  位置清零 (通用) */
 
/* ====== 命令参数取值 ====== */
#define MB_DIR_CW         0x00
#define MB_DIR_CCW        0x01
#define MB_MODE_REL_LAST  0x00   /* 相对"上次目标位置" */
#define MB_MODE_ABS       0x01   /* 相对"坐标零点"(绝对定位) */
#define MB_MODE_REL_CUR   0x02   /* 相对"当前实时位置" */
#define MB_SYNC_NOW       0x00   /* 立即执行 */
#define MB_SYNC_BUFFER    0x01   /* 先缓存, 等同步触发一起执行 */
 
/* ====== 电机状态标志位 (手册3.4.14) ====== */
#define ST_ENABLED        0x01   /* Ens_TF  已使能 */
#define ST_REACHED        0x02   /* Prf_TF  位置已到达 */
#define ST_STALL          0x08   /* Cgp_TF  堵转保护已触发 */
 
/* ====== 返回码 ====== */
typedef enum {
    MB_OK = 0,
    MB_ERR_TX,        /* 串口发送失败 */
    MB_ERR_TIMEOUT,   /* 等应答超时 */
    MB_ERR_CRC,       /* 应答 CRC 校验错 */
    MB_ERR_REJECT     /* 电机返回错误码(功能码|0x80)，多半是寄存器/参数填错 */
} mb_result_t;
 
/* ---- 工具 ---- */
uint16_t    modbus_crc16(const uint8_t *data, uint16_t len);
const char *motor_result_str(mb_result_t r);
 
/* ---- 运动控制 ----
 * addr        : 电机地址 1~4
 * on          : 1=使能 0=松轴
 * dir         : MB_DIR_CW / MB_DIR_CCW
 * acc/dec     : 加速/减速加速度，单位 RPM/s
 * vmax_0p1rpm : 最大速度，单位 0.1RPM (例: 600 = 60.0RPM)
 * pos_0p1deg  : 位置角度，单位 0.1° (例: 3600 = 360.0°)
 * mode        : MB_MODE_REL_LAST / MB_MODE_ABS / MB_MODE_REL_CUR
 * sync        : MB_SYNC_NOW / MB_SYNC_BUFFER
 */
mb_result_t motor_enable(uint8_t addr, uint8_t on, uint8_t sync);
mb_result_t motor_move_trapezoid(uint8_t addr, uint8_t dir,
                                 uint16_t acc_rpms, uint16_t dec_rpms,
                                 uint16_t vmax_0p1rpm, uint32_t pos_0p1deg,
                                 uint8_t mode, uint8_t sync);
mb_result_t motor_sync_trigger(void);              /* 广播触发所有缓存命令 */
mb_result_t motor_stop(uint8_t addr, uint8_t sync); /* 立即停止(刹车) */
mb_result_t motor_zero_position(uint8_t addr);      /* 当前位置角度清零 */
mb_result_t motor_calibrate_encoder(uint8_t addr);  /* 触发编码器校准 */
 
/* ---- 读取 ---- */
mb_result_t motor_read_position(uint8_t addr, int32_t *pos_0p1deg); /* 实时位置, 单位0.1° */
mb_result_t motor_read_status(uint8_t addr, uint8_t *status);       /* 状态标志字节 */
 
#endif /* __MOTOR_H */
