/* ============================================================
 *  motor.h — ZDT X42S 闭环步进电机 Modbus-RTU 驱动 接口
 * ============================================================ */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

/* ---- Modbus 功能码 ---- */
#define MB_FUNC_READ     0x03
#define MB_FUNC_WRITE1   0x06
#define MB_FUNC_WRITEN   0x10
#define FUNC_ENABLE      0x10

/* ---- 寄存器地址 ---- */
#define REG_ENABLE       0x00E0   /* 使能 */
#define REG_TRAP_POS     0x00F6   /* 梯形曲线位置模式(X固件), 7寄存器 */
#define REG_SYNC         0x00FF   /* 广播同步触发 */
#define REG_STOP         0x00FE   /* 立即停止 */
#define REG_ZERO_POS     0x000A   /* 当前位置清零 */
#define REG_CAL_ENCODER  0x0006   /* 触发编码器校准 */
#define REG_READ_RTPOS   0x0046   /* 读实时位置 */
#define REG_READ_STATUS  0x0054   /* X42S: 回零标志+电机标志(功能码0x04) */

/* ---- 方向 ---- */
#define MB_DIR_CW        0x00     /* 收线(绳变短), 实测使位置增大 */
#define MB_DIR_CCW       0x01     /* 放线(绳变长), 实测使位置减小 */

/* ---- 运动模式 ---- */
#define MB_MODE_REL_LAST 0x00     /* 相对上一目标位置 */
#define MB_MODE_ABS      0x01     /* 绝对(不可靠, 勿用) */
#define MB_MODE_REL_CUR  0x02     /* 相对当前实时位置(可靠, 在用) */

/* ---- 同步标志 ---- */
#define MB_SYNC_NOW      0x00     /* 立即执行 */
#define MB_SYNC_BUFFER   0x01     /* 缓存, 等广播触发 */

/* ---- 电机状态标志位(0x0054 应答 rsp[4]) ---- */
#define ST_ENABLED       0x01     /* Ens_TF 使能 */
#define ST_REACHED       0x02     /* Prf_TF 到位 */
#define ST_STALL         0x08     /* Cgp_TF 堵转保护 */

/* ---- 返回码 ---- */
typedef enum {
    MB_OK = 0,
    MB_ERR_TX,
    MB_ERR_TIMEOUT,
    MB_ERR_CRC,
    MB_ERR_REJECT
} mb_result_t;

uint16_t       modbus_crc16(const uint8_t *data, uint16_t len);
const char    *motor_result_str(mb_result_t r);

mb_result_t motor_enable(uint8_t addr, uint8_t on, uint8_t sync);
mb_result_t motor_move_pos(uint8_t addr, uint8_t dir,
                           uint16_t vmax_0p1rpm, uint32_t pos_0p1deg,
                           uint8_t mode, uint8_t sync);
mb_result_t motor_sync_trigger(void);
mb_result_t motor_stop(uint8_t addr, uint8_t sync);
mb_result_t motor_zero_position(uint8_t addr);
mb_result_t motor_calibrate_encoder(uint8_t addr);

mb_result_t motor_read_position(uint8_t addr, int32_t *pos_0p1deg);
mb_result_t motor_read_status(uint8_t addr, uint8_t *status);
mb_result_t motor_read_pos_error(uint8_t addr, int32_t *err_x100);
mb_result_t motor_read_pos_window(uint8_t addr, uint16_t *prw_x10);
mb_result_t motor_set_pos_window(uint8_t addr, uint16_t prw_x10, uint8_t save);

#endif /* __MOTOR_H */
