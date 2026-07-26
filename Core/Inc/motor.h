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

/* ---- USART3 Modbus receive implementation switch ----
 * DMA_SCAN: current DMA stream + frame scan implementation.
 * BLOCKING_BYTE: backup-current/main style HAL_UART_Receive byte polling.
 * Keep command protocol unchanged; this switch only changes how responses
 * are collected after each request.
 */
#define MOTOR_MODBUS_RX_DMA_SCAN      0U
#define MOTOR_MODBUS_RX_BLOCKING_BYTE 1U
#ifndef MOTOR_MODBUS_RX_IMPL
#define MOTOR_MODBUS_RX_IMPL MOTOR_MODBUS_RX_DMA_SCAN
#endif

/* ---- 寄存器地址 ---- */
#define REG_ENABLE       0x00E0   /* 使能 */
#define REG_TRAP_POS     0x00F6   /* 梯形曲线位置模式(X固件), 7寄存器 */
#define REG_SPEED_MODE   0x00F6   /* F6 speed mode, 3 registers */
#define REG_SYNC         0x00FF   /* 广播同步触发 */
#define REG_STOP         0x00FE   /* 立即停止 */
#define REG_ZERO_POS     0x000A   /* 当前位置清零 */
#define REG_CAL_ENCODER  0x0006   /* 触发编码器校准 */
#define REG_READ_RTPOS   0x0046   /* 读实时位置 */
#define REG_READ_STATUS  0x0054   /* X42S: 回零标志+电机标志(功能码0x04) */

/* ---- 方向 ---- */
#define MB_DIR_CW        0x00     /* 驱动器 CW, 位置增大 */
#define MB_DIR_CCW       0x01     /* 驱动器 CCW, 位置减小 */

/* ---- 绳长动作 (卷线机构镜像安装) ---- */
#define MOTOR_CABLE_PAYOUT  0U
#define MOTOR_CABLE_TAKEUP  1U

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
    MB_ERR_REJECT,
    MB_ERR_FRAME,
    MB_ERR_BUSY
} mb_result_t;

#define MOTOR_DIAG_FRAME_MAX 32U

typedef struct {
    uint32_t transaction_id;
    uint8_t addr;
    uint8_t function;
    uint8_t exception_code;
    uint8_t response_complete;
    uint8_t tx_ok;
    uint16_t request_register;
    uint16_t request_quantity;
    uint16_t tx_len;
    uint16_t rx_len;
    uint16_t raw_rx_len;
    uint16_t expected_rx_len;
    uint16_t frame_offset;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint32_t elapsed_ms;
    uint32_t tx_time_us;
    uint32_t first_rx_us;
    uint32_t last_rx_us;
    uint32_t uart_error;
    uint32_t uart_sr;
    uint32_t dma_isr;
    mb_result_t result;
    uint8_t tx[MOTOR_DIAG_FRAME_MAX];
    uint8_t rx[MOTOR_DIAG_FRAME_MAX];
    uint8_t raw_rx[MOTOR_DIAG_FRAME_MAX];
} motor_transaction_diag_t;

typedef struct {
    uint32_t transaction_id;
    uint16_t request_register;
    uint16_t request_quantity;
    uint16_t rx_len;
    uint16_t raw_rx_len;
    uint16_t expected_rx_len;
    uint16_t frame_offset;
    uint16_t tx_time_us;
    uint16_t first_rx_us;
    uint16_t last_rx_us;
    uint16_t elapsed_ms;
    uint8_t addr;
    uint8_t function;
    uint8_t exception_code;
    uint8_t response_complete;
    uint32_t uart_error;
    mb_result_t result;
} motor_transaction_summary_t;

typedef struct {
    uint16_t firmware_raw;
    uint16_t hardware_raw;
    uint8_t series_code;
    uint8_t type_code;
    uint8_t hardware_version;
} motor_identity_t;

uint16_t       modbus_crc16(const uint8_t *data, uint16_t len);
const char    *motor_result_str(mb_result_t r);
const char    *motor_rx_mode_name(void);
uint8_t        motor_cable_direction(uint8_t addr, uint8_t cable_action);
void           motor_get_last_transaction_diag(motor_transaction_diag_t *diag);
void           motor_get_last_transaction_summary(motor_transaction_summary_t *summary);
void           motor_print_last_transaction_diag(const char *prefix);

mb_result_t motor_enable(uint8_t addr, uint8_t on, uint8_t sync);
mb_result_t motor_move_pos(uint8_t addr, uint8_t dir,
                           uint16_t vmax_0p1rpm, uint32_t pos_0p1deg,
                           uint8_t mode, uint8_t sync);
mb_result_t motor_run_speed(uint8_t addr, uint8_t dir,
                            uint16_t speed_0p1rpm,
                            uint8_t acc, uint8_t sync);
mb_result_t motor_sync_trigger(void);
mb_result_t motor_stop(uint8_t addr, uint8_t sync);
mb_result_t motor_zero_position(uint8_t addr);
mb_result_t motor_calibrate_encoder(uint8_t addr);

mb_result_t motor_read_position(uint8_t addr, int32_t *pos_0p1deg);
mb_result_t motor_read_status(uint8_t addr, uint8_t *status);
mb_result_t motor_read_pos_error(uint8_t addr, int32_t *err_x100);
mb_result_t motor_read_pos_window(uint8_t addr, uint16_t *prw_x10);
mb_result_t motor_set_pos_window(uint8_t addr, uint16_t prw_x10, uint8_t save);
mb_result_t motor_read_identity_x(uint8_t addr, motor_identity_t *identity);
mb_result_t motor_read_bus_voltage(uint8_t addr, uint16_t *millivolts);
mb_result_t motor_read_state_x(uint8_t addr, uint8_t *state);
mb_result_t motor_read_heartbeat_time_x(uint8_t addr, uint32_t *milliseconds);

#endif /* __MOTOR_H */
