/* ============================================================
 *  motor.c — ZDT X42S 闭环步进电机 Modbus-RTU 驱动
 *  总线: USART3 + RS485(自动方向)
 *  收发: HAL_UART_AbortReceive 复位接收 + 按字节收帧
 * ============================================================ */
#include "motor.h"
#include <string.h>

extern UART_HandleTypeDef huart3;          /* CubeMX 生成的 USART3 句柄 */

#define MB_TX_TIMEOUT   50                 /* 发送超时 ms */
#define MB_RX_TIMEOUT   80                 /* 等应答超时 ms */
#define MB_BUF_SIZE     64

/* ---------- Modbus CRC16 (低字节在前) ---------- */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;   /* 发送时: 先发低字节, 再发高字节 */
}

const char *motor_result_str(mb_result_t r)
{
    switch (r) {
        case MB_OK:          return "OK";
        case MB_ERR_TX:      return "TX_FAIL";
        case MB_ERR_TIMEOUT: return "TIMEOUT";
        case MB_ERR_CRC:     return "CRC_BAD";
        case MB_ERR_REJECT:  return "REJECTED";
        default:             return "?";
    }
}

/* ---------- 一次完整事务: 发请求 + 收应答并校验 ----------
 * 注意: 本函数只发一次, 不做任何重发(避免相对命令被重复累加) */
static mb_result_t mb_transaction(uint8_t *req, uint16_t req_payload_len,
                                  uint8_t *rsp, uint16_t rsp_len)
{
    uint8_t tx[MB_BUF_SIZE];
    memcpy(tx, req, req_payload_len);
    uint16_t crc = modbus_crc16(tx, req_payload_len);
    tx[req_payload_len]   = (uint8_t)(crc & 0xFF);
    tx[req_payload_len+1] = (uint8_t)(crc >> 8);
    uint16_t tx_total = req_payload_len + 2;

    HAL_UART_AbortReceive(&huart3);        /* 关键: 复位接收, 清残留/错误标志 */

    if (HAL_UART_Transmit(&huart3, tx, tx_total, MB_TX_TIMEOUT) != HAL_OK)
        return MB_ERR_TX;

    /* 按字节收满 rsp_len, 或超时 */
    uint16_t cnt = 0;
    uint32_t t0  = HAL_GetTick();
    while ((HAL_GetTick() - t0) < MB_RX_TIMEOUT && cnt < rsp_len) {
        uint8_t b;
        if (HAL_UART_Receive(&huart3, &b, 1, 5) == HAL_OK) {
            rsp[cnt++] = b;
            if (cnt >= 2 && (rsp[1] & 0x80) && cnt >= 5) break;   /* 异常帧 5 字节 */
        }
    }
    if (cnt < rsp_len) {
        if (cnt >= 2 && (rsp[1] & 0x80)) return MB_ERR_REJECT;
        return MB_ERR_TIMEOUT;
    }
    if (rsp[1] & 0x80) return MB_ERR_REJECT;
    uint16_t cc = modbus_crc16(rsp, rsp_len - 2);
    uint16_t cr = (uint16_t)rsp[rsp_len-2] | ((uint16_t)rsp[rsp_len-1] << 8);
    if (cc != cr) return MB_ERR_CRC;
    return MB_OK;
}

/* ============================================================
 *  运动控制命令
 * ============================================================ */

/* 使能/松轴。多寄存器写: 寄存器1=AB|使能态, 寄存器2=同步|00 */
mb_result_t motor_enable(uint8_t addr, uint8_t on, uint8_t sync)
{
    uint8_t f[13];
    f[0] = addr;
    f[1] = FUNC_ENABLE;
    f[2] = (uint8_t)(REG_ENABLE >> 8);
    f[3] = (uint8_t)(REG_ENABLE & 0xFF);
    f[4] = 0x00; f[5] = 0x02;
    f[6] = 0x04;
    f[7] = 0xAB;
    f[8] = on ? 0x01 : 0x00;
    f[9] = sync ? 0x01 : 0x00;
    f[10]= 0x00;
    uint8_t rsp[8];
    return mb_transaction(f, 11, rsp, 8);
}

/* 0x00F0 直通限速位置模式 (方向/速度/位置/模式/同步), 大端字, 低字先发。
 * 该命令没有独立加减速; 要真正梯形软启停得用 0x00F6 (见文末)。 */
mb_result_t motor_move_pos(uint8_t addr, uint8_t dir,
                           uint16_t vmax_0p1rpm, uint32_t pos_0p1deg,
                           uint8_t mode, uint8_t sync)
{
    uint16_t pos_low  = (uint16_t)( pos_0p1deg        & 0xFFFF);
    uint16_t pos_high = (uint16_t)((pos_0p1deg >> 16) & 0xFFFF);

    uint8_t f[24]; uint8_t n = 0;
    f[n++] = addr;
    f[n++] = MB_FUNC_WRITEN;                 /* 0x10 */
    f[n++] = 0x00; f[n++] = 0xF0;            /* 寄存器 0x00F0 */
    f[n++] = 0x00; f[n++] = 0x05;            /* 5 个寄存器 */
    f[n++] = 0x0A;                           /* 10 字节 */
    f[n++] = 0x00; f[n++] = dir;             /* reg1: 方向 00=CW(收线) 01=CCW */
    f[n++] = (uint8_t)(vmax_0p1rpm >> 8);    /* reg2: 速度 0.1RPM, 大端 */
    f[n++] = (uint8_t)(vmax_0p1rpm & 0xFF);
    f[n++] = (uint8_t)(pos_low  >> 8);       /* reg3: 位置低字(0.1°) */
    f[n++] = (uint8_t)(pos_low  & 0xFF);
    f[n++] = (uint8_t)(pos_high >> 8);       /* reg4: 位置高字 */
    f[n++] = (uint8_t)(pos_high & 0xFF);
    f[n++] = mode;                           /* reg5 高字节: 模式 */
    f[n++] = sync;                           /* reg5 低字节: 同步标志 */

    uint8_t rsp[8];
    return mb_transaction(f, n, rsp, 8);
}
/* 广播多机同步触发: 地址0, 06 00FF 6600 (电机不应答, 只发不收) */
mb_result_t motor_sync_trigger(void)
{
    uint8_t f[8];
    f[0] = 0x00;
    f[1] = MB_FUNC_WRITE1;
    f[2] = (uint8_t)(REG_SYNC >> 8);
    f[3] = (uint8_t)(REG_SYNC & 0xFF);
    f[4] = 0x66; f[5] = 0x00;
    uint16_t crc = modbus_crc16(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)(crc >> 8);
    HAL_UART_AbortReceive(&huart3);
    if (HAL_UART_Transmit(&huart3, f, 8, MB_TX_TIMEOUT) != HAL_OK)
        return MB_ERR_TX;
    return MB_OK;
}

/* 立即停止: 06 00FE 98|同步 */
mb_result_t motor_stop(uint8_t addr, uint8_t sync)
{
    uint8_t f[8];
    f[0] = addr;
    f[1] = MB_FUNC_WRITE1;
    f[2] = (uint8_t)(REG_STOP >> 8);
    f[3] = (uint8_t)(REG_STOP & 0xFF);
    f[4] = 0x98;
    f[5] = sync ? 0x01 : 0x00;
    uint8_t rsp[8];
    return mb_transaction(f, 6, rsp, 8);
}

/* 当前位置角度清零: 06 000A 0001 */
mb_result_t motor_zero_position(uint8_t addr)
{
    uint8_t f[8];
    f[0] = addr;
    f[1] = MB_FUNC_WRITE1;
    f[2] = (uint8_t)(REG_ZERO_POS >> 8);
    f[3] = (uint8_t)(REG_ZERO_POS & 0xFF);
    f[4] = 0x00; f[5] = 0x01;
    uint8_t rsp[8];
    return mb_transaction(f, 6, rsp, 8);
}

/* 触发编码器校准: 06 0006 0001 */
mb_result_t motor_calibrate_encoder(uint8_t addr)
{
    uint8_t f[8];
    f[0] = addr;
    f[1] = MB_FUNC_WRITE1;
    f[2] = (uint8_t)(REG_CAL_ENCODER >> 8);
    f[3] = (uint8_t)(REG_CAL_ENCODER & 0xFF);
    f[4] = 0x00; f[5] = 0x01;
    uint8_t rsp[8];
    return mb_transaction(f, 6, rsp, 8);
}

/* ============================================================
 *  读取命令
 * ============================================================ */

/* 读实时位置: 大端字解码(与验证过的一致), 单位 0.1° */
mb_result_t motor_read_position(uint8_t addr, int32_t *pos_0p1deg)
{
    uint8_t f[8];
    f[0] = addr; f[1] = MB_FUNC_READ;      /* 0x04 */
    f[2] = 0x00; f[3] = 0x46;
    f[4] = 0x00; f[5] = 0x03;
    uint8_t rsp[11];
    mb_result_t r = mb_transaction(f, 6, rsp, 11);
    if (r != MB_OK) return r;

    uint16_t sign     = ((uint16_t)rsp[3] << 8) | rsp[4];
    uint16_t pos_low  = ((uint16_t)rsp[5] << 8) | rsp[6];
    uint16_t pos_high = ((uint16_t)rsp[7] << 8) | rsp[8];
    uint32_t mag = ((uint32_t)pos_high << 16) | pos_low;
    *pos_0p1deg = (sign == 1) ? -(int32_t)mag : (int32_t)mag;
    return MB_OK;
}

/* 读电机状态标志 (X42S: 寄存器0x0054, 功能码0x04)
 * 应答: addr 04 02 [回零标志] [电机标志] crcL crcH = 7字节, 到位位 Prf_TF=rsp[4]&0x02 */
mb_result_t motor_read_status(uint8_t addr, uint8_t *status)
{
    uint8_t f[8];
    f[0] = addr;
    f[1] = 0x04;
    f[2] = 0x00; f[3] = 0x54;
    f[4] = 0x00; f[5] = 0x01;
    uint8_t rsp[7];
    mb_result_t r = mb_transaction(f, 6, rsp, 7);
    if (r != MB_OK) return r;
    *status = rsp[4];
    return MB_OK;
}

/* 读位置角度误差 (寄存器0x0037), X固件单位=值/100 度 */
mb_result_t motor_read_pos_error(uint8_t addr, int32_t *err_x100)
{
    uint8_t f[8];
    f[0] = addr; f[1] = MB_FUNC_READ;
    f[2] = 0x00; f[3] = 0x37;
    f[4] = 0x00; f[5] = 0x03;
    uint8_t rsp[11];
    mb_result_t r = mb_transaction(f, 6, rsp, 11);
    if (r != MB_OK) return r;
    uint16_t sign     = ((uint16_t)rsp[3] << 8) | rsp[4];
    uint16_t pos_low  = ((uint16_t)rsp[5] << 8) | rsp[6];
    uint16_t pos_high = ((uint16_t)rsp[7] << 8) | rsp[8];
    uint32_t mag = ((uint32_t)pos_high << 16) | pos_low;
    *err_x100 = (sign == 1) ? -(int32_t)mag : (int32_t)mag;
    return MB_OK;
}

/* 读位置到达窗口 PRWindow (寄存器0x0041), 返回值 ×10 (8=0.8°) */
mb_result_t motor_read_pos_window(uint8_t addr, uint16_t *prw_x10)
{
    uint8_t f[8];
    f[0] = addr; f[1] = MB_FUNC_READ;
    f[2] = 0x00; f[3] = 0x41;
    f[4] = 0x00; f[5] = 0x01;
    uint8_t rsp[7];
    mb_result_t r = mb_transaction(f, 6, rsp, 7);
    if (r != MB_OK) return r;
    *prw_x10 = ((uint16_t)rsp[3] << 8) | rsp[4];
    return MB_OK;
}

/* 设置位置到达窗口 PRWindow (寄存器0x00D1)
 * prw_x10: 窗口角度×10 (0.8°传8) ; save: 1=保存掉电不丢 */
mb_result_t motor_set_pos_window(uint8_t addr, uint16_t prw_x10, uint8_t save)
{
    uint8_t f[16];
    uint8_t n = 0;
    f[n++] = addr;
    f[n++] = MB_FUNC_WRITEN;
    f[n++] = 0x00; f[n++] = 0xD1;
    f[n++] = 0x00; f[n++] = 0x02;
    f[n++] = 0x04;
    f[n++] = 0x07;                   /* 寄存器1高: 固定 0x07 */
    f[n++] = save ? 0x01 : 0x00;
    f[n++] = (uint8_t)(prw_x10 >> 8);
    f[n++] = (uint8_t)(prw_x10 & 0xFF);
    uint8_t rsp[8];
    return mb_transaction(f, n, rsp, 8);
}
