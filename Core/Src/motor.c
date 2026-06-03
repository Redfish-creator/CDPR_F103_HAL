/* ============================================================
 *  motor.c — ZDT X42S 闭环步进电机 Modbus-RTU 驱动
 *  总线: USART3 + RS485(自动方向)
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

/* ---------- 底层: 发送一帧 (自动补 CRC) ----------
 * frame[0..len-1] 为 地址+功能码+数据, 本函数追加 2 字节 CRC 后发出 */
static mb_result_t mb_send(uint8_t *frame, uint16_t len)
{
    uint16_t crc = modbus_crc16(frame, len);
    frame[len]   = (uint8_t)(crc & 0xFF);   /* CRC 低字节 */
    frame[len+1] = (uint8_t)(crc >> 8);     /* CRC 高字节 */

    /* RS485 为自动方向, 直接发即可。发送前清掉接收端可能残留的回显/旧数据 */
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)huart3.Instance->DR;

    if (HAL_UART_Transmit(&huart3, frame, len + 2, MB_TX_TIMEOUT) != HAL_OK)
        return MB_ERR_TX;
    return MB_OK;
}

/* ---------- 底层: 收一帧并校验 ----------
 * 期望应答 expect_len 字节(含 CRC)。返回 MB_OK 时 rx[] 内是有效帧。
 * 兼容 RS485 自收回显: 若开头出现刚发出去的帧, 自动跳过。 */
static mb_result_t mb_recv(uint8_t *rx, uint16_t expect_len,
                           const uint8_t *sent, uint16_t sent_len)
{
    uint8_t  tmp[MB_BUF_SIZE];
    uint16_t got = 0;
    uint32_t t0  = HAL_GetTick();

    /* 持续收, 直到凑够 期望长度 或 超时 */
    while (got < expect_len + sent_len) {
        uint8_t  ch;
        uint32_t left = MB_RX_TIMEOUT - (HAL_GetTick() - t0);
        if ((HAL_GetTick() - t0) >= MB_RX_TIMEOUT) break;
        if (HAL_UART_Receive(&huart3, &ch, 1, left) != HAL_OK) break;
        if (got < MB_BUF_SIZE) tmp[got] = ch;
        got++;
        /* 已收到至少 期望长度, 且不含回显 -> 可提前结束 */
        if (got >= expect_len &&
            !(sent_len && got < sent_len)) {
            /* 简单判断: 若开头不是回显, expect_len 字节就够了 */
            if (!(sent && sent_len && got >= sent_len &&
                  memcmp(tmp, sent, sent_len) == 0)) {
                if (got >= expect_len) break;
            }
        }
    }
    if (got < expect_len) return MB_ERR_TIMEOUT;

    /* 处理 RS485 回显: 若帧头与刚发的内容一致, 偏移过去 */
    uint8_t *frame = tmp;
    if (sent && sent_len && got >= sent_len + expect_len &&
        memcmp(tmp, sent, sent_len) == 0) {
        frame = tmp + sent_len;
    }

    /* 电机返回错误码: 功能码 | 0x80 */
    if (frame[1] & 0x80) return MB_ERR_REJECT;

    /* 校验 CRC (帧尾 2 字节) */
    uint16_t crc_calc = modbus_crc16(frame, expect_len - 2);
    uint16_t crc_recv = (uint16_t)frame[expect_len-2] |
                        ((uint16_t)frame[expect_len-1] << 8);
    if (crc_calc != crc_recv) return MB_ERR_CRC;

    memcpy(rx, frame, expect_len);
    return MB_OK;
}

/* ---------- 一次完整事务: 发请求 + 收应答 ---------- */
static mb_result_t mb_transaction(uint8_t *req, uint16_t req_payload_len,
                                  uint8_t *rsp, uint16_t rsp_len)
{
    uint8_t tx[MB_BUF_SIZE];
    memcpy(tx, req, req_payload_len);
    uint16_t crc = modbus_crc16(tx, req_payload_len);
    tx[req_payload_len]   = (uint8_t)(crc & 0xFF);
    tx[req_payload_len+1] = (uint8_t)(crc >> 8);
    uint16_t tx_total = req_payload_len + 2;

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)huart3.Instance->DR;
    if (HAL_UART_Transmit(&huart3, tx, tx_total, MB_TX_TIMEOUT) != HAL_OK)
        return MB_ERR_TX;

    return mb_recv(rsp, rsp_len, tx, tx_total);
}

/* ============================================================
 *  运动控制命令
 * ============================================================ */

/* 电机使能/松轴。手册3.2.1: 多寄存器写, 寄存器1=AB|使能态, 寄存器2=同步|00 */
mb_result_t motor_enable(uint8_t addr, uint8_t on, uint8_t sync)
{
    uint8_t f[13];
    f[0] = addr;
    f[1] = FUNC_ENABLE;                 /* 0x10 */
    f[2] = (uint8_t)(REG_ENABLE >> 8);
    f[3] = (uint8_t)(REG_ENABLE & 0xFF);
    f[4] = 0x00; f[5] = 0x02;           /* 寄存器数量 = 2 */
    f[6] = 0x04;                        /* 字节数 = 4 */
    f[7] = 0xAB;                        /* 固定校验字节 */
    f[8] = on ? 0x01 : 0x00;            /* 使能状态 */
    f[9] = sync ? 0x01 : 0x00;          /* 同步标志 */
    f[10]= 0x00;
    uint8_t rsp[8];
    return mb_transaction(f, 11, rsp, 8);   /* 应答: addr+10+00E0+0002+CRC = 8B */
}

/* 梯形曲线位置模式 (X固件, 手册3.2.9)。功能码0x10, 寄存器F6, 7个寄存器 */
mb_result_t motor_move_trapezoid(uint8_t addr, uint8_t dir,
                                 uint16_t acc_rpms, uint16_t dec_rpms,
                                 uint16_t vmax_0p1rpm, uint32_t pos_0p1deg,
                                 uint8_t mode, uint8_t sync)
{
    uint8_t f[24];
    uint8_t n = 0;
    f[n++] = addr;
    f[n++] = MB_FUNC_WRITEN;            /* 0x10 */
    f[n++] = (uint8_t)(REG_TRAP_POS >> 8);
    f[n++] = (uint8_t)(REG_TRAP_POS & 0xFF);
    f[n++] = 0x00; f[n++] = 0x07;       /* 寄存器数量 = 7 */
    f[n++] = 0x0E;                      /* 字节数 = 14 */
    /* 寄存器1: 方向(占低字节, 高字节补0) */
    f[n++] = 0x00; f[n++] = dir;
    /* 寄存器2: 加速加速度 */
    f[n++] = (uint8_t)(acc_rpms >> 8); f[n++] = (uint8_t)(acc_rpms & 0xFF);
    /* 寄存器3: 减速加速度 */
    f[n++] = (uint8_t)(dec_rpms >> 8); f[n++] = (uint8_t)(dec_rpms & 0xFF);
    /* 寄存器4: 最大速度(0.1RPM) */
    f[n++] = (uint8_t)(vmax_0p1rpm >> 8); f[n++] = (uint8_t)(vmax_0p1rpm & 0xFF);
    /* 寄存器5+6: 位置角度 32bit —— X固件纯小端 (最低字节先发) */
    f[n++] = (uint8_t)(pos_0p1deg & 0xFF);
    f[n++] = (uint8_t)(pos_0p1deg >> 8);
    f[n++] = (uint8_t)(pos_0p1deg >> 16);
    f[n++] = (uint8_t)(pos_0p1deg >> 24);
    /* 寄存器7: 运动模式(高字节) + 同步标志(低字节) */
    f[n++] = mode;
    f[n++] = sync;
    uint8_t rsp[8];
    return mb_transaction(f, n, rsp, 8);    /* 应答: addr+10+00F6+0007+CRC = 8B */
}

/* 广播多机同步触发 (手册3.2.13): 地址0, 06 00FF 6600 */
mb_result_t motor_sync_trigger(void)
{
    uint8_t f[8];
    f[0] = 0x00;                        /* 广播地址 */
    f[1] = MB_FUNC_WRITE1;
    f[2] = (uint8_t)(REG_SYNC >> 8);
    f[3] = (uint8_t)(REG_SYNC & 0xFF);
    f[4] = 0x66; f[5] = 0x00;
    uint16_t crc = modbus_crc16(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)(crc >> 8);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)huart3.Instance->DR;
    /* 广播命令电机不应答, 只发不收 */
    if (HAL_UART_Transmit(&huart3, f, 8, MB_TX_TIMEOUT) != HAL_OK)
        return MB_ERR_TX;
    return MB_OK;
}

/* 立即停止 (手册3.2.12): 06 00FE 98|同步 */
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

/* 当前位置角度清零 (手册3.1.3): 06 000A 0001 */
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

/* 触发编码器校准 (手册3.1.1): 06 0006 0001。电机会缓慢正反转一圈 */
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

mb_result_t motor_read_position(uint8_t addr, int32_t *pos_0p1deg)
{
    uint8_t f[8];
    f[0] = addr;
    f[1] = MB_FUNC_READ;
    f[2] = (uint8_t)(REG_READ_RTPOS >> 8);
    f[3] = (uint8_t)(REG_READ_RTPOS & 0xFF);
    f[4] = 0x00; f[5] = 0x03;
    uint8_t rsp[11];
    mb_result_t r = mb_transaction(f, 6, rsp, 11);
    if (r != MB_OK) return r;

    /* 帧: [0]addr [1]func [2]bc=06
           [3][4] 寄1=符号(小端,看[4])
           [5][6][7][8] 寄2+寄3=位置32bit(纯小端)
           [9][10] CRC */
    uint8_t  sign = rsp[4];
    uint32_t mag  = ((uint32_t)rsp[8] << 24) | ((uint32_t)rsp[7] << 16) |
                    ((uint32_t)rsp[6] << 8)  |  (uint32_t)rsp[5];
    *pos_0p1deg = sign ? -(int32_t)mag : (int32_t)mag;
    return MB_OK;
}

/* 读电机状态标志 (手册3.4.14)。
 * 请求: addr 03 REG 0001   应答: addr 03 02 00 [状态字节] CRC = 7B */
mb_result_t motor_read_status(uint8_t addr, uint8_t *status)
{
    uint8_t f[8];
    f[0] = addr;
    f[1] = MB_FUNC_READ;
    f[2] = (uint8_t)(REG_READ_STATUS >> 8);
    f[3] = (uint8_t)(REG_READ_STATUS & 0xFF);
    f[4] = 0x00; f[5] = 0x01;
    uint8_t rsp[7];
    mb_result_t r = mb_transaction(f, 6, rsp, 7);
    if (r != MB_OK) return r;
    *status = rsp[4];                   /* [addr][func][bc=02][00][status][crcL][crcH] */
    return MB_OK;
}
