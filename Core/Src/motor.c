/* ============================================================
 *  motor.c — ZDT X42S 闭环步进电机 Modbus-RTU 驱动
 *  总线: USART3 + RS485(自动方向)
 *  收发: RX 可切换 DMA 扫帧/旧阻塞接收；TX 默认 DMA 连续帧发送
 * ============================================================ */
#include "motor.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;          /* CubeMX 生成的 USART3 句柄 */

#define MB_TX_TIMEOUT             50U      /* 发送超时 ms */
#define MB_RX_TIMEOUT             400U     /* 完整应答窗口 ms */
#define MB_BUF_SIZE                64U
#define MB_DMA_RX_CAPACITY        64U      /* DMA1_Channel3, USART3_RX */

#if (MOTOR_MODBUS_RX_IMPL != MOTOR_MODBUS_RX_DMA_SCAN) && \
    (MOTOR_MODBUS_RX_IMPL != MOTOR_MODBUS_RX_BLOCKING_BYTE)
#error "Unsupported MOTOR_MODBUS_RX_IMPL"
#endif

#if (MOTOR_MODBUS_TX_IMPL != MOTOR_MODBUS_TX_DMA_CONTIG) && \
    (MOTOR_MODBUS_TX_IMPL != MOTOR_MODBUS_TX_HAL_BLOCKING)
#error "Unsupported MOTOR_MODBUS_TX_IMPL"
#endif

static motor_transaction_diag_t s_last_diag;
static uint8_t s_dma_rx_buf[MB_DMA_RX_CAPACITY];
static uint8_t s_dma_rx_active;
static uint32_t s_transaction_sequence;
static uint32_t s_transaction_start_cycles;
static uint8_t s_cycle_counter_ready;

static uint32_t motor_cycles_now(void)
{
    if (s_cycle_counter_ready == 0U) {
        SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
        DWT->CYCCNT = 0U;
        SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
        s_cycle_counter_ready = 1U;
    }
    return DWT->CYCCNT;
}

static uint32_t motor_cycles_to_us(uint32_t cycles)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        cycles_per_us = 1U;
    }
    return cycles / cycles_per_us;
}

static uint32_t motor_transaction_us_now(void)
{
    return motor_cycles_to_us(motor_cycles_now() -
                              s_transaction_start_cycles);
}

static uint16_t motor_u32_to_u16_sat(uint32_t value)
{
    return (value > 65535U) ? 65535U : (uint16_t)value;
}

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
        case MB_ERR_FRAME:   return "FRAME_BAD";
        case MB_ERR_BUSY:    return "UART_BUSY";
        default:             return "?";
    }
}

const char *motor_rx_mode_name(void)
{
#if MOTOR_MODBUS_RX_IMPL == MOTOR_MODBUS_RX_BLOCKING_BYTE
    return "BLOCKING_BYTE";
#else
    return "DMA_SCAN";
#endif
}

const char *motor_tx_mode_name(void)
{
#if MOTOR_MODBUS_TX_IMPL == MOTOR_MODBUS_TX_HAL_BLOCKING
    return "HAL_BLOCKING";
#else
    return "DMA_CONTIG";
#endif
}

void motor_get_last_transaction_diag(motor_transaction_diag_t *diag)
{
    if (diag != NULL) {
        *diag = s_last_diag;
    }
}

void motor_get_last_transaction_summary(motor_transaction_summary_t *summary)
{
    if (summary == NULL) {
        return;
    }

    memset(summary, 0, sizeof(*summary));
    summary->transaction_id = s_last_diag.transaction_id;
    summary->request_register = s_last_diag.request_register;
    summary->request_quantity = s_last_diag.request_quantity;
    summary->rx_len = s_last_diag.rx_len;
    summary->raw_rx_len = s_last_diag.raw_rx_len;
    summary->expected_rx_len = s_last_diag.expected_rx_len;
    summary->frame_offset = s_last_diag.frame_offset;
    summary->tx_time_us = motor_u32_to_u16_sat(s_last_diag.tx_time_us);
    summary->first_rx_us = motor_u32_to_u16_sat(s_last_diag.first_rx_us);
    summary->last_rx_us = motor_u32_to_u16_sat(s_last_diag.last_rx_us);
    summary->elapsed_ms = motor_u32_to_u16_sat(s_last_diag.elapsed_ms);
    summary->addr = s_last_diag.addr;
    summary->function = s_last_diag.function;
    summary->exception_code = s_last_diag.exception_code;
    summary->response_complete = s_last_diag.response_complete;
    summary->uart_error = s_last_diag.uart_error;
    summary->result = s_last_diag.result;
}

static mb_result_t motor_diag_finish(mb_result_t result, uint32_t start_ms)
{
    s_last_diag.result = result;
    s_last_diag.elapsed_ms = HAL_GetTick() - start_ms;
    return result;
}

static void motor_diag_reset(const uint8_t *tx,
                             uint16_t tx_len,
                             uint16_t expected_rx_len)
{
    memset(&s_last_diag, 0, sizeof(s_last_diag));
    s_last_diag.transaction_id = ++s_transaction_sequence;
    if (tx != NULL && tx_len != 0U) {
        s_last_diag.addr = tx[0];
        s_last_diag.function = (tx_len > 1U) ? tx[1] : 0U;
        if (tx_len >= 6U) {
            s_last_diag.request_register =
                ((uint16_t)tx[2] << 8) | tx[3];
            s_last_diag.request_quantity =
                ((uint16_t)tx[4] << 8) | tx[5];
        }
        s_last_diag.tx_len = (tx_len > MOTOR_DIAG_FRAME_MAX) ?
                             MOTOR_DIAG_FRAME_MAX : tx_len;
        memcpy(s_last_diag.tx, tx, s_last_diag.tx_len);
    }
    s_last_diag.expected_rx_len = expected_rx_len;
    s_last_diag.tx_ok = 0U;
}

/*
 * USART3 is used as a foreground-controlled, half-duplex Modbus port in this
 * application; response bytes are collected by a short DMA window.
 * CubeMX still enables the NVIC line, but no USART3 interrupt reception is
 * started anywhere.  Leaving a stale/pending IRQ enabled lets the HAL handler
 * consume a byte (or terminate an old receive) while the foreground code is
 * polling the same DR register.  Disable that unused path once, at the same
 * time as the receive cleanup.
 */
static void motor_uart_disable_unused_irq(void)
{
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
}

static void motor_uart_clear_status(void)
{
    /*
     * On STM32F1 PE/FE/NE/ORE/IDLE are cleared by the SR-then-DR sequence.
     * Do this once rather than calling the HAL macros repeatedly: each macro
     * reads DR and could eat a newly arrived byte.
     */
    volatile uint32_t sr = READ_REG(huart3.Instance->SR);
    volatile uint32_t dr = READ_REG(huart3.Instance->DR);
    (void)sr;
    (void)dr;
}

static uint32_t motor_uart_error_from_sr(uint32_t sr)
{
    uint32_t error = HAL_UART_ERROR_NONE;

    /*
     * USART_SR_FE and USART_SR_NE do not use the same bit values as the HAL
     * error enum, so translate each flag explicitly instead of copying SR.
     */
    if ((sr & USART_SR_PE) != 0U)  error |= HAL_UART_ERROR_PE;
    if ((sr & USART_SR_NE) != 0U)  error |= HAL_UART_ERROR_NE;
    if ((sr & USART_SR_FE) != 0U)  error |= HAL_UART_ERROR_FE;
    if ((sr & USART_SR_ORE) != 0U) error |= HAL_UART_ERROR_ORE;
    return error;
}

static void motor_diag_capture_raw(const uint8_t *data, uint16_t len)
{
    uint16_t copy_len = len;

    s_last_diag.raw_rx_len = len;
    if (copy_len > MOTOR_DIAG_FRAME_MAX) {
        copy_len = MOTOR_DIAG_FRAME_MAX;
    }
    if (data != NULL && copy_len != 0U) {
        memcpy(s_last_diag.raw_rx, data, copy_len);
    }
}

static void motor_uart_tx_dma_stop(void)
{
#if MOTOR_MODBUS_TX_IMPL == MOTOR_MODBUS_TX_DMA_CONTIG
    /* USART3_TX is fixed to DMA1 Channel 2 on STM32F103. */
    CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAT);
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 |
                 DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2;
#endif
}

/*
 * Send one complete Modbus request without foreground byte-to-byte feeding.
 *
 * The field capture showed every no-response 0x10 request taking 1868--1907
 * us to transmit, while normal 19-byte requests consistently took about
 * 1650 us at 115200 8N1.  HAL_UART_Transmit() waits for TXE between bytes;
 * a long unrelated ISR can therefore create an RTU frame-internal gap that a
 * driver discards without replying.  DMA1 Channel 2 keeps DR supplied at the
 * hardware TXE cadence.  We wait for UART TC, not merely DMA TC, so the last
 * stop bit has left PB10 before an automatic RS485 direction circuit changes
 * back to receive.
 */
static HAL_StatusTypeDef motor_uart_transmit_frame(const uint8_t *data,
                                                   uint16_t len,
                                                   uint32_t timeout_ms)
{
#if MOTOR_MODBUS_TX_IMPL == MOTOR_MODBUS_TX_HAL_BLOCKING
    return HAL_UART_Transmit(&huart3, (uint8_t *)data, len, timeout_ms);
#else
    uint32_t start_ms;
    uint32_t dma_flags;
    volatile uint32_t sr;

    if (data == NULL || len == 0U) {
        return HAL_ERROR;
    }
    if (((READ_REG(DMA1_Channel2->CCR) & DMA_CCR_EN) != 0U) ||
        ((READ_REG(huart3.Instance->CR3) & USART_CR3_DMAT) != 0U)) {
        return HAL_BUSY;
    }

    __HAL_RCC_DMA1_CLK_ENABLE();
    motor_uart_tx_dma_stop();

    DMA1_Channel2->CPAR = (uint32_t)(uintptr_t)&huart3.Instance->DR;
    DMA1_Channel2->CMAR = (uint32_t)(uintptr_t)data;
    DMA1_Channel2->CNDTR = len;
    DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL;

    /*
     * The STM32F1 clears a previous TC through SR-read then DR-write.  Read
     * SR immediately before enabling DMA; the first DMA write to DR then
     * forms that sequence without reading DR (which could steal pre-armed
     * response data from the RX DMA path).
     */
    sr = READ_REG(huart3.Instance->SR);
    (void)sr;
    SET_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAT);

    start_ms = HAL_GetTick();
    do {
        dma_flags = READ_REG(DMA1->ISR);
        if ((dma_flags & DMA_ISR_TEIF2) != 0U) {
            motor_uart_tx_dma_stop();
            return HAL_ERROR;
        }
        if ((dma_flags & DMA_ISR_TCIF2) != 0U) {
            break;
        }
    } while ((HAL_GetTick() - start_ms) < timeout_ms);

    if ((dma_flags & DMA_ISR_TCIF2) == 0U) {
        motor_uart_tx_dma_stop();
        return HAL_TIMEOUT;
    }

    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start_ms) >= timeout_ms) {
            motor_uart_tx_dma_stop();
            return HAL_TIMEOUT;
        }
    }

    motor_uart_tx_dma_stop();
    return HAL_OK;
#endif
}

static void motor_uart_dma_stop(void)
{
    /*
     * DMA1 Channel 3 is the fixed STM32F103 USART3_RX mapping.  It is not
     * used by CubeMX in this project (USART2_RX uses Channel 6).
     */
    CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 |
                 DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;
    s_dma_rx_active = 0U;
}

static void motor_uart_dma_start(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    motor_uart_dma_stop();
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));

    DMA1_Channel3->CPAR = (uint32_t)(uintptr_t)&huart3.Instance->DR;
    DMA1_Channel3->CMAR = (uint32_t)(uintptr_t)s_dma_rx_buf;
    DMA1_Channel3->CNDTR = MB_DMA_RX_CAPACITY;
    /*
     * Peripheral-to-memory, byte-to-byte, memory increment, very high
     * priority.  No DMA interrupt is enabled; the foreground waits for a
     * complete Modbus candidate and the channel itself prevents ORE.
     */
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_PL;
    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart3.Instance->CR1, USART_CR1_RE);
    s_dma_rx_active = 1U;
}

static void motor_uart_flush_rx(void)
{
    (void)HAL_UART_AbortReceive(&huart3);
    motor_uart_disable_unused_irq();

    motor_uart_tx_dma_stop();
    motor_uart_dma_stop();
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_ERR);

#if MOTOR_MODBUS_RX_IMPL == MOTOR_MODBUS_RX_BLOCKING_BYTE
    /*
     * Match backup-current/main receive behavior: the UART receiver remains
     * enabled and the foreground later collects response bytes with
     * HAL_UART_Receive().  This mode is only for an A/B receive experiment.
     */
    SET_BIT(huart3.Instance->CR1, USART_CR1_RE);
#else
    /*
     * DMA scan mode leaves the receiver disabled after cleanup.
     * mb_transaction() will either pre-arm RX DMA before TX for echo-safe
     * commands, or keep RE low until the reply phase for 0x06 commands where
     * local echo is ambiguous.
     */
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RE);
#endif

    /*
     * A failed transaction can leave a late response in the UART peripheral.
     * Drain a bounded number of bytes before the next request so that a stale
     * CRC cannot be mistaken for the next motor's response.
     */
    for (uint8_t i = 0U; i < MOTOR_DIAG_FRAME_MAX; i++) {
        if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == RESET) {
            break;
        }
        (void)READ_REG(huart3.Instance->DR);
    }
    motor_uart_clear_status();

    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxXferCount = 0U;
    huart3.RxXferSize = 0U;
    huart3.RxState = HAL_UART_STATE_READY;
    huart3.ReceptionType = HAL_UART_RECEPTION_STANDARD;
}

static void motor_uart_enable_rx(void)
{
    SET_BIT(huart3.Instance->CR1, USART_CR1_RE);
    /*
     * RE was low during TX.  Preserve a clean RXNE byte because a fast
     * driver may already have started its reply by the time TC is observed;
     * only an error status proves that the latched byte stream is unusable.
     */
    if ((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) ||
        (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE) != RESET) ||
        (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE) != RESET) ||
        (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE) != RESET)) {
        motor_uart_clear_status();
    }
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxState = HAL_UART_STATE_READY;
}

#if MOTOR_MODBUS_RX_IMPL == MOTOR_MODBUS_RX_DMA_SCAN
static uint8_t motor_frame_crc_ok(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len < 5U) {
        return 0U;
    }

    uint16_t calc = modbus_crc16(frame, (uint16_t)(len - 2U));
    uint16_t recv = (uint16_t)frame[len - 2U] |
                    ((uint16_t)frame[len - 1U] << 8);
    return (calc == recv) ? 1U : 0U;
}

static uint16_t motor_uart_dma_count(void)
{
    uint16_t remaining = (uint16_t)READ_REG(DMA1_Channel3->CNDTR);
    /*
     * CNDTR is decremented after the peripheral byte has been moved.  Keep
     * subsequent CPU reads of s_dma_rx_buf behind that hardware observation.
     */
    __DMB();
    if (remaining > MB_DMA_RX_CAPACITY) {
        remaining = MB_DMA_RX_CAPACITY;
    }
    return (uint16_t)(MB_DMA_RX_CAPACITY - remaining);
}

/*
 * Find a complete valid response in the DMA stream.  This also tolerates a
 * byte or a local echo left by an automatic RS485 adapter: the request echo
 * does not have the response length/CRC and is skipped instead of shifting
 * the next motor transaction.
 */
static uint8_t motor_find_response_frame(const uint8_t *stream,
                                          uint16_t stream_len,
                                          uint8_t address,
                                          uint8_t function,
                                          uint16_t expected_len,
                                          uint16_t *offset,
                                          uint16_t *frame_len)
{
    if (stream == NULL || expected_len < 5U) {
        return 0U;
    }

    for (uint16_t i = 0U; i + 2U <= stream_len; i++) {
        if (stream[i] != address) {
            continue;
        }

        uint8_t response_function = stream[i + 1U];
        if (response_function == (uint8_t)(function | 0x80U)) {
            if ((uint16_t)(stream_len - i) >= 5U &&
                motor_frame_crc_ok(&stream[i], 5U)) {
                if (offset != NULL) *offset = i;
                if (frame_len != NULL) *frame_len = 5U;
                return 1U;
            }
        } else if (response_function == function &&
                   (uint16_t)(stream_len - i) >= expected_len &&
                   motor_frame_crc_ok(&stream[i], expected_len)) {
            if (offset != NULL) *offset = i;
            if (frame_len != NULL) *frame_len = expected_len;
            return 1U;
        }
    }
    return 0U;
}
#endif

/*
 * Receive one complete Modbus response with DMA1 Channel 3.  The old
 * one-byte HAL polling path could be pre-empted by other interrupts,
 * including MaixCAM USART2 processing, and overrun the STM32F1 one-byte DR
 * (the field log showed
 * uart_error=0x00000008).  DMA keeps every byte; the foreground only scans
 * the accumulated stream for an address/function/CRC-valid response.  When
 * DMA was armed before TX, this function keeps the captured echo/prefix and
 * searches through it instead of restarting the receiver.
 */
static HAL_StatusTypeDef motor_uart_receive_frame(uint8_t *data,
                                                   uint16_t expected_len,
                                                   uint8_t address,
                                                   uint8_t function,
                                                   uint16_t *received_len)
{
#if MOTOR_MODBUS_RX_IMPL == MOTOR_MODBUS_RX_BLOCKING_BYTE
    uint32_t first_tick = HAL_GetTick();
    uint16_t cnt = 0U;
    uint32_t error_flags_seen = 0U;

    (void)address;
    (void)function;

    if (data == NULL || expected_len == 0U ||
        expected_len > MOTOR_DIAG_FRAME_MAX) {
        if (received_len != NULL) *received_len = 0U;
        return HAL_ERROR;
    }

    memset(data, 0, expected_len);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxXferSize = expected_len;
    huart3.RxXferCount = expected_len;
    huart3.RxState = HAL_UART_STATE_READY;
    SET_BIT(huart3.Instance->CR1, USART_CR1_RE);

    /*
     * backup-current/main style receive path:
     * read one byte at a time after TX; stop when the expected response is
     * complete, or when a Modbus exception frame reaches 5 bytes.
     */
    while ((HAL_GetTick() - first_tick) < MB_RX_TIMEOUT &&
           cnt < expected_len) {
        uint8_t b = 0U;
        HAL_StatusTypeDef st =
            HAL_UART_Receive(&huart3, &b, 1U, 5U);

        error_flags_seen |= huart3.ErrorCode;
        error_flags_seen |=
            motor_uart_error_from_sr(READ_REG(huart3.Instance->SR));

        if (st == HAL_OK) {
            uint32_t rx_us = motor_transaction_us_now();
            if (cnt == 0U) {
                s_last_diag.first_rx_us = rx_us;
            }
            data[cnt++] = b;
            s_last_diag.last_rx_us = rx_us;
            huart3.RxXferCount = (uint16_t)(expected_len - cnt);
            if (cnt >= 2U && (data[1] & 0x80U) != 0U && cnt >= 5U) {
                break;
            }
        } else if (st == HAL_ERROR) {
            /*
             * Keep polling until the global transaction timeout expires.
             * Diagnostics retain the UART error bits for later comparison
             * against DMA_SCAN mode.
             */
            huart3.RxState = HAL_UART_STATE_READY;
        }
    }

    huart3.ErrorCode = error_flags_seen;
    s_last_diag.uart_sr = READ_REG(huart3.Instance->SR);
    huart3.RxXferCount = (uint16_t)(expected_len - cnt);
    huart3.RxState = HAL_UART_STATE_READY;
    motor_diag_capture_raw(data, cnt);
    s_last_diag.frame_offset = 0U;
    if (received_len != NULL) *received_len = cnt;

    if (cnt >= expected_len ||
        (cnt >= 5U && (data[1] & 0x80U) != 0U)) {
        return HAL_OK;
    }
    return (error_flags_seen != 0U) ? HAL_ERROR : HAL_TIMEOUT;
#else
    uint32_t first_tick = HAL_GetTick();
    uint16_t observed = 0U;
    uint16_t offset = 0U;
    uint16_t frame_len = 0U;
    uint32_t error_flags_seen = 0U;

    if (data == NULL || expected_len == 0U ||
        expected_len > MB_DMA_RX_CAPACITY) {
        if (received_len != NULL) *received_len = 0U;
        return HAL_ERROR;
    }

    memset(data, 0, expected_len);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxXferSize = expected_len;
    huart3.RxXferCount = expected_len;
    huart3.RxState = HAL_UART_STATE_BUSY_RX;

    if (s_dma_rx_active == 0U) {
        /*
         * For commands where pre-TX receive is not safe, RE was held low
         * during TX.  Drain only stale status/data before arming DMA, then
         * enable DMA and RE in that order so the first reply byte cannot be
         * missed.
         */
        CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RE);
        while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
            (void)READ_REG(huart3.Instance->DR);
        }
        motor_uart_clear_status();
        motor_uart_dma_start();
    }

    while ((HAL_GetTick() - first_tick) < MB_RX_TIMEOUT) {
        uint16_t count = motor_uart_dma_count();

        if (count > observed) {
            uint32_t rx_us = motor_transaction_us_now();
            if (observed == 0U) {
                s_last_diag.first_rx_us = rx_us;
            }
            observed = count;
            s_last_diag.last_rx_us = rx_us;
            huart3.RxXferCount = (uint16_t)
                                 ((expected_len > observed) ?
                                  (expected_len - observed) : 0U);
        }

        if (motor_find_response_frame(s_dma_rx_buf, observed,
                                      address, function, expected_len,
                                      &offset, &frame_len) != 0U) {
            if (frame_len > expected_len) {
                frame_len = expected_len;
            }
            error_flags_seen |=
                motor_uart_error_from_sr(READ_REG(huart3.Instance->SR));
            s_last_diag.uart_sr = READ_REG(huart3.Instance->SR);
            s_last_diag.dma_isr = READ_REG(DMA1->ISR);
            motor_diag_capture_raw(s_dma_rx_buf, observed);
            s_last_diag.frame_offset = offset;
            memcpy(data, &s_dma_rx_buf[offset], frame_len);
            motor_uart_dma_stop();
            huart3.RxXferCount = 0U;
            huart3.RxState = HAL_UART_STATE_READY;
            if (received_len != NULL) *received_len = frame_len;
            /*
             * A valid CRC-framed response is trustworthy even if an
             * unrelated echo/noise byte raised ORE in the same DMA window.
             * Keep the flag in diagnostics, but do not duplicate motion.
             */
            huart3.ErrorCode = error_flags_seen;
            return HAL_OK;
        }

        error_flags_seen |=
            motor_uart_error_from_sr(READ_REG(huart3.Instance->SR));

        /*
         * Do not shorten the global reply window after seeing only a prefix.
         * With receive armed before TX, a transceiver echo or a noise byte can
         * arrive before the real slave reply.  Treating that byte as the start
         * of a response and applying a short inter-byte timeout would turn the
         * 400 ms transaction window into a false early timeout.
         */
        if (observed >= MB_DMA_RX_CAPACITY) {
            break;
        }
    }

    /*
     * Preserve the raw prefix for the existing diagnostic print, then stop
     * DMA/RE before clearing SR/DR.  A partial or CRC-invalid relative
     * response remains a hard failure; no duplicate buffered command is sent.
     */
    uint16_t raw_count = motor_uart_dma_count();
    if (raw_count > MB_DMA_RX_CAPACITY) raw_count = MB_DMA_RX_CAPACITY;
    if (raw_count > observed) {
        uint32_t rx_us = motor_transaction_us_now();
        if (observed == 0U) {
            s_last_diag.first_rx_us = rx_us;
        }
        s_last_diag.last_rx_us = rx_us;
    }
    s_last_diag.uart_sr = READ_REG(huart3.Instance->SR);
    s_last_diag.dma_isr = READ_REG(DMA1->ISR);
    motor_diag_capture_raw(s_dma_rx_buf, raw_count);
    s_last_diag.frame_offset = 0xFFFFU;
    motor_uart_dma_stop();
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RE);
    uint16_t parser_count = raw_count;
    if (parser_count > expected_len) parser_count = expected_len;
    if (parser_count != 0U) memcpy(data, s_dma_rx_buf, parser_count);
    huart3.ErrorCode = error_flags_seen;
    if (error_flags_seen != 0U) {
        motor_uart_clear_status();
    }
    huart3.RxXferCount = (uint16_t)(expected_len - parser_count);
    huart3.RxState = HAL_UART_STATE_READY;
    if (received_len != NULL) *received_len = parser_count;
    return (error_flags_seen != 0U) ? HAL_ERROR : HAL_TIMEOUT;
#endif
}

static void motor_diag_record_tx_only(const uint8_t *tx,
                                      uint16_t tx_len,
                                      mb_result_t result,
                                      uint32_t elapsed_ms)
{
    motor_diag_reset(tx, tx_len, 0U);
    s_last_diag.tx_ok = (result == MB_OK) ? 1U : 0U;
    s_last_diag.result = result;
    s_last_diag.elapsed_ms = elapsed_ms;
}

static void motor_print_hex(const char *label,
                            const uint8_t *data,
                            uint16_t len)
{
    printf("%s", (label != NULL) ? label : "");
    for (uint16_t i = 0U; i < len; i++) {
        printf("%02X", (unsigned int)data[i]);
        if ((i + 1U) < len) {
            putchar(' ');
        }
    }
}

void motor_print_last_transaction_diag(const char *prefix)
{
    const char *p = (prefix != NULL) ? prefix : "motor";

    printf("%s diag id=%lu result=%s addr=%u func=0x%02X "
           "reg=0x%04X qty=%u tx_len=%u rx_len=%u raw_len=%u "
           "expect=%u off=%u elapsed=%lums "
           "t_us=[tx:%lu first:%lu last:%lu]",
           p,
           (unsigned long)s_last_diag.transaction_id,
           motor_result_str(s_last_diag.result),
           (unsigned int)s_last_diag.addr,
           (unsigned int)s_last_diag.function,
           (unsigned int)s_last_diag.request_register,
           (unsigned int)s_last_diag.request_quantity,
           (unsigned int)s_last_diag.tx_len,
           (unsigned int)s_last_diag.rx_len,
           (unsigned int)s_last_diag.raw_rx_len,
           (unsigned int)s_last_diag.expected_rx_len,
           (unsigned int)s_last_diag.frame_offset,
           (unsigned long)s_last_diag.elapsed_ms,
           (unsigned long)s_last_diag.tx_time_us,
           (unsigned long)s_last_diag.first_rx_us,
           (unsigned long)s_last_diag.last_rx_us);
    if (s_last_diag.exception_code != 0U) {
        printf(" exception=0x%02X", (unsigned int)s_last_diag.exception_code);
    }
    if (s_last_diag.crc_calc != 0U || s_last_diag.crc_recv != 0U) {
        printf(" crc_calc=0x%04X crc_recv=0x%04X",
               (unsigned int)s_last_diag.crc_calc,
               (unsigned int)s_last_diag.crc_recv);
    }
    if (s_last_diag.uart_error != HAL_UART_ERROR_NONE) {
        printf(" uart_error=0x%08lX",
               (unsigned long)s_last_diag.uart_error);
    }
    printf(" sr=0x%08lX dma_isr=0x%08lX",
           (unsigned long)s_last_diag.uart_sr,
           (unsigned long)s_last_diag.dma_isr);
    printf("\r\n");
    motor_print_hex("  tx=", s_last_diag.tx, s_last_diag.tx_len);
    printf("\r\n");
    motor_print_hex("  rx=", s_last_diag.rx, s_last_diag.rx_len);
    printf("\r\n");
    motor_print_hex("  raw=", s_last_diag.raw_rx,
                    (s_last_diag.raw_rx_len > MOTOR_DIAG_FRAME_MAX) ?
                    MOTOR_DIAG_FRAME_MAX : s_last_diag.raw_rx_len);
    if (s_last_diag.raw_rx_len > MOTOR_DIAG_FRAME_MAX) {
        printf(" ...");
    }
    printf("\r\n");
}

/* 实测卷线方向:
 *   M1/M3: CCW=收线, CW=放线
 *   M2/M4: CW =收线, CCW=放线 */
uint8_t motor_cable_direction(uint8_t addr, uint8_t cable_action)
{
    uint8_t takeup_dir = ((addr == 2U) || (addr == 4U)) ? MB_DIR_CW : MB_DIR_CCW;

    if (cable_action == MOTOR_CABLE_TAKEUP) return takeup_dir;
    return (takeup_dir == MB_DIR_CW) ? MB_DIR_CCW : MB_DIR_CW;
}

/* ---------- 一次完整事务: 发请求 + 收应答并校验 ----------
 * 注意: 本函数只发一次, 不做任何重发(避免相对命令被重复累加) */
static mb_result_t mb_transaction(uint8_t *req, uint16_t req_payload_len,
                                  uint8_t *rsp, uint16_t rsp_len)
{
    uint8_t tx[MB_BUF_SIZE];
    uint32_t start_ms = HAL_GetTick();

    if (req == NULL || rsp == NULL ||
        req_payload_len + 2U > MB_BUF_SIZE ||
        rsp_len > MOTOR_DIAG_FRAME_MAX) {
        motor_diag_reset(req, req_payload_len, rsp_len);
        return motor_diag_finish(MB_ERR_FRAME, start_ms);
    }

    memcpy(tx, req, req_payload_len);
    uint16_t crc = modbus_crc16(tx, req_payload_len);
    tx[req_payload_len]   = (uint8_t)(crc & 0xFF);
    tx[req_payload_len+1] = (uint8_t)(crc >> 8);
    uint16_t tx_total = req_payload_len + 2;

    motor_diag_reset(tx, tx_total, rsp_len);
    s_transaction_start_cycles = motor_cycles_now();
    motor_uart_flush_rx();

    /*
     * Arm RX before TX for reads and 0x10 writes.  Automatic-direction
     * RS485 adapters can let a fast driver reply immediately after TC; if
     * DMA is started only after transmit completion, field logs can
     * show rx_len=0 even though the driver answered.  Do not pre-arm 0x06:
     * its request echo is byte-for-byte identical to a normal response.
     */
#if MOTOR_MODBUS_RX_IMPL == MOTOR_MODBUS_RX_DMA_SCAN
    uint8_t prearm_rx = (req[1] != MB_FUNC_WRITE1) ? 1U : 0U;
#else
    uint8_t prearm_rx = 0U;
#endif
    if (prearm_rx != 0U) {
        motor_uart_dma_start();
    }

    {
        uint32_t tx_start_cycles = motor_cycles_now();
        HAL_StatusTypeDef tx_status =
            motor_uart_transmit_frame(tx, tx_total, MB_TX_TIMEOUT);
        s_last_diag.tx_time_us =
            motor_cycles_to_us(motor_cycles_now() - tx_start_cycles);
        if (tx_status != HAL_OK) {
            /*
             * HAL_BUSY is returned before HAL starts this blocking transfer.
             * HAL_TIMEOUT/HAL_ERROR may occur after a partial frame and are
             * therefore ambiguous for a relative buffered command.
             */
            motor_uart_dma_stop();
            motor_uart_enable_rx();
            return motor_diag_finish((tx_status == HAL_BUSY) ?
                                     MB_ERR_BUSY : MB_ERR_TX, start_ms);
        }
    }
    s_last_diag.tx_ok = 1U;

    /*
     * Receive the response through the selected implementation.  A normal
     * write response is 8 bytes; an exception frame is still accepted below
     * after this call returns with 5 bytes.  The receiver records partial
     * bytes and STM32F1 UART error flags without allowing a stale HAL receive
     * state to carry into the next motor.
     */
    memset(rsp, 0, rsp_len);
    {
        uint16_t cnt = 0U;
        HAL_StatusTypeDef rx_status =
            motor_uart_receive_frame(rsp, rsp_len, req[0], req[1], &cnt);
        if (cnt > MOTOR_DIAG_FRAME_MAX) {
            cnt = MOTOR_DIAG_FRAME_MAX;
        }
        s_last_diag.rx_len = cnt;
        if (cnt != 0U) {
            memcpy(s_last_diag.rx, rsp, cnt);
        }
        s_last_diag.uart_error = huart3.ErrorCode;

        if (rx_status != HAL_OK && cnt < rsp_len &&
            !(cnt >= 5U && (rsp[1] & 0x80U) != 0U)) {
            return motor_diag_finish(MB_ERR_TIMEOUT, start_ms);
        }
    }

    uint16_t cnt = s_last_diag.rx_len;
    if (cnt >= 1U && rsp[0] != req[0]) {
        s_last_diag.response_complete = (cnt >= rsp_len) ? 1U : 0U;
        return motor_diag_finish(MB_ERR_FRAME, start_ms);
    }

    if (cnt >= 2U && rsp[1] != (uint8_t)(req[1] | 0x80U) &&
        rsp[1] != req[1]) {
        s_last_diag.response_complete = (cnt >= rsp_len) ? 1U : 0U;
        return motor_diag_finish(MB_ERR_FRAME, start_ms);
    }

    if (cnt >= 2U && (rsp[1] & 0x80U) != 0U) {
        if (cnt < 5U) {
            return motor_diag_finish(MB_ERR_TIMEOUT, start_ms);
        }
        s_last_diag.response_complete = 1U;
        s_last_diag.exception_code = rsp[2];
        s_last_diag.crc_calc = modbus_crc16(rsp, 3U);
        s_last_diag.crc_recv = (uint16_t)rsp[3] |
                               ((uint16_t)rsp[4] << 8);
        if (s_last_diag.crc_calc != s_last_diag.crc_recv) {
            return motor_diag_finish(MB_ERR_CRC, start_ms);
        }
        return motor_diag_finish(MB_ERR_REJECT, start_ms);
    }

    if (cnt < rsp_len) {
        return motor_diag_finish(MB_ERR_TIMEOUT, start_ms);
    }

    s_last_diag.response_complete = 1U;
    s_last_diag.crc_calc = modbus_crc16(rsp, rsp_len - 2U);
    s_last_diag.crc_recv = (uint16_t)rsp[rsp_len - 2U] |
                           ((uint16_t)rsp[rsp_len - 1U] << 8);
    if (s_last_diag.crc_calc != s_last_diag.crc_recv) {
        return motor_diag_finish(MB_ERR_CRC, start_ms);
    }
    if (req[1] == MB_FUNC_WRITEN || req[1] == MB_FUNC_WRITE1) {
        if (rsp[2] != req[2] || rsp[3] != req[3] ||
            rsp[4] != req[4] || rsp[5] != req[5]) {
            return motor_diag_finish(MB_ERR_FRAME, start_ms);
        }
    } else if (req[1] == MB_FUNC_READ || req[1] == 0x04U) {
        uint16_t qty = ((uint16_t)req[4] << 8) | req[5];
        uint16_t byte_count = (uint16_t)(qty * 2U);
        if (byte_count > 255U || rsp[2] != (uint8_t)byte_count) {
            return motor_diag_finish(MB_ERR_FRAME, start_ms);
        }
    }
    return motor_diag_finish(MB_OK, start_ms);
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
    f[n++] = 0x00; f[n++] = dir;             /* reg1: 00=CW, 01=CCW; 收放线映射见 motor_cable_direction */
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

/* 0x00F6 speed mode. The driver is configured for 0.1RPM speed input,
 * matching the unit used by motor_move_pos(). */
mb_result_t motor_run_speed(uint8_t addr, uint8_t dir,
                            uint16_t speed_0p1rpm,
                            uint8_t acc, uint8_t sync)
{
    uint8_t f[16];
    uint8_t n = 0;

    f[n++] = addr;
    f[n++] = MB_FUNC_WRITEN;
    f[n++] = (uint8_t)(REG_SPEED_MODE >> 8);
    f[n++] = (uint8_t)(REG_SPEED_MODE & 0xFF);
    f[n++] = 0x00; f[n++] = 0x03;
    f[n++] = 0x06;
    f[n++] = dir;
    f[n++] = (uint8_t)(speed_0p1rpm >> 8);
    f[n++] = (uint8_t)(speed_0p1rpm & 0xFF);
    f[n++] = acc;
    f[n++] = sync;
    f[n++] = 0x00;

    uint8_t rsp[8];
    return mb_transaction(f, n, rsp, 8);
}

/* 广播多机同步触发: 地址0, 06 00FF 6600 (电机不应答, 只发不收) */
mb_result_t motor_sync_trigger(void)
{
    uint8_t f[8];
    uint32_t start_ms = HAL_GetTick();
    f[0] = 0x00;
    f[1] = MB_FUNC_WRITE1;
    f[2] = (uint8_t)(REG_SYNC >> 8);
    f[3] = (uint8_t)(REG_SYNC & 0xFF);
    f[4] = 0x66; f[5] = 0x00;
    uint16_t crc = modbus_crc16(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)(crc >> 8);
    motor_uart_flush_rx();
    HAL_StatusTypeDef tx_status =
        motor_uart_transmit_frame(f, 8U, MB_TX_TIMEOUT);
    if (tx_status != HAL_OK) {
        motor_uart_enable_rx();
        mb_result_t result = (tx_status == HAL_BUSY) ?
                             MB_ERR_BUSY : MB_ERR_TX;
        motor_diag_record_tx_only(f, 8U, result,
                                  HAL_GetTick() - start_ms);
        return result;
    }
    motor_uart_enable_rx();
    motor_diag_record_tx_only(f, 8U, MB_OK, HAL_GetTick() - start_ms);
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

static mb_result_t motor_read_registers_04(uint8_t addr,
                                            uint16_t reg,
                                            uint16_t quantity,
                                            uint8_t *data,
                                            uint16_t data_capacity)
{
    uint8_t f[6];
    uint8_t rsp[MOTOR_DIAG_FRAME_MAX];
    uint16_t byte_count;
    uint16_t rsp_len;

    if (data == NULL || quantity == 0U) {
        return MB_ERR_FRAME;
    }
    byte_count = (uint16_t)(quantity * 2U);
    rsp_len = (uint16_t)(byte_count + 5U);
    if (byte_count > data_capacity || rsp_len > MOTOR_DIAG_FRAME_MAX) {
        return MB_ERR_FRAME;
    }

    f[0] = addr;
    f[1] = 0x04U;
    f[2] = (uint8_t)(reg >> 8);
    f[3] = (uint8_t)(reg & 0xFFU);
    f[4] = (uint8_t)(quantity >> 8);
    f[5] = (uint8_t)(quantity & 0xFFU);

    mb_result_t result = mb_transaction(f, sizeof(f), rsp, rsp_len);
    if (result != MB_OK) {
        return result;
    }
    memcpy(data, &rsp[3], byte_count);
    return MB_OK;
}

/* Manual V1.0.0_251204, X firmware:
 * version register 0x0010, two input registers, function 0x04.
 * firmware/hardware words are both transmitted big-endian. */
mb_result_t motor_read_identity_x(uint8_t addr, motor_identity_t *identity)
{
    uint8_t data[4];
    mb_result_t result;

    if (identity == NULL) {
        return MB_ERR_FRAME;
    }
    memset(identity, 0, sizeof(*identity));
    result = motor_read_registers_04(addr, 0x0010U, 2U,
                                     data, sizeof(data));
    if (result != MB_OK) {
        return result;
    }

    identity->firmware_raw = ((uint16_t)data[0] << 8) | data[1];
    identity->hardware_raw = ((uint16_t)data[2] << 8) | data[3];
    identity->series_code = (uint8_t)((identity->hardware_raw >> 12) & 0x0FU);
    identity->type_code = (uint8_t)((identity->hardware_raw >> 8) & 0x0FU);
    identity->hardware_version = (uint8_t)(identity->hardware_raw & 0xFFU);
    return MB_OK;
}

/* Manual common register 0x0024, result unit mV. */
mb_result_t motor_read_bus_voltage(uint8_t addr, uint16_t *millivolts)
{
    uint8_t data[2];
    mb_result_t result;

    if (millivolts == NULL) {
        return MB_ERR_FRAME;
    }
    result = motor_read_registers_04(addr, 0x0024U, 1U,
                                     data, sizeof(data));
    if (result == MB_OK) {
        *millivolts = ((uint16_t)data[0] << 8) | data[1];
    }
    return result;
}

/* Manual X firmware motor-state register 0x0050.
 * Low byte bits: enabled/reached/stall/stall-protection/limits/overcurrent. */
mb_result_t motor_read_state_x(uint8_t addr, uint8_t *state)
{
    uint8_t data[2];
    mb_result_t result;

    if (state == NULL) {
        return MB_ERR_FRAME;
    }
    result = motor_read_registers_04(addr, 0x0050U, 1U,
                                     data, sizeof(data));
    if (result == MB_OK) {
        *state = data[1];
    }
    return result;
}

/* Manual 3.5.24: heartbeat protection time, register 0x0016.
 * The reply table contains one 16-bit register, unit ms. */
mb_result_t motor_read_heartbeat_time_x(uint8_t addr, uint32_t *milliseconds)
{
    uint8_t data[2];
    mb_result_t result;

    if (milliseconds == NULL) {
        return MB_ERR_FRAME;
    }
    result = motor_read_registers_04(addr, 0x0016U, 1U,
                                     data, sizeof(data));
    if (result != MB_OK) {
        return result;
    }
    *milliseconds = ((uint16_t)data[0] << 8) | data[1];
    return MB_OK;
}

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
