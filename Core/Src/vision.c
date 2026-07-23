/* ============================================================
 *  vision.c — MaixCAM Pro 视觉数据接收 (USART2 + DMA + IDLE)
 * ============================================================ */
#include "vision.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>   /* qsort */
#include <stdio.h>

extern UART_HandleTypeDef huart2;          /* CubeMX 生成的 USART2 句柄 */

/* ---- DMA 环形接收缓冲 (硬件自动写) ---- */
#define RX2_SIZE   256
static uint8_t  rx2_dma[RX2_SIZE];
static uint16_t rx2_rd = 0;                /* 软件已处理到的位置 */

/* ---- 单包装配缓冲 ($...# 之间的内容) ---- */
#define VISION_LINE_MAX   VISION_FRAME_TEXT_MAX
static char    line[VISION_LINE_MAX];
static uint8_t line_len = 0;
static uint8_t in_frame = 0;

/* ---- 对外数据 ---- */
vision_data_t g_vision;

static void copy_frame_text(char *dst, const char *src)
{
    uint8_t i = 0;
    while (src[i] != '\0' && i < (VISION_FRAME_TEXT_MAX - 1U)) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void record_bad_frame(vision_parse_error_t reason, const char *frame)
{
    copy_frame_text(g_vision.last_bad_frame, frame);
    g_vision.last_bad_reason = reason;
    g_vision.bad_frame_count++;
}

static void record_unknown_frame(const char *frame)
{
    copy_frame_text(g_vision.last_unknown_frame, frame);
    g_vision.unknown_count++;
}

static uint8_t parse_float_strict(const char *text, float *value)
{
    char *end = NULL;
    float parsed;

    if (text == NULL || *text == '\0') return 0;
    parsed = strtof(text, &end);
    if (end == text || *end != '\0' || !isfinite(parsed)) return 0;

    *value = parsed;
    return 1;
}

static void mark_valid_frame(uint32_t now)
{
    g_vision.valid_frame_count++;
    g_vision.last_valid_ms = now;
}

static uint8_t is_home_tag(const char *tag)
{
    return (strcmp(tag, "HOME") == 0 ||
            strcmp(tag, "CENTER") == 0 ||
            strcmp(tag, "ORIGIN") == 0 ||
            strcmp(tag, "OFFSET") == 0 ||
            strcmp(tag, "ERR") == 0) ? 1U : 0U;
}

/* ---------- 初始化 ---------- */
void vision_init(void)
{
    HAL_StatusTypeDef status;

    memset(&g_vision, 0, sizeof(g_vision));
    rx2_rd = 0; line_len = 0; in_frame = 0;

    /* 启动 DMA + IDLE 接收, 整个程序只调这一次 */
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_dma, RX2_SIZE);
    g_vision.dma_start_status = (uint8_t)status;
    if (status == HAL_OK && huart2.hdmarx != NULL) {
        g_vision.dma_started = 1;
        /* 不需要 DMA 半满中断 */
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

/* ---------- 解析一包 (line 里是 $ 和 # 之间的内容) ---------- */
static void parse_packet(char *s)
{
    uint32_t now = HAL_GetTick();
    char raw[VISION_FRAME_TEXT_MAX];
    char *end = NULL;
    long hb;

    g_vision.last_rx_ms = now;
    g_vision.frame_count++;
    copy_frame_text(raw, s);

    if (*s == '\0') {
        record_bad_frame(VISION_PARSE_ERROR_EMPTY, raw);
        return;
    }

    char *p = strchr(s, ',');
    if (p) { *p = '\0'; p++; }
    char *tag = s;

    if (strcmp(tag, "HB") == 0) {
        if (p == NULL || *p == '\0') {
            record_bad_frame(VISION_PARSE_ERROR_MISSING_FIELD, raw);
            return;
        }
        hb = strtol(p, &end, 10);
        if (end == p || *end != '\0' || hb < INT_MIN || hb > INT_MAX) {
            record_bad_frame(VISION_PARSE_ERROR_INVALID_NUMBER, raw);
            return;
        }
        g_vision.hb_value   = (int)hb;
        g_vision.hb_pending = 1;
        g_vision.hb_count++;
        mark_valid_frame(now);
        return;
    }

    if (strcmp(tag, "LASER") != 0 && is_home_tag(tag) == 0U &&
        strcmp(tag, "FIRE") != 0 &&
        strcmp(tag, "CIRCLE") != 0 && strcmp(tag, "TILT") != 0) {
        record_unknown_frame(raw);
        return;
    }

    if (!p) {
        record_bad_frame(VISION_PARSE_ERROR_MISSING_FIELD, raw);
        return;
    }

    char *q = strchr(p, ',');
    if (!q) {
        record_bad_frame(VISION_PARSE_ERROR_MISSING_FIELD, raw);
        return;
    }
    *q = '\0';

    float x;
    float y;
    if (!parse_float_strict(p, &x) || !parse_float_strict(q + 1, &y)) {
        record_bad_frame(VISION_PARSE_ERROR_INVALID_NUMBER, raw);
        return;
    }

    if (strcmp(tag, "LASER") == 0) {
        g_vision.laser_x = x; g_vision.laser_y = y;
        g_vision.laser_fresh = 1; g_vision.laser_ms = now;
        g_vision.laser_count++;
    } else if (is_home_tag(tag) != 0U) {
        g_vision.home_x = x; g_vision.home_y = y;
        g_vision.home_fresh = 1; g_vision.home_ms = now;
        g_vision.home_count++;
    } else if (strcmp(tag, "FIRE") == 0) {
        g_vision.fire_x = x; g_vision.fire_y = y;
        g_vision.fire_fresh = 1; g_vision.fire_ms = now;
        g_vision.fire_count++;
    } else if (strcmp(tag, "CIRCLE") == 0) {
        g_vision.circle_x = x; g_vision.circle_y = y;
        g_vision.circle_fresh = 1; g_vision.circle_ms = now;
        g_vision.circle_count++;
    } else if (strcmp(tag, "TILT") == 0) {
        g_vision.tilt_x = x; g_vision.tilt_y = y;
        g_vision.tilt_fresh = 1; g_vision.tilt_ms = now;
        g_vision.tilt_count++;
    }
    mark_valid_frame(now);
}

/* ---------- 逐字节拼包 ---------- */
static void feed_char(uint8_t c)
{
    g_vision.rx_byte_count++;
    g_vision.last_byte_ms = HAL_GetTick();

    if (c == '$') {
        if (in_frame && line_len > 0U) {
            line[line_len] = '\0';
            record_bad_frame(VISION_PARSE_ERROR_RESTARTED, line);
        }
        in_frame = 1; line_len = 0;
        return;
    }
    if (!in_frame) {
        if (c == '#') record_bad_frame(VISION_PARSE_ERROR_NO_START, "#");
        return;
    }
    if (c == '#') {
        line[line_len] = '\0';
        parse_packet(line);
        in_frame = 0; line_len = 0;
        return;
    }
    if (c == '\r' || c == '\n') return;    /* 忽略换行 */
    if (line_len < VISION_LINE_MAX - 1U) {
        line[line_len++] = (char)c;
    } else {                               /* 超长 -> 丢弃本包 */
        line[line_len] = '\0';
        record_bad_frame(VISION_PARSE_ERROR_TOO_LONG, line);
        in_frame = 0; line_len = 0;
    }
}

/* ---------- HAL 回调: IDLE / 缓冲满 时被调用 ----------
 * Size = DMA 已写入到的位置(从缓冲头算起的字节数) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;

    /* 从上次处理位置 rx2_rd 一直处理到 Size, 环形 */
    if (Size >= RX2_SIZE) {
        while (rx2_rd < RX2_SIZE) {
            feed_char(rx2_dma[rx2_rd++]);
        }
        rx2_rd = 0;
        return;
    }

    while (rx2_rd != Size) {
        feed_char(rx2_dma[rx2_rd]);
        rx2_rd++;
        if (rx2_rd >= RX2_SIZE) rx2_rd = 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    g_vision.last_uart_error = HAL_UART_GetError(huart);
    g_vision.uart_error_count++;
}

/* ---------- 主循环任务: 回发心跳 ---------- */
void vision_task(void)
{
    if (g_vision.hb_pending) {
        g_vision.hb_pending = 0;
        char out[20];
        int n = 0;
        out[n++] = '$'; out[n++] = 'H'; out[n++] = 'B'; out[n++] = ',';
        /* 把 hb_value 转成十进制字符串 */
        int v = g_vision.hb_value;
        char num[12]; int ni = 0;
        if (v == 0) num[ni++] = '0';
        else {
            int neg = v < 0; if (neg) v = -v;
            while (v > 0 && ni < 11) { num[ni++] = (char)('0' + v % 10); v /= 10; }
            if (neg) num[ni++] = '-';
        }
        while (ni > 0) out[n++] = num[--ni];
        out[n++] = '#'; out[n++] = '\n';
        HAL_UART_Transmit(&huart2, (uint8_t *)out, n, 20);
    }
}

uint8_t vision_send_mode(const char *mode)
{
    char out[24];
    int n;

    if (mode == NULL || *mode == '\0') {
        return 0U;
    }

    n = snprintf(out, sizeof(out), "&mode,%s#\n", mode);
    if (n <= 0 || n >= (int)sizeof(out)) {
        return 0U;
    }

    return (HAL_UART_Transmit(&huart2, (uint8_t *)out, (uint16_t)n, 30) == HAL_OK) ? 1U : 0U;
}

/* ---------- 链路在线判断 ---------- */
uint8_t vision_is_online(uint32_t timeout_ms)
{
    if (g_vision.last_valid_ms == 0) return 0;       /* 从没收到过合法帧 */
    return (HAL_GetTick() - g_vision.last_valid_ms) <= timeout_ms;
}

/* MaixCAM LASER 发包仅 ~10Hz (其 config.py serial_tx_interval_ms=100)。
 * 取 5 帧中值: 10Hz 凑 5 帧约 500ms, 窗口给到 900ms 容忍丢帧/降速;
 * 最少需要 VIS_N/2+1 = 3 帧才有效。原来 9 帧/300ms 在 10Hz 下永远凑不齐。 */
#define VIS_N        5
#define VIS_WAIT_MS  900
#define VIS_PAIR_MAX_MS 80U
static int cmp_f(const void *a, const void *b){ float d=*(const float*)a-*(const float*)b; return (d>0)-(d<0); }
static uint32_t abs_u32_diff(uint32_t a, uint32_t b){ return (a >= b) ? (a - b) : (b - a); }

uint8_t vision_read_filtered(float *x, float *y)
{
    float bx[VIS_N], by[VIS_N]; uint8_t n = 0;
    uint32_t t0 = HAL_GetTick();
    while (n < VIS_N && (HAL_GetTick() - t0) < VIS_WAIT_MS) {
        if (g_vision.laser_fresh) { g_vision.laser_fresh = 0; bx[n]=g_vision.laser_x; by[n]=g_vision.laser_y; n++; }
    }
    if (n < (VIS_N/2 + 1)) return 0;
    qsort(bx, n, sizeof(float), cmp_f);
    qsort(by, n, sizeof(float), cmp_f);
    *x = bx[n/2]; *y = by[n/2];
    return 1;
}

uint8_t vision_read_circle_error_with_ref_filtered(float *err_x, float *err_y,
                                                   float *circle_x, float *circle_y)
{
    float bx[VIS_N], by[VIS_N], cx[VIS_N], cy[VIS_N]; uint8_t n = 0;
    uint32_t last_laser_count = 0U;
    uint32_t last_circle_count = 0U;
    uint32_t t0 = HAL_GetTick();

    while (n < VIS_N && (HAL_GetTick() - t0) < VIS_WAIT_MS) {
        uint32_t laser_count = g_vision.laser_count;
        uint32_t circle_count = g_vision.circle_count;

        if (laser_count == 0U || circle_count == 0U) {
            continue;
        }
        if (laser_count == last_laser_count || circle_count == last_circle_count) {
            continue;
        }
        if (abs_u32_diff(g_vision.laser_ms, g_vision.circle_ms) > VIS_PAIR_MAX_MS) {
            continue;
        }

        bx[n] = g_vision.laser_x - g_vision.circle_x;
        by[n] = g_vision.laser_y - g_vision.circle_y;
        cx[n] = g_vision.circle_x;
        cy[n] = g_vision.circle_y;
        n++;
        last_laser_count = laser_count;
        last_circle_count = circle_count;
    }

    if (n < (VIS_N/2 + 1)) return 0;
    qsort(bx, n, sizeof(float), cmp_f);
    qsort(by, n, sizeof(float), cmp_f);
    qsort(cx, n, sizeof(float), cmp_f);
    qsort(cy, n, sizeof(float), cmp_f);
    if (err_x != NULL) *err_x = bx[n/2];
    if (err_y != NULL) *err_y = by[n/2];
    if (circle_x != NULL) *circle_x = cx[n/2];
    if (circle_y != NULL) *circle_y = cy[n/2];
    return 1;
}

uint8_t vision_read_circle_error_filtered(float *x, float *y)
{
    return vision_read_circle_error_with_ref_filtered(x, y, NULL, NULL);
}

uint8_t vision_read_home_filtered(float *x, float *y)
{
    float bx[VIS_N], by[VIS_N]; uint8_t n = 0;
    uint32_t t0 = HAL_GetTick();
    while (n < VIS_N && (HAL_GetTick() - t0) < VIS_WAIT_MS) {
        if (g_vision.home_fresh) {
            g_vision.home_fresh = 0;
            bx[n] = g_vision.home_x;
            by[n] = g_vision.home_y;
            n++;
        }
    }
    if (n < (VIS_N/2 + 1)) return 0;
    qsort(bx, n, sizeof(float), cmp_f);
    qsort(by, n, sizeof(float), cmp_f);
    *x = bx[n/2]; *y = by[n/2];
    return 1;
}
