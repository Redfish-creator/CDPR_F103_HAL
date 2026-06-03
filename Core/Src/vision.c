/* ============================================================
 *  vision.c — MaixCAM Pro 视觉数据接收 (USART2 + DMA + IDLE)
 * ============================================================ */
#include "vision.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;          /* CubeMX 生成的 USART2 句柄 */

/* ---- DMA 环形接收缓冲 (硬件自动写) ---- */
#define RX2_SIZE   256
static uint8_t  rx2_dma[RX2_SIZE];
static uint16_t rx2_rd = 0;                /* 软件已处理到的位置 */

/* ---- 单包装配缓冲 ($...# 之间的内容) ---- */
#define LINE_MAX   48
static char    line[LINE_MAX];
static uint8_t line_len = 0;
static uint8_t in_frame = 0;

/* ---- 对外数据 ---- */
vision_data_t g_vision;

/* ---------- 初始化 ---------- */
void vision_init(void)
{
    memset(&g_vision, 0, sizeof(g_vision));
    rx2_rd = 0; line_len = 0; in_frame = 0;

    /* 启动 DMA + IDLE 接收, 整个程序只调这一次 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_dma, RX2_SIZE);
    /* 不需要 DMA 半满中断 */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

/* ---------- 解析一包 (line 里是 $ 和 # 之间的内容) ---------- */
static void parse_packet(char *s)
{
    uint32_t now = HAL_GetTick();
    g_vision.last_rx_ms = now;

    /* 取出 TAG (逗号前) */
    char *p = strchr(s, ',');
    if (p) { *p = '\0'; p++; }             /* p 现在指向第一个参数 */
    char *tag = s;

    /* 心跳: $HB,n# */
    if (strcmp(tag, "HB") == 0) {
        g_vision.hb_value   = p ? atoi(p) : 0;
        g_vision.hb_pending = 1;
        return;
    }
    if (!p) return;                        /* 其余包都需要参数 */

    /* 解析 x,y 两个浮点 */
    float x = (float)atof(p);
    char *q = strchr(p, ',');
    float y = q ? (float)atof(q + 1) : 0.0f;

    if (strcmp(tag, "LASER") == 0) {
        g_vision.laser_x = x; g_vision.laser_y = y;
        g_vision.laser_fresh = 1; g_vision.laser_ms = now;
    } else if (strcmp(tag, "FIRE") == 0) {
        g_vision.fire_x = x; g_vision.fire_y = y;
        g_vision.fire_fresh = 1; g_vision.fire_ms = now;
    } else if (strcmp(tag, "CIRCLE") == 0) {
        g_vision.circle_x = x; g_vision.circle_y = y;
        g_vision.circle_fresh = 1;
    } else if (strcmp(tag, "TILT") == 0) {
        g_vision.tilt_x = x; g_vision.tilt_y = y;
        g_vision.tilt_fresh = 1;
    }
}

/* ---------- 逐字节拼包 ---------- */
static void feed_char(uint8_t c)
{
    if (c == '$') { in_frame = 1; line_len = 0; return; }
    if (!in_frame) return;
    if (c == '#') {
        line[line_len] = '\0';
        parse_packet(line);
        in_frame = 0; line_len = 0;
        return;
    }
    if (c == '\r' || c == '\n') return;    /* 忽略换行 */
    if (line_len < LINE_MAX - 1) {
        line[line_len++] = (char)c;
    } else {                               /* 超长 -> 丢弃本包 */
        in_frame = 0; line_len = 0;
    }
}

/* ---------- HAL 回调: IDLE / 缓冲满 时被调用 ----------
 * Size = DMA 已写入到的位置(从缓冲头算起的字节数) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;

    /* 从上次处理位置 rx2_rd 一直处理到 Size, 环形 */
    while (rx2_rd != Size) {
        feed_char(rx2_dma[rx2_rd]);
        rx2_rd++;
        if (rx2_rd >= RX2_SIZE) rx2_rd = 0;
    }
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

/* ---------- 链路在线判断 ---------- */
uint8_t vision_is_online(uint32_t timeout_ms)
{
    if (g_vision.last_rx_ms == 0) return 0;          /* 从没收到过 */
    return (HAL_GetTick() - g_vision.last_rx_ms) <= timeout_ms;
}
