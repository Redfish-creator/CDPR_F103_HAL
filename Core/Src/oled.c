#include "oled.h"
#include <string.h>

static uint8_t g_oled_gram[OLED_PAGES][OLED_WIDTH];

/* ========== 6x8 点阵，只保留菜单会用到的字符 ========== */
static const uint8_t FONT_SPACE[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t FONT_MINUS[6] = {0x08,0x08,0x08,0x08,0x08,0x00};
static const uint8_t FONT_COLON[6] = {0x00,0x36,0x36,0x00,0x00,0x00};
static const uint8_t FONT_SLASH[6] = {0x20,0x10,0x08,0x04,0x02,0x00};
static const uint8_t FONT_DOT[6]   = {0x00,0x60,0x60,0x00,0x00,0x00};

static const uint8_t FONT_0[6] = {0x3E,0x51,0x49,0x45,0x3E,0x00};
static const uint8_t FONT_1[6] = {0x00,0x42,0x7F,0x40,0x00,0x00};
static const uint8_t FONT_2[6] = {0x42,0x61,0x51,0x49,0x46,0x00};
static const uint8_t FONT_3[6] = {0x21,0x41,0x45,0x4B,0x31,0x00};
static const uint8_t FONT_4[6] = {0x18,0x14,0x12,0x7F,0x10,0x00};
static const uint8_t FONT_5[6] = {0x27,0x45,0x45,0x45,0x39,0x00};
static const uint8_t FONT_6[6] = {0x3C,0x4A,0x49,0x49,0x30,0x00};
static const uint8_t FONT_7[6] = {0x01,0x71,0x09,0x05,0x03,0x00};
static const uint8_t FONT_8[6] = {0x36,0x49,0x49,0x49,0x36,0x00};
static const uint8_t FONT_9[6] = {0x06,0x49,0x49,0x29,0x1E,0x00};

static const uint8_t FONT_A[6] = {0x7E,0x11,0x11,0x11,0x7E,0x00};
static const uint8_t FONT_B[6] = {0x7F,0x49,0x49,0x49,0x36,0x00};
static const uint8_t FONT_C[6] = {0x3E,0x41,0x41,0x41,0x22,0x00};
static const uint8_t FONT_D[6] = {0x7F,0x41,0x41,0x22,0x1C,0x00};
static const uint8_t FONT_E[6] = {0x7F,0x49,0x49,0x49,0x41,0x00};
static const uint8_t FONT_F[6] = {0x7F,0x09,0x09,0x09,0x01,0x00};
static const uint8_t FONT_G[6] = {0x3E,0x41,0x49,0x49,0x7A,0x00};
static const uint8_t FONT_H[6] = {0x7F,0x08,0x08,0x08,0x7F,0x00};
static const uint8_t FONT_I[6] = {0x00,0x41,0x7F,0x41,0x00,0x00};
static const uint8_t FONT_J[6] = {0x20,0x40,0x41,0x3F,0x01,0x00};
static const uint8_t FONT_K[6] = {0x7F,0x08,0x14,0x22,0x41,0x00};
static const uint8_t FONT_L[6] = {0x7F,0x40,0x40,0x40,0x40,0x00};
static const uint8_t FONT_M[6] = {0x7F,0x02,0x0C,0x02,0x7F,0x00};
static const uint8_t FONT_N[6] = {0x7F,0x04,0x08,0x10,0x7F,0x00};
static const uint8_t FONT_O[6] = {0x3E,0x41,0x41,0x41,0x3E,0x00};
static const uint8_t FONT_P[6] = {0x7F,0x09,0x09,0x09,0x06,0x00};
static const uint8_t FONT_Q[6] = {0x3E,0x41,0x51,0x21,0x5E,0x00};
static const uint8_t FONT_R[6] = {0x7F,0x09,0x19,0x29,0x46,0x00};
static const uint8_t FONT_S[6] = {0x46,0x49,0x49,0x49,0x31,0x00};
static const uint8_t FONT_T[6] = {0x01,0x01,0x7F,0x01,0x01,0x00};
static const uint8_t FONT_U[6] = {0x3F,0x40,0x40,0x40,0x3F,0x00};
static const uint8_t FONT_V[6] = {0x1F,0x20,0x40,0x20,0x1F,0x00};
static const uint8_t FONT_W[6] = {0x3F,0x40,0x38,0x40,0x3F,0x00};
static const uint8_t FONT_X[6] = {0x63,0x14,0x08,0x14,0x63,0x00};
static const uint8_t FONT_Y[6] = {0x07,0x08,0x70,0x08,0x07,0x00};
static const uint8_t FONT_Z[6] = {0x61,0x51,0x49,0x45,0x43,0x00};
static const uint8_t FONT_QMARK[6] = {0x02,0x01,0x51,0x09,0x06,0x00};

static const uint8_t* OLED_GetFont6x8(char c)
{
    if (c >= 'a' && c <= 'z')
    {
        c = c - 'a' + 'A';
    }

    switch (c)
    {
        case ' ': return FONT_SPACE;
        case '-': return FONT_MINUS;
        case ':': return FONT_COLON;
        case '/': return FONT_SLASH;
        case '.': return FONT_DOT;

        case '0': return FONT_0;
        case '1': return FONT_1;
        case '2': return FONT_2;
        case '3': return FONT_3;
        case '4': return FONT_4;
        case '5': return FONT_5;
        case '6': return FONT_6;
        case '7': return FONT_7;
        case '8': return FONT_8;
        case '9': return FONT_9;

        case 'A': return FONT_A;
        case 'B': return FONT_B;
        case 'C': return FONT_C;
        case 'D': return FONT_D;
        case 'E': return FONT_E;
        case 'F': return FONT_F;
        case 'G': return FONT_G;
        case 'H': return FONT_H;
        case 'I': return FONT_I;
        case 'J': return FONT_J;
        case 'K': return FONT_K;
        case 'L': return FONT_L;
        case 'M': return FONT_M;
        case 'N': return FONT_N;
        case 'O': return FONT_O;
        case 'P': return FONT_P;
        case 'Q': return FONT_Q;
        case 'R': return FONT_R;
        case 'S': return FONT_S;
        case 'T': return FONT_T;
        case 'U': return FONT_U;
        case 'V': return FONT_V;
        case 'W': return FONT_W;
        case 'X': return FONT_X;
        case 'Y': return FONT_Y;
        case 'Z': return FONT_Z;

        default:  return FONT_QMARK;
    }
}

static uint8_t OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2];
    buf[0] = 0x00;
    buf[1] = cmd;

    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, 2, 100) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

static void OLED_WriteData(uint8_t *data, uint16_t len)
{
    uint8_t buf[17];
    uint16_t i = 0, n = 0;

    while (i < len)
    {
        buf[0] = 0x40;
        n = 0;
        while (i < len && n < 16)
        {
            buf[n + 1] = data[i];
            i++;
            n++;
        }
        HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, n + 1, 100);
    }
}

void OLED_Init(void)
{
    HAL_Delay(100);

    OLED_WriteCmd(0xAE); // display off

    OLED_WriteCmd(0x20); OLED_WriteCmd(0x10); // page addressing mode
    OLED_WriteCmd(0xB0);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0x40);

    OLED_WriteCmd(0x81); OLED_WriteCmd(0x7F);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0xF0);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0x22);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF); // display on

    OLED_Clear();
    OLED_Refresh();
}

void OLED_Clear(void)
{
    memset(g_oled_gram, 0x00, sizeof(g_oled_gram));
}

void OLED_Refresh(void)
{
    uint8_t page;
    for (page = 0; page < OLED_PAGES; page++)
    {
        OLED_WriteCmd(0xB0 + page);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        OLED_WriteData(g_oled_gram[page], OLED_WIDTH);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t page, char ch)
{
    uint8_t i;
    const uint8_t *font;

    if (page >= OLED_PAGES || x > OLED_WIDTH - 6)
    {
        return;
    }

    font = OLED_GetFont6x8(ch);

    for (i = 0; i < 6; i++)
    {
        g_oled_gram[page][x + i] = font[i];
    }
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *str)
{
    while (*str != '\0' && x <= OLED_WIDTH - 6)
    {
        OLED_ShowChar(x, page, *str);
        x += 6;
        str++;
    }
}

void OLED_ShowLine(uint8_t line, const char *str)
{
    if (line >= OLED_PAGES)
    {
        return;
    }
    OLED_ShowString(0, line, str);
}
