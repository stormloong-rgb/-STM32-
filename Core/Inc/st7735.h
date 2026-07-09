#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

#define ST7735_WIDTH   128
#define ST7735_HEIGHT  160

#define ST7735_CS_PIN   GPIO_PIN_4
#define ST7735_DC_PIN  GPIO_PIN_1
#define ST7735_RES_PIN GPIO_PIN_0
#define ST7735_BLK_PIN GPIO_PIN_12

#define ST7735_CS_PORT   GPIOA
#define ST7735_DC_PORT  GPIOB
#define ST7735_RES_PORT GPIOB
#define ST7735_BLK_PORT GPIOB

#define ST7735_COLOR_BLACK       0x0000
#define ST7735_COLOR_WHITE       0xFFFF
#define ST7735_COLOR_RED         0xF800
#define ST7735_COLOR_GREEN       0x07E0
#define ST7735_COLOR_BLUE       0x001F
#define ST7735_COLOR_YELLOW      0xFFE0
#define ST7735_COLOR_ORANGE      0xFA20
#define ST7735_COLOR_CYAN       0x07FF
#define ST7735_COLOR_MAGENTA    0xF81F
#define ST7735_COLOR_GRAY       0x8410
#define ST7735_COLOR_LIGHT_GRAY 0xC618
#define ST7735_COLOR_PURPLE     0x780F
#define ST7735_COLOR_PINK       0xF81F
#define ST7735_COLOR_DARK_GREEN 0x03E0

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint16_t color;
} ST7735_AreaTypeDef;

void ST7735_Init(void);
void ST7735_Reset(void);
void ST7735_WriteCommand(uint8_t cmd);
void ST7735_WriteData(uint8_t data);
void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ST7735_FillColor(uint16_t color);
void ST7735_FillArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg_color, uint16_t bg_color);
void ST7735_DrawChinese(uint16_t x, uint16_t y, uint8_t index, uint16_t fg_color, uint16_t bg_color);
void ST7735_DrawString(uint16_t x, uint16_t y, char *str, uint16_t fg_color, uint16_t bg_color);
void ST7735_DrawNumber(uint16_t x, uint16_t y, int num, uint16_t fg_color, uint16_t bg_color);
void ST7735_DisplayOn(void);
void ST7735_DisplayOff(void);

void ST7735_DisplayParkingStatus(uint8_t parking1_occupied, uint8_t parking2_occupied,
                                   uint8_t parking1_number, uint8_t parking2_number);
/* 局部刷新停车页中的 FREE/P1/P2 区域，不重绘整屏 */
void ST7735_RefreshParkingStatusArea(uint8_t parking1_occupied, uint8_t parking2_occupied);
void ST7735_DisplayWelcome(void);
void ST7735_DisplayNavigation(uint8_t i, uint8_t j, char *license_plate);
void ST7735_DisplayBilling(char *license_plate, uint32_t fee);

void ST7735_DrawIcon(uint16_t x, uint16_t y, uint8_t icon_id, uint16_t color);

void ST7735_Test(void);

#endif
