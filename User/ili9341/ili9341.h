#ifndef _ILI9341_H_
#define _ILI9341_H_

#include <stdbool.h>
#include <stdint.h>

extern uint16_t LCD_X_LENGTH;
extern uint16_t LCD_Y_LENGTH;
extern uint8_t  ILI9341_SCAN_MODE;

void ili9341_BackLedSw      (bool sw);
void ili9341_SetDirection   (uint8_t direction);
void ili9341_Init           (void);
void ili9341_SetRange       (uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void ili9341_PrintImage(uint16_t width, uint16_t height, uint16_t (*pixelData)());



#endif
