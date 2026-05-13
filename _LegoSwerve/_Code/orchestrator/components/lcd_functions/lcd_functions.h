#ifndef LCD_FUNCTIONS_H
#define LCD_FUNCTIONS_H

#include <stdint.h>

#define LCD_ADDR 0x27
#define RS 0x01
#define RW 0x02
#define EN 0x04
#define BL 0x08

void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(const char *str);
void LCD_SetCursor(uint8_t line, uint8_t col);
void LCD_Print_Centered(const char *str, int line);
void LCD_Clear_Line(int line);
void LCD_Init(void);

#endif // LCD_FUNCTIONS_H