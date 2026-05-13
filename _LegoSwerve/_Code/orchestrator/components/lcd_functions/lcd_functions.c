#include "lcd_functions.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern i2c_master_dev_handle_t i2c_lcd_dev_handle;

void LCD_ExpanderWrite(uint8_t data)
{
    uint8_t tx = data | BL;
    i2c_master_transmit(i2c_lcd_dev_handle, &tx, 1, -1);
}

void LCD_PulseEnable(uint8_t data)
{
    LCD_ExpanderWrite(data | EN);
    esp_rom_delay_us(1);

    LCD_ExpanderWrite(data & ~EN);
    esp_rom_delay_us(50);
}


void LCD_Write4Bits(uint8_t nibble)
{
    LCD_ExpanderWrite(nibble);
    LCD_PulseEnable(nibble);
}


void LCD_SendCommand(uint8_t cmd)
{
    uint8_t high = cmd & 0xF0;
    uint8_t low  = (cmd << 4) & 0xF0;

    LCD_Write4Bits(high);
    LCD_Write4Bits(low);

}

void LCD_SendData(uint8_t data)
{
    uint8_t high = (data & 0xF0) | RS;
    uint8_t low  = ((data << 4) & 0xF0) | RS;

    LCD_Write4Bits(high);
    LCD_Write4Bits(low);

}


void LCD_Print(const char *str)
{
    while (*str)
    {
        LCD_SendData((uint8_t)*str++);
    }
}

// Set cursor to line (1-4) and column (0-19)
void LCD_SetCursor(uint8_t line, uint8_t col)
{
    uint8_t addr;

    switch (line)
    {
        case 1: addr = 0x00; break;
        case 2: addr = 0x40; break;
        case 3: addr = 0x14; break;
        case 4: addr = 0x54; break;
        default: addr = 0x00; break;
    }

    LCD_SendCommand(0x80 | (addr + col));
}


void LCD_Print_Centered(const char *str, int line)
{
    int len = 0;
    const char *ptr = str;

    while (*ptr++) len++;   // calculate string length

    int col = 0;
    if (len < 20) {
        col = (20 - len) / 2;   // starting column to center
    }

    LCD_SetCursor(line, col);
    LCD_Print(str);
}

void LCD_Clear_Line(int line)
{
    LCD_SetCursor(line, 0);   // move to start of the line

    for (int i = 0; i < 20; i++)   // overwrite 20 characters
    {
        LCD_SendData(' ');
    }
    LCD_SetCursor(line, 0);   // return cursor to start
}

void LCD_Init(void)
{
    // Wait for LCD to power up
    vTaskDelay(pdMS_TO_TICKS(50));

    // Force 8-bit mode 3 times
    LCD_Write4Bits(0x30);
    vTaskDelay(pdMS_TO_TICKS(5));

    LCD_Write4Bits(0x30);
    esp_rom_delay_us(150);

    LCD_Write4Bits(0x30);
    esp_rom_delay_us(150);

    // Switch to 4-bit mode
    LCD_Write4Bits(0x20);
    esp_rom_delay_us(150);

    // Function set: 4-bit, 2-line, 5x8 dots
    LCD_SendCommand(0x28);

    // Display ON, cursor OFF, blink OFF
    LCD_SendCommand(0x0C);

    // Entry mode: increment cursor, no shift
    LCD_SendCommand(0x06);

    // Clear display
    LCD_SendCommand(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}
