#include <stdint.h>

#ifndef LED_H
#define LED_H


void LCD_write_char(uint8_t lcd_data);
void LCD_write_word(char lcd_word[]);
void LCD_init(void);
void LCD_clear_display(void);
void LCD_goto(uint8_t row, uint8_t column);

#endif	// End of LED_H
