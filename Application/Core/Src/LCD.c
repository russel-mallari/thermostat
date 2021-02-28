/*******************************************************************
 * 	LCD module
 *
 * 	The 16x2 LCD Display uses I2C to communicate with the microcontroller.
 *
 *  The LCD data will be transmitted on 4-bit mode. The first nibble transmitted is the upper nibble (D7-D4).
 *  While the second nibble transmitted is the lower nibble (D3-D0).
 *
 *  The backlight will be always on.
 *
 ********************************************************************/

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "LCD.h"

/**********************************/

#define BACKLIGHT 	(1U << 3)
#define EN_LCD  	(1U << 2)
#define RW_LCD		(1U << 1)
#define RS_LCD      (1U << 0)
#define CTRL_PINS_LCD (EN_LCD | RW_LCD | RS_LCD)

#define I2C_EXP_ADDRESS	(0x27 << 1)		// The shift by 1 is required by I2C interface.

/**********************************/

extern I2C_HandleTypeDef hi2c3;

/**********************************/

static void LCD_write_command(uint8_t lcd_data);

/*******************************************************************
 * This function will transmit command to the LCD.
 * The RW pin is held low, while the RS pin is held low also to signify transmit command.
 *
 * The LCD command will be transmitted on two consecutive nibbles.
 *******************************************************************/
static void LCD_write_command(uint8_t lcd_data)
{
  uint8_t iic_data;
  uint32_t high_nibble_lcd;
  uint32_t low_nibble_lcd;

  high_nibble_lcd = (lcd_data & 0xF0);
  low_nibble_lcd = (lcd_data & 0x0F) << 4;

  HAL_Delay(2);

  iic_data = (high_nibble_lcd & ~(CTRL_PINS_LCD)) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  // Start of the enable pulse
  iic_data = high_nibble_lcd | EN_LCD | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  HAL_Delay(1);

  iic_data = (low_nibble_lcd & ~EN_LCD) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);
  // End of the enable pulse

  HAL_Delay(5);

  iic_data = (low_nibble_lcd & ~(CTRL_PINS_LCD)) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  // Start of the enable pulse
  iic_data = low_nibble_lcd | EN_LCD | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  HAL_Delay(1);

  iic_data = (low_nibble_lcd & ~EN_LCD) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);
  // End of the enable pulse
}


/*************************************************************************
 * This function will transmit data(character) to the LCD.
 * The RW pin is held low, while the RS pin is held high to signify transmit characters.
 *
 * The LCD command will be transmitted on two consecutive nibbles.
 *************************************************************************/
void LCD_write_char(uint8_t lcd_data)
{
  uint8_t iic_data;
  uint32_t high_nibble_lcd;
  uint32_t low_nibble_lcd;

  high_nibble_lcd = (lcd_data & 0xF0);
  low_nibble_lcd = (lcd_data & 0x0F) << 4;

  HAL_Delay(2);


  iic_data = high_nibble_lcd | (RS_LCD & ~EN_LCD) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  // Start of the enable pulse
  iic_data = high_nibble_lcd | RS_LCD  | EN_LCD | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  HAL_Delay(1);

  iic_data = high_nibble_lcd | (RS_LCD & ~EN_LCD) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);
  // End of the enable pulse

  // Start of the enable pulse
  iic_data = low_nibble_lcd | RS_LCD  | EN_LCD | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);

  HAL_Delay(1);

  iic_data = low_nibble_lcd | (RS_LCD & ~EN_LCD) | BACKLIGHT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_EXP_ADDRESS, &iic_data, 1, HAL_MAX_DELAY);
  // End of the enable pulse
}


void LCD_write_word(char lcd_word[])
{
  char *char_ptr;
  char_ptr = lcd_word;

  while(*char_ptr != '\0')
  {
	LCD_write_char(*char_ptr);
    ++char_ptr;
  }
}

/*********************************************************************
 * This function will initialize the mode, display, and cursor of LCD
 *
 ********************************************************************/
void LCD_init(void)
{
	LCD_write_command(0x33);	// 4 bit mode initialization starts.
	LCD_write_command(0x32);
	LCD_write_command(0x28);
	LCD_write_command(0x01);	// clear display
	LCD_write_command(0x0C);  	// 4 bit mode initialization end. Display on, cursor off, blinking cursor position off.
	LCD_write_command(0x06);	// Increment cursor by 1 and no shift.
	LCD_write_command(0x80);	// DDRAM data is sent instead of CGRAM.
}

void LCD_clear_display(void)
{
	LCD_write_command(0x01);
}

void LCD_goto(uint8_t row, uint8_t column)
{
  if(row == 0)
  {
	  LCD_write_command(0x80 + column);
  }
  else if(row == 1)
  {
	  LCD_write_command(0xC0 + column);
  }
}


