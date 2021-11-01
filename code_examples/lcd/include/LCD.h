/*****************************************************************************
*
* File                : LCD.h
* Hardware Environment: Raspberry Pi
* Build Environment   : GCC
* Version             : V0.0.1b
*
*              (c) Copyright 2021, domhathair
*              
******************************************************************************/

#ifndef _LCD_H_
#define _LCD_H_

#define SS	6	/*WPi 6, physical 22*/

/* 
 * Command set list: https://digilent.com/reference/pmod/pmod/cls/user_guide
*/

#define LCD_SAVE_CURSOR_POSITION				2, 's'
#define LCD_RESTORE_CURSOR_POSITION				2, 'u'
#define LCD_CLEAR_DISPLAY						2, 'j'
#define LCD_RESET_DISPLAY						2, '*'
#define LCD_ENABLE_WRITE_TO_EEPROM				2, 'w'

#define LCD_ERASE_WITHIN_LINE					3, 'K'
#define LCD_ERASE_FIELD_IN_CURRENT_LINE 		3, 'N'
#define LCD_SET_DISPLAY_MODE					3, 'h'
#define LCD_SET_CURSOR_MODE						3, 'c'
#define LCD_SAVE_COMMUNICATION_MODE_TO_EEPROM	3, 'm'
#define LCD_SAVE_CURSOR_MODE_TO_EEPROM			3, 'n'
#define LCD_SAVE_DISPLAY_MODE_TO_EEPROM			3, 'o'

#define LCD_SET_CURSOR_POSITION					4,	0

void LCD_send_command(int n, ...);
int LCD_begin();
void LCD_print(char* string);

#endif
