#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
void var_printH(uint16_t,uint8_t,uint8_t,uint16_t,uint16_t);
void svar_printH(int32_t,uint8_t,uint16_t,uint16_t);
void txt_printH(uint8_t *, uint8_t, uint8_t, uint16_t, uint16_t);


void disp1( char RANGE ,char TIME ,char R ,char range_flag ,char tim_flag ,char r_flag )
{
	BSP_LCD_SetFont(&Font20);
//------------------------------------------------------------------------		
	if ( range_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
	switch (RANGE)	{
		case 0 : {
			txt_printH("1.5 KV ",7,11,198,320);
			break;
			}
		case 1 : {
			txt_printH("3.0 KV ",7,11,198,320);
			break;
			}
	}
//------------------------------------------------------------------------		
	if ( tim_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
	switch (TIME)	{
		case 0 : {
			txt_printH("20 ms ",6,11,198,238);
			break;
			}
		case 1 : {
			txt_printH("40 ms ",6,11,198,238);
			break;
			}
	}
//------------------------------------------------------------------------		
	if ( r_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
//--------------------------	
var_printH(R,3,11,198,167);
txt_printH("ohm",3,11,198,134);
//--------------------------	
	
}
