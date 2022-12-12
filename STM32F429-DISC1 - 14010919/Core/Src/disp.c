#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
void var_printH(uint16_t,uint8_t,uint8_t,uint16_t,uint16_t);
void svar_printH(int32_t,uint8_t,uint16_t,uint16_t);
void txt_printH(uint8_t *, uint8_t, uint8_t, uint16_t, uint16_t);


void disp( char MODE ,uint8_t CURVE_TYPE ,uint16_t BPM ,uint16_t WIDTH ,char AMPL ,char mod_flag ,char wav_flag ,char bpm_flag ,char wid_flag ,char amp_flag )
{
	char strCT[6];
	int a=9;
	BSP_LCD_SetFont(&Font20);
//--------------------------	
	if ( mod_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
//--------------------------	
	switch (MODE)	{
		case 0 : {
			txt_printH("ECG-SIM",7,11,220,320);
			break;
			}
		case 1 : {
			txt_printH("DEFITST",7,11,220,320);
			break;
			}
	}
//--------------------------	
	if ( wav_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
//--------------------------	
	sprintf(strCT , "WAV %2d",CURVE_TYPE);
			txt_printH(strCT,6,11,220,238);
	
//	switch (CURVE_TYPE)	{
//		case 0 : {
//			//txt_printH("QRS   ",6,11,220,238);
//			sprintf(strCT , "WAV %2d",CURVE_TYPE);
//			txt_printH(strCT,6,11,220,238);
//			break;
//		}
//		case 1 : {
//			txt_printH("SINUS ",6,11,220,238);
//			break;
//		}
//		case 2 : {
//			txt_printH("SINSQU",6,11,220,238);
//			break;
//		}
//		case 3 : {
//			txt_printH("TRIANG",6,11,220,238);
//			break;
//		}
//		case 4 : {
//			txt_printH("RETANG",6,11,220,238);
//			break;
//		}
//		case 5 : {
//			txt_printH("TRAPOS",6,11,220,238);
//			break;
//		}
//		case 6 : {
//			txt_printH("ISO   ",6,11,220,238);
//			break;
//		}
//		case 7 : {
//			txt_printH("FIBRIL",6,11,220,238);
//			break;
//		}
//		case 8 : {
//			txt_printH("TACHI ",6,11,220,238);
//			break;
//		}
//	}	
//--------------------------	
	if ( bpm_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
//--------------------------	
var_printH(BPM,3,11,220,167);
txt_printH("BPM",3,11,220,134);
//--------------------------	
	if ( wid_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
//--------------------------	
var_printH(WIDTH,3,11,220,96);
txt_printH("ms",2,11,220,62);
//--------------------------	
	if ( amp_flag == 0 )	{
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//set text color 
	}
	else	{
		BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
	}	
//--------------------------	
var_printH(AMPL,1,11,220,35);
txt_printH("mv",2,11,220,24);

}
